#!/usr/bin/python3
"""Rasterize supported static maze worlds from the same collision geometry as Gazebo."""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


SUPPORTED_WORLDS = ('flat_small_scale_maze.world', 'maze_with_obstacles.sdf')


def compose(parent, element):
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('relative_to poses are not supported')
    values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid SDF pose')
    x, y, z, roll, pitch, yaw = values
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                            math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw))
    transform = np.eye(4)
    transform[:3, :3] = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]]
    transform[:3, 3] = [x, y, z]
    return parent @ transform


def collision_shapes(element, resource_dir, parent=None, depth=0):
    if depth > 12:
        raise ValueError('cyclic or deeply nested SDF includes')
    pose = compose(np.eye(4) if parent is None else parent, element)
    if element.tag == 'collision':
        geometry = element.find('geometry')
        if geometry is None or len(geometry) != 1:
            raise ValueError('expected one collision geometry')
        shape = geometry[0]
        if shape.tag == 'box':
            size = tuple(float(v) for v in shape.findtext('size', '').split())
            if len(size) != 3:
                raise ValueError('box requires three dimensions')
        elif shape.tag == 'cylinder':
            size = (float(shape.findtext('radius', 'nan')), float(shape.findtext('length', 'nan')))
        elif shape.tag == 'sphere':
            size = (float(shape.findtext('radius', 'nan')),)
        else:
            raise ValueError(f'unsupported collision geometry: {shape.tag}')
        if not all(math.isfinite(v) and v > 0 for v in size):
            raise ValueError('invalid collision dimensions')
        # Sphere orientation is irrelevant, including spheres that rolled before saving.
        if shape.tag != 'sphere' and not np.allclose(pose[2, :3], [0, 0, 1], atol=1e-5):
            raise ValueError('only upright boxes and cylinders are supported')
        yield shape.tag, pose, size
    for child in element:
        if child.tag in ('world', 'model', 'link', 'collision'):
            yield from collision_shapes(child, resource_dir, pose, depth + 1)
        elif child.tag == 'include':
            uri = child.findtext('uri', '')
            if not uri.startswith('model://'):
                raise ValueError(f'unsupported include: {uri}')
            model_path = (resource_dir / 'model' / uri[8:] / 'model.sdf').resolve()
            if not model_path.is_relative_to((resource_dir / 'model').resolve()):
                raise ValueError('model include escapes resource directory')
            model = ET.parse(model_path).getroot()
            yield from collision_shapes(model, resource_dir, compose(pose, child), depth + 1)


def vertical_extent(shape):
    kind, pose, size = shape
    half_height = size[2]/2 if kind == 'box' else (size[1]/2 if kind == 'cylinder' else size[0])
    return pose[2, 3] - half_height, pose[2, 3] + half_height


def rasterize(shapes, resolution):
    grounds = [s for s in shapes if s[0] == 'box' and vertical_extent(s)[1] <= 0.05]
    if len(grounds) != 1 or not np.allclose(grounds[0][1][:3, :3], np.eye(3), atol=1e-6):
        raise ValueError('expected one axis-aligned ground box')
    _, ground_pose, (sx, sy, _) = grounds[0]
    ox, oy = ground_pose[0, 3] - sx/2, ground_pose[1, 3] - sy/2
    width, height = math.ceil(sx/resolution), math.ceil(sy/resolution)
    xx, yy = np.meshgrid(ox + (np.arange(width)+0.5)*resolution,
                         oy + (np.arange(height)+0.5)*resolution)
    occupied = np.zeros((height, width), dtype=np.int8)
    pad = resolution / math.sqrt(2)
    for kind, pose, size in shapes:
        low, high = vertical_extent((kind, pose, size))
        if high <= 0.15 or low >= 2.0:
            continue
        dx, dy = xx-pose[0, 3], yy-pose[1, 3]
        if kind == 'box':
            rx = pose[0, 0]*dx + pose[1, 0]*dy
            ry = pose[0, 1]*dx + pose[1, 1]*dy
            mask = (np.abs(rx) <= size[0]/2+pad) & (np.abs(ry) <= size[1]/2+pad)
        else:
            # Full horizontal sphere projection is conservative for the robot height band.
            mask = dx*dx + dy*dy <= (size[0]+pad)**2
        occupied[mask] = 100
    return occupied, ox, oy


def build_map(resource_dir, resolution=0.1, world_name='flat_small_scale_maze.world'):
    """Return raw occupancy and origin; obstacle inflation belongs to WorldView."""
    if not math.isfinite(resolution) or not 0.05 <= resolution <= 0.5:
        raise ValueError('resolution must be in [0.05, 0.5] metres')
    if world_name not in SUPPORTED_WORLDS:
        raise ValueError(f'Unsupported built-in world {world_name!r}; provide an external map')
    world = ET.parse(resource_dir / 'worlds' / world_name).getroot()
    return rasterize(list(collision_shapes(world, resource_dir)), resolution)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from ament_index_python.packages import get_package_share_directory

    rclpy.init()
    node = Node('maze_global_map')
    world = node.declare_parameter('world_name', 'flat_small_scale_maze.world').value
    resolution = node.declare_parameter('resolution', 0.1).value
    topic = node.declare_parameter('map_topic', '/global_map').value
    resource = Path(get_package_share_directory('finenav_simulator')) / 'resource'
    data, ox, oy = build_map(resource, resolution, world)
    msg = OccupancyGrid()
    msg.header.frame_id = 'map'
    msg.info.resolution = resolution
    msg.info.width, msg.info.height = data.shape[1], data.shape[0]
    msg.info.origin.position.x, msg.info.origin.position.y = ox, oy
    msg.info.origin.orientation.w = 1.0
    msg.data = data.ravel().tolist()
    publisher = node.create_publisher(OccupancyGrid, topic, QoSProfile(
        depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE))
    msg.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(msg)
    node.get_logger().info(f'Global map for {world} ready: {msg.info.width} x {msg.info.height} on {topic}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
