#!/usr/bin/python3
"""Static ground-truth map for flat_small_scale_maze.world, not a SLAM mapper.

Parse the same nested box collisions Gazebo uses. Only this known planar world
is supported; other worlds must provide their own OccupancyGrid.
"""
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


def compose(parent, element):
    pose = element.find('pose')
    if pose is not None and pose.get('relative_to'):
        raise ValueError('relative_to poses are not supported')
    values = [float(v) for v in element.findtext('pose', '0 0 0 0 0 0').split()]
    if len(values) != 6 or not all(math.isfinite(v) for v in values):
        raise ValueError('invalid SDF pose')
    x, y, z, roll, pitch, yaw = values
    if abs(roll) > 1e-6 or abs(pitch) > 1e-6:
        raise ValueError('only planar box geometry is supported')
    px, py, pz, angle = parent
    return (px + math.cos(angle)*x - math.sin(angle)*y,
            py + math.sin(angle)*x + math.cos(angle)*y, pz + z, angle + yaw)


def collision_boxes(element, resource_dir, parent=(0., 0., 0., 0.), depth=0):
    if depth > 12:
        raise ValueError('cyclic or deeply nested SDF includes')
    pose = compose(parent, element)
    if element.tag == 'collision':
        size = element.findtext('geometry/box/size')
        if size is None:
            raise ValueError('maze map requires box collision geometry')
        sizes = tuple(float(v) for v in size.split())
        if len(sizes) != 3 or not all(math.isfinite(v) and v > 0 for v in sizes):
            raise ValueError('invalid box size')
        yield (*pose, *sizes)
    for child in element:
        if child.tag in ('world', 'model', 'link', 'collision'):
            yield from collision_boxes(child, resource_dir, pose, depth + 1)
        elif child.tag == 'include':
            uri = child.findtext('uri', '')
            if not uri.startswith('model://'):
                raise ValueError(f'unsupported include: {uri}')
            model_path = (resource_dir / 'model' / uri[8:] / 'model.sdf').resolve()
            if not model_path.is_relative_to((resource_dir / 'model').resolve()):
                raise ValueError('model include escapes resource directory')
            model = ET.parse(model_path).getroot()
            yield from collision_boxes(model, resource_dir, compose(pose, child), depth + 1)


def build_map(resource_dir, resolution=0.1):
    """Return raw occupancy, origin x/y. Inflation belongs to the WorldView."""
    if not math.isfinite(resolution) or not 0.05 <= resolution <= 0.5:
        raise ValueError('resolution must be in [0.05, 0.5] metres')
    world = ET.parse(resource_dir / 'worlds' / 'flat_small_scale_maze.world').getroot()
    boxes = list(collision_boxes(world, resource_dir))
    grounds = [b for b in boxes if b[2] + b[6]/2 <= 0.05]
    if len(grounds) != 1 or abs(grounds[0][3]) > 1e-6:
        raise ValueError('expected one axis-aligned ground box')
    gx, gy, _, _, sx, sy, _ = grounds[0]
    ox, oy = gx - sx/2, gy - sy/2
    width, height = math.ceil(sx/resolution), math.ceil(sy/resolution)
    xx, yy = np.meshgrid(ox + (np.arange(width)+0.5)*resolution,
                         oy + (np.arange(height)+0.5)*resolution)
    occupied = np.zeros((height, width), dtype=np.int8)
    # Conservative rasterization includes every cell intersecting an obstacle.
    pad = resolution / math.sqrt(2)
    for x, y, z, yaw, dx, dy, dz in boxes:
        if z + dz/2 <= 0.15 or z - dz/2 >= 2.0:
            continue
        rx = math.cos(yaw)*(xx-x) + math.sin(yaw)*(yy-y)
        ry = -math.sin(yaw)*(xx-x) + math.cos(yaw)*(yy-y)
        occupied[(np.abs(rx) <= dx/2+pad) & (np.abs(ry) <= dy/2+pad)] = 100
    return occupied, ox, oy


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
    from nav_msgs.msg import OccupancyGrid
    from ament_index_python.packages import get_package_share_directory

    rclpy.init()
    node = Node('maze_global_map')
    world = node.declare_parameter('world_name', 'flat_small_scale_maze.world').value
    if world != 'flat_small_scale_maze.world':
        raise ValueError('Built-in map only supports flat_small_scale_maze.world; '
                         'set publish_global_map:=false and provide an external map')
    resolution = node.declare_parameter('resolution', 0.1).value
    topic = node.declare_parameter('map_topic', '/global_map').value
    resource = Path(get_package_share_directory('finenav_simulator')) / 'resource'
    data, ox, oy = build_map(resource, resolution)
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

    # Transient local durability supplies the static grid to late subscribers.
    msg.header.stamp = node.get_clock().now().to_msg()
    publisher.publish(msg)
    node.get_logger().info(f'Global maze map ready: {msg.info.width} x {msg.info.height} on {topic}')
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
