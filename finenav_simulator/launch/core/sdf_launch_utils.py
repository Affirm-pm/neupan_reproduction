import os
import sys
import math
import copy
import xml.etree.ElementTree as ET

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PACKAGE_ROOT_CANDIDATES = [
    os.path.abspath(os.path.join(_THIS_DIR, "..", "..")),
    os.path.abspath(os.path.join(_THIS_DIR, "..", "..", "..")),
]
for _package_root in _PACKAGE_ROOT_CANDIDATES:
    _third_party_root = os.path.join(_package_root, "third_party")
    for _vendor in ("xmacro", "sdformat_tools"):
        _vendor_path = os.path.join(_third_party_root, _vendor)
        if os.path.isdir(_vendor_path) and _vendor_path not in sys.path:
            sys.path.insert(0, _vendor_path)

import xacro
from urdf_generator import UrdfGenerator
from xmacro.xmacro4sdf import XMLMacro4sdf


def _identity_pose():
    return (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)


def _parse_pose(pose_elem):
    if pose_elem is None or pose_elem.text is None:
        return _identity_pose(), None

    values = [float(v) for v in pose_elem.text.split()]
    if len(values) != 6:
        raise ValueError(f"Expected 6 pose values, got {len(values)} from '{pose_elem.text}'")
    return tuple(values), pose_elem.get("relative_to")


def _parse_int_text(text, default="0"):
    value = text if text is not None else default
    return int(float(value))


def _rpy_to_matrix(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _matrix_multiply(a, b):
    return [
        [
            a[row][0] * b[0][col] + a[row][1] * b[1][col] + a[row][2] * b[2][col]
            for col in range(3)
        ]
        for row in range(3)
    ]


def _matrix_vector_multiply(matrix, vector):
    return [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]


def _matrix_transpose(matrix):
    return [[matrix[col][row] for col in range(3)] for row in range(3)]


def _matrix_to_rpy(matrix):
    if abs(matrix[2][0]) < 1.0:
        pitch = math.asin(-matrix[2][0])
        roll = math.atan2(matrix[2][1], matrix[2][2])
        yaw = math.atan2(matrix[1][0], matrix[0][0])
        return roll, pitch, yaw

    pitch = math.pi / 2 if matrix[2][0] <= -1.0 else -math.pi / 2
    roll = math.atan2(-matrix[0][1], matrix[1][1])
    yaw = 0.0
    return roll, pitch, yaw


def _compose_pose(parent_pose, child_pose):
    parent_rotation = _rpy_to_matrix(*parent_pose[3:])
    child_rotation = _rpy_to_matrix(*child_pose[3:])
    child_translation = list(child_pose[:3])

    rotated_child = _matrix_vector_multiply(parent_rotation, child_translation)
    translation = tuple(parent_pose[i] + rotated_child[i] for i in range(3))
    rotation = _matrix_to_rpy(_matrix_multiply(parent_rotation, child_rotation))
    return (*translation, *rotation)


def _invert_pose(pose):
    rotation = _rpy_to_matrix(*pose[3:])
    rotation_inv = _matrix_transpose(rotation)
    translation = [-pose[0], -pose[1], -pose[2]]
    translated = _matrix_vector_multiply(rotation_inv, translation)
    rotation_rpy = _matrix_to_rpy(rotation_inv)
    return (*translated, *rotation_rpy)


def _relative_pose(parent_pose, child_pose):
    return _compose_pose(_invert_pose(parent_pose), child_pose)


def _find_link_sensor(model, sensor_name):
    for link in model.findall("link"):
        sensor = link.find(f"./sensor[@name='{sensor_name}']")
        if sensor is not None:
            return link, sensor
    raise KeyError(f"Could not find sensor '{sensor_name}' in SDF model")


def _build_entity_graph(model):
    entities = {}
    fixed_joint_children = {}

    for joint in model.findall("joint"):
        if joint.get("type") != "fixed":
            continue
        child = joint.findtext("child")
        parent = joint.findtext("parent")
        if not child or not parent:
            continue
        joint_pose, joint_relative_to = _parse_pose(joint.find("pose"))
        fixed_joint_children[child] = {
            "parent": joint_relative_to or parent,
            "pose": joint_pose,
        }

    for link in model.findall("link"):
        pose, relative_to = _parse_pose(link.find("pose"))
        parent = relative_to
        if parent is None and link.get("name") in fixed_joint_children:
            parent = fixed_joint_children[link.get("name")]["parent"]
            pose = fixed_joint_children[link.get("name")]["pose"]
        entities[link.get("name")] = {
            "parent": parent,
            "pose": pose,
        }

    for frame in model.findall("frame"):
        pose, relative_to = _parse_pose(frame.find("pose"))
        parent = relative_to or frame.get("attached_to")
        entities[frame.get("name")] = {
            "parent": parent,
            "pose": pose,
        }

    for link in model.findall("link"):
        link_name = link.get("name")
        for sensor in link.findall("sensor"):
            pose, relative_to = _parse_pose(sensor.find("pose"))
            entities[sensor.get("name")] = {
                "parent": relative_to or link_name,
                "pose": pose,
            }

    return entities


def _resolve_entity_pose(entities, entity_name, cache=None):
    if cache is None:
        cache = {}
    if entity_name in cache:
        return cache[entity_name]
    if entity_name not in entities:
        raise KeyError(f"Unknown SDF entity '{entity_name}'")

    entity = entities[entity_name]
    if entity["parent"] is None:
        cache[entity_name] = entity["pose"]
        return cache[entity_name]

    parent_pose = _resolve_entity_pose(entities, entity["parent"], cache)
    cache[entity_name] = _compose_pose(parent_pose, entity["pose"])
    return cache[entity_name]


def _pose_to_tf_args(parent_frame, child_frame, pose):
    x, y, z, roll, pitch, yaw = pose
    return [
        "--x", f"{x}",
        "--y", f"{y}",
        "--z", f"{z}",
        "--roll", f"{roll}",
        "--pitch", f"{pitch}",
        "--yaw", f"{yaw}",
        "--frame-id", parent_frame,
        "--child-frame-id", child_frame,
    ]


def load_robot_sdf_config(sdf_path):
    with open(sdf_path, "r") as sdf_file:
        return load_robot_sdf_config_from_string(sdf_file.read())


def build_tf_args(parent_frame, child_frame, pose):
    return _pose_to_tf_args(parent_frame, child_frame, pose)


def generate_robot_sdf(xacro_path, mappings=None):
    doc = xacro.process_file(xacro_path, mappings=mappings or {})
    return doc.toxml()


def generate_robot_sdf_from_xmacro(xmacro_path, mappings=None):
    xmacro_generator = XMLMacro4sdf()
    xmacro_generator.set_xml_file(xmacro_path)
    xmacro_generator.generate(mappings or {})
    return xmacro_generator.to_string()


def generate_robot_urdf_from_sdf(sdf_xml):
    urdf_generator = UrdfGenerator()
    urdf_generator.parse_from_sdf_string(sdf_xml)
    return urdf_generator.to_string()


def _parse_urdf_vector(origin_elem, attribute_name):
    if origin_elem is None:
        return (0.0, 0.0, 0.0)
    values = origin_elem.get(attribute_name, "0 0 0").split()
    return tuple(float(v) for v in values)


def _set_urdf_origin(origin_elem, xyz=None, rpy=None):
    if xyz is not None:
        origin_elem.set("xyz", " ".join(f"{value:g}" for value in xyz))
    if rpy is not None:
        origin_elem.set("rpy", " ".join(f"{value:g}" for value in rpy))


def _ensure_origin(parent_elem):
    origin = parent_elem.find("origin")
    if origin is None:
        origin = ET.Element("origin")
        parent_elem.insert(0, origin)
    return origin


def normalize_robot_urdf_for_rviz(urdf_xml):
    root = ET.fromstring(urdf_xml)

    links = {link.get("name"): link for link in root.findall("link")}
    joints = {joint.get("name"): joint for joint in root.findall("joint")}

    base_origin_link = links.get("base_origin")
    base_link = links.get("base_link")
    base_origin_to_base_link = joints.get("base_origin_to_base_link")
    if base_origin_link is None or base_link is None or base_origin_to_base_link is None:
        return urdf_xml

    base_shift_xyz = _parse_urdf_vector(base_origin_to_base_link.find("origin"), "xyz")
    # Shift content from the Gazebo body link into base_link so RViz keeps a valid TF tree.
    base_shift = tuple(-value for value in base_shift_xyz)

    for child in list(base_link):
        base_link.remove(child)

    for child_tag in ("inertial", "collision", "visual"):
        child_elem = base_origin_link.find(child_tag)
        if child_elem is None:
            continue
        child_copy = copy.deepcopy(child_elem)
        origin = _ensure_origin(child_copy)
        xyz = _parse_urdf_vector(origin, "xyz")
        _set_urdf_origin(origin, xyz=(xyz[0] + base_shift[0], xyz[1] + base_shift[1], xyz[2] + base_shift[2]))
        base_link.append(child_copy)

    base_lidar_link = links.get("base_lidar")
    base_lidar_joint = joints.get("base_lidar_joint")
    if base_lidar_link is not None and base_lidar_joint is not None:
        link_origin = base_lidar_link.find("origin")
        if link_origin is not None:
            base_lidar_link.remove(link_origin)
        base_lidar_joint.find("parent").set("link", "base_link")
        joint_origin = _ensure_origin(base_lidar_joint)
        xyz = _parse_urdf_vector(joint_origin, "xyz")
        _set_urdf_origin(joint_origin, xyz=(xyz[0] + base_shift[0], xyz[1] + base_shift[1], xyz[2] + base_shift[2]))

    for joint_name, link_name in (
        ("left_front_joint", "left_front_link"),
        ("left_rear_joint", "left_rear_link"),
        ("right_front_joint", "right_front_link"),
        ("right_rear_joint", "right_rear_link"),
    ):
        joint = joints.get(joint_name)
        link = links.get(link_name)
        if joint is None or link is None:
            continue

        link_origin = link.find("origin")
        if link_origin is None:
            continue

        xyz = _parse_urdf_vector(link_origin, "xyz")
        rpy = _parse_urdf_vector(link_origin, "rpy")
        link.remove(link_origin)

        for child_tag in ("inertial", "visual", "collision"):
            child_elem = link.find(child_tag)
            if child_elem is None:
                continue
            child_origin = _ensure_origin(child_elem)
            _set_urdf_origin(child_origin, xyz=(0.0, 0.0, 0.0), rpy=rpy)

        joint.find("parent").set("link", "base_link")
        joint_origin = _ensure_origin(joint)
        _set_urdf_origin(
            joint_origin,
            xyz=(xyz[0] + base_shift[0], xyz[1] + base_shift[1], xyz[2] + base_shift[2]),
            rpy=(0.0, 0.0, 0.0),
        )

    root.remove(base_origin_link)
    root.remove(base_origin_to_base_link)

    return ET.tostring(root, encoding="unicode")


def load_robot_sdf_config_from_string(sdf_xml):
    root = ET.fromstring(sdf_xml)
    model = root.find("model")
    if model is None:
        raise RuntimeError("Could not find <model> in generated SDF XML")

    entities = _build_entity_graph(model)
    cache = {}

    base_link_pose = _resolve_entity_pose(entities, "base_link", cache)
    lidar_frame_pose = _resolve_entity_pose(entities, "lidar_frame", cache)
    imu_pose = _resolve_entity_pose(entities, "imu_sensor", cache)

    lidar_parent_link, lidar_sensor = _find_link_sensor(model, "gpu_lidar")
    imu_parent_link, imu_sensor = _find_link_sensor(model, "imu_sensor")

    horizontal = lidar_sensor.find("./ray/scan/horizontal")
    vertical = lidar_sensor.find("./ray/scan/vertical")
    lidar_range = lidar_sensor.find("./ray/range")

    if horizontal is None or vertical is None or lidar_range is None:
        raise RuntimeError("Could not find complete GPU LiDAR scan configuration in SDF")

    update_rate = float(lidar_sensor.findtext("update_rate", default="0") or 0.0)
    scan_period = 1.0 / update_rate if update_rate > 0.0 else 0.1

    return {
        "base_link_to_lidar": _relative_pose(base_link_pose, lidar_frame_pose),
        "base_link_to_imu": _relative_pose(base_link_pose, imu_pose),
        "lidar_parent_link": lidar_parent_link.get("name"),
        "lidar_sensor_name": lidar_sensor.get("name"),
        "imu_parent_link": imu_parent_link.get("name"),
        "imu_sensor_name": imu_sensor.get("name"),
        "lidar_vertical_samples": _parse_int_text(vertical.findtext("samples", default="0")),
        "lidar_horizontal_samples": _parse_int_text(horizontal.findtext("samples", default="0")),
        "lidar_vertical_min_angle": float(vertical.findtext("min_angle", default="0.0")),
        "lidar_vertical_max_angle": float(vertical.findtext("max_angle", default="0.0")),
        "lidar_horizontal_min_angle": float(horizontal.findtext("min_angle", default="0.0")),
        "lidar_horizontal_max_angle": float(horizontal.findtext("max_angle", default="0.0")),
        "lidar_min_range": float(lidar_range.findtext("min", default="0.0")),
        "lidar_max_range": float(lidar_range.findtext("max", default="0.0")),
        "lidar_scan_period": scan_period,
    }
