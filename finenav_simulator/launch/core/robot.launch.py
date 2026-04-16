import os
import sys
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from launch_ros.actions import Node

pkg_share_dir = get_package_share_directory("finenav_simulator")
scripts_dir = os.path.join(pkg_share_dir, "scripts")
if scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from sdf_launch_utils import (
    build_tf_args,
    generate_robot_sdf_from_xmacro,
    generate_robot_urdf_from_sdf,
    load_robot_sdf_config_from_string,
    normalize_robot_urdf_for_rviz,
)


def _load_spawn_pose(config_path, world_name):
    with open(config_path, "r", encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file) or {}

    poses = config.get("world_spawn_poses", config)
    pose = poses.get(world_name)
    if pose is None:
        pose = poses.get(os.path.splitext(world_name)[0])

    if pose is None:
        available = ", ".join(sorted(poses.keys()))
        raise RuntimeError(
            f"No spawn pose configured for world '{world_name}'. Available entries: {available}"
        )

    return {
        "x": str(float(pose["x"])),
        "y": str(float(pose["y"])),
        "z": str(float(pose["z"])),
        "yaw": str(float(pose.get("yaw", 0.0))),
    }


def _launch_setup(context):
    pkg = get_package_share_directory("finenav_simulator")

    sdf_xmacro_path = os.path.join(pkg, "resource", "robot", "model.sdf.xmacro")
    bridge_config = os.path.join(pkg, "config", "ros_gz_bridge.yaml")
    spawn_pose_config = os.path.join(pkg, "config", "world_spawn_poses.yaml")

    robot_name = "fine_robot"
    world_name = LaunchConfiguration("world_name").perform(context)
    spawn_pose = _load_spawn_pose(spawn_pose_config, world_name)
    x_pose = spawn_pose["x"]
    y_pose = spawn_pose["y"]
    z_pose = spawn_pose["z"]
    yaw = spawn_pose["yaw"]

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_xml = generate_robot_sdf_from_xmacro(sdf_xmacro_path)
    robot_urdf_xml = normalize_robot_urdf_for_rviz(generate_robot_urdf_from_sdf(robot_xml))
    sdf_config = load_robot_sdf_config_from_string(robot_xml)

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-string", robot_xml,
            "-name", robot_name,
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose,
            "-Y", yaw,
        ],
    )

    robot_ign_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=robot_name,
        parameters=[{"config_file": bridge_config}],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        remappings=remappings,
        parameters=[{
            "use_sim_time": True,
            "robot_description": robot_urdf_xml,
        }],
    )

    base_link_to_imu = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_imu",
        arguments=build_tf_args("base_link", "imu", sdf_config["base_link_to_imu"]),
        parameters=[{"use_sim_time": True}],
    )

    imu_sensor_alias = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="imu_sensor_frame_alias",
        arguments=build_tf_args(
            "imu",
            f"{robot_name}/{sdf_config['imu_parent_link']}/{sdf_config['imu_sensor_name']}",
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        parameters=[{"use_sim_time": True}],
    )

    simulator_helper = Node(
        package="finenav_simulator",
        executable="simulator_helper_node",
        parameters=[{
            "use_sim_time": True,
            "input_odom_topic": PythonExpression(["'/Odometry' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else '/odom'"]),
            "path_topic": "/path",
            "max_path_points": 2000,
            "publish_tf": PythonExpression(["'False' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else 'True'"]),
            "path_odom_frame": "odom",
            "path_base_frame": "base_link",
            "teleop_input_topic": "/keyboard/keypress",
            "teleop_output_topic": "/cmd_vel",
            "initial_linear_speed": 0.5,
            "initial_angular_speed": 1.0,
            "linear_step": 0.1,
            "angular_step": 0.1,
            "min_linear_speed": 0.0,
            "max_linear_speed": 2.0,
            "min_angular_speed": 0.0,
            "max_angular_speed": 3.0,
            "enable_keyboard_teleop": True,
            "enable_path_publisher": True,
            "enable_ground_truth_localizer": PythonExpression([
                "'False' if '", LaunchConfiguration("lio_type"), "' == 'fast_lio' else 'True'"
            ]),
        }],
    )

    return [
        spawn_robot,
        robot_ign_bridge,
        robot_state_publisher,
        base_link_to_imu,
        imu_sensor_alias,
        simulator_helper,
    ]


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(
        DeclareLaunchArgument(
            "lio_type",
            default_value="fast_lio",
            description="Choose localization chain: fast_lio or gazebo_odom",
        )
    )
    ld.add_action(
        DeclareLaunchArgument(
            "world_name",
            default_value="flat_small_scale_maze.world",
            description="World file name under finenav_simulator/resource/worlds",
        )
    )
    ld.add_action(OpaqueFunction(function=_launch_setup))
    return ld
