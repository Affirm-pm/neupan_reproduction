cmake_minimum_required(VERSION 3.22)
project(finenav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/Trajectory.msg
        msg/RobotState.msg
        msg/NavStatus.msg
        msg/TaskCmd.msg
        msg/TaskFeedback.msg
        action/NavigateToPose.action
        DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(geometry_msgs std_msgs action_msgs)

ament_package()
