cmake_minimum_required(VERSION 3.22)
project(finenav_msgs)

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/Trajectory.msg
        action/ComputePlan.action
        action/NavigateToPose.action
        DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_auto_package()

