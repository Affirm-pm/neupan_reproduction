cmake_minimum_required(VERSION 3.22)
project(finenav_interfaces)

find_package(ament_cmake_auto REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
        msg/PlanResult.msg
        msg/MapCell.msg
        msg/MapChunk.msg
        action/ComputePlan.action
        DEPENDENCIES geometry_msgs # Add packages that above messages depend on, in this case geometry_msgs for Sphere.msg
)

ament_auto_package()

