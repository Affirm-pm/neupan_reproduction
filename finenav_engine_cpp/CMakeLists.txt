cmake_minimum_required(VERSION 3.22)
project(finenav_engine_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nlohmann_json REQUIRED)

set(finenav_dependencies
        finenav_common
        finenav_core
        finenav_localizer
        finenav_navigator
        finenav_interfaces
        finenav_visualizer
        finenav_util
)

foreach(dep IN LISTS finenav_dependencies)
    find_package(${dep} REQUIRED)
endforeach()

add_library(finenav_engine_cpp SHARED
        src/finenav_engine.cpp
)
ament_target_dependencies(finenav_engine_cpp
        rclcpp
        rclcpp_action
        geometry_msgs
        ament_index_cpp
        behaviortree_cpp
        ${finenav_dependencies}
)
# Header-only JSON; plain signature keeps ament_target_dependencies (which uses
# the plain form internally) from clashing with a keyword signature.
target_link_libraries(finenav_engine_cpp nlohmann_json::nlohmann_json)
target_include_directories(finenav_engine_cpp PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

install(TARGETS finenav_engine_cpp
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(finenav_engine_cpp)
ament_export_dependencies(${finenav_dependencies})

ament_package()
