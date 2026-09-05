cmake_minimum_required(VERSION 3.22)
project(finenav_core)

# TODO: 设置编译选项和Cpp标准
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(finenav_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(finenav_util REQUIRED)
find_package(finenav_visualizer REQUIRED)


set(ros_dependencies
        eigen3_cmake_module
        rclcpp
        rclcpp_action
        finenav_interfaces
        geometry_msgs
        finenav_util
        finenav_visualizer
)

add_library(${PROJECT_NAME} SHARED
        src/common/plan_registry.cpp
        src/common/map_registry.cpp
        src/common/mapview_registry.cpp
        src/robot_state/robot_state_provider.cpp
        src/task/task_channel.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_20)
target_link_libraries(${PROJECT_NAME} PUBLIC
        fmt::fmt
        Eigen3::Eigen
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC
        ${ros_dependencies}
)

install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(
        fmt
        Eigen3
        ${ros_dependencies}
)

if(BUILD_TESTING)
        find_package(ament_cmake_gtest REQUIRED)
        ament_find_gtest()
        add_subdirectory(test)
endif()

ament_package()
