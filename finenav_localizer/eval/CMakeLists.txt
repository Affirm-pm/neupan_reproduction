find_package(rosbag2_cpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_executable(${PROJECT_NAME}_eval
        eval_scenario.cpp
)
target_include_directories(${PROJECT_NAME}_eval PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
get_filename_component(_pkg_dir ${CMAKE_CURRENT_SOURCE_DIR} DIRECTORY)
target_compile_definitions(${PROJECT_NAME}_eval PRIVATE
        FINENAV_LOCALIZER_PACKAGE_DIR=${_pkg_dir}
)
target_link_libraries(${PROJECT_NAME}_eval
        finenav_localizer
)
ament_target_dependencies(${PROJECT_NAME}_eval
        rosbag2_cpp
        nav_msgs
        tf2_msgs
)
