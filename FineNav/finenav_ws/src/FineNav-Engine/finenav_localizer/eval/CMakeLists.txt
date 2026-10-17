find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

# ── finenav_localizer_node — entry point for eval ─────────────────────────
add_executable(finenav_localizer_node localizer_node.cpp)
target_include_directories(finenav_localizer_node PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)
ament_target_dependencies(finenav_localizer_node rclcpp)
target_link_libraries(finenav_localizer_node finenav_localizer)

install(TARGETS finenav_localizer_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

install(FILES config/eval_params.yaml DESTINATION share/${PROJECT_NAME}/eval/config)
install(DIRECTORY test_bag1 test_bag2 test_bag3 DESTINATION share/${PROJECT_NAME}/eval)
install(FILES record_eval.py plot.py DESTINATION share/${PROJECT_NAME}/eval)
install(FILES launch/eval.launch.py DESTINATION share/${PROJECT_NAME})
