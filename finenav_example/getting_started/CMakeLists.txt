add_executable(hello_finenav main.cpp)

# The algorithm .so's (planner, map, world_view) are deliberately NOT linked here:
# a self-registering library the executable references no symbol from is dropped
# by --as-needed, silently skipping its static-init registrar.  Instead they are
# dlopened via FineNavEngine::loadAlgoPlugins() (see main.cpp).  The name-only
# user needs NO algorithm headers at all — everything is resolved by NAME.
target_link_libraries(hello_finenav
    Eigen3::Eigen)
ament_target_dependencies(hello_finenav rclcpp finenav_core finenav_util finenav_engine_cpp geometry_msgs)
install(TARGETS hello_finenav DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME}/getting_started)
install(DIRECTORY params DESTINATION share/${PROJECT_NAME}/getting_started)
