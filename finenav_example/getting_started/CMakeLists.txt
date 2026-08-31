add_executable(hello_finenav main.cpp)

# The algorithm .so's are deliberately NOT linked here.  A self-registering
# library that the executable references no symbol from is dropped by
# --as-needed, silently skipping its static-init registrar.  Instead the
# libraries are dlopened via FineNavEngine::loadAlgoPlugins() (see main.cpp),
# which forces their registrars to run.  For the typed recovery below we only
# need the algorithm headers; the generated parameter headers come from the
# params targets, and RTTI unification across the DSO boundary is handled by
# RTLD_GLOBAL inside BT::SharedLibrary.
target_include_directories(hello_finenav PRIVATE
    ../create_custom_map/include
    ../create_custom_planner/include)
target_link_libraries(hello_finenav
    dummy_map_params
    dummy_planner_params
    Eigen3::Eigen)
ament_target_dependencies(hello_finenav rclcpp finenav_core finenav_util finenav_engine geometry_msgs)
install(TARGETS hello_finenav DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME}/getting_started)
install(DIRECTORY params DESTINATION share/${PROJECT_NAME}/getting_started)
