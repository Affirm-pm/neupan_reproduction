# ==============================================================================
#  create_custom_neupan_planner
#
#  NeuPAN (via neupan_cpp/libneupan) as a FineNav-Engine Planner plugin, plus
#  the WorldView bridge that feeds it obstacle points.  Both halves ship in this
#  one directory because the bridge exists only to serve this planner.
# ==============================================================================

find_package(generate_parameter_library REQUIRED)

# The package dependency in package.xml makes colcon build libneupan first
# and add its installation prefix to the CMake search path.
find_package(libneupan REQUIRED)

find_package(Eigen3 REQUIRED)

# Direct dependencies declared by finenav_example/package.xml.
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
# For setVisualizer(): the plugin draws the reference path / plan as markers.
# finenav_core already depends on it and exports it, but naming it here keeps
# the include path and the library explicit rather than transitive.
find_package(finenav_visualizer REQUIRED)

generate_parameter_library(neupan_planner_params neupan_planner.yaml)
generate_parameter_library(neupan_world_view_params neupan_world_view.yaml)

# ── The planner ──────────────────────────────────────────────────────────────
#
# The target is named neupan_planner, not neupan: libneupan already exports the
# imported target neupan::neupan, and shadowing its bare name here would only
# invite confusion.  The plugin's REGISTRATION name is independent of the file
# name: the resource index below tells loadAlgoPlugins() which .so to load.
add_library(neupan_planner SHARED src/neupan_planner.cpp)
target_include_directories(neupan_planner PUBLIC include)
target_link_libraries(neupan_planner
    neupan_planner_params
    neupan::neupan)
ament_target_dependencies(neupan_planner finenav_core finenav_visualizer)

# Default model is resolved relative to the installed plugin at runtime.
target_link_libraries(neupan_planner ${CMAKE_DL_LIBS})

if(BUILD_TESTING)
    # Optional development tests; keep this example's package manifest untouched.
    find_package(ament_cmake_gtest QUIET)
    if(ament_cmake_gtest_FOUND)
        ament_add_gtest(test_goal_alignment test/test_goal_alignment.cpp)
        target_include_directories(test_goal_alignment PRIVATE include)
        ament_add_gtest(test_goal_progress test/test_goal_progress.cpp)
        target_include_directories(test_goal_progress PRIVATE include)
    endif()
endif()

# ── The WorldView bridge ─────────────────────────────────────────────────────
#
# dummy_map is linked because the bridge is registered against example::DummyMap
# (FINENAV_REGISTER_WORLDVIEW needs at least one map type).  The bridge never
# reads it; see neupan_world_view.cpp.
add_library(neupan_world_view SHARED src/neupan_world_view.cpp)
target_include_directories(neupan_world_view PUBLIC include)
target_link_libraries(neupan_world_view
    neupan_world_view_params
    neupan_planner
    neupan_planner_params
    dummy_map
    dummy_map_params)
ament_target_dependencies(neupan_world_view
    finenav_core sensor_msgs std_msgs tf2 tf2_ros)

# ── Install ──────────────────────────────────────────────────────────────────
#
# Publish both libraries in the package's FineNav resource index. The engine
# loads these entries with loadAlgoPlugins({"finenav_example"}).
finenav_register_algo_plugin(neupan_planner)
finenav_register_algo_plugin(neupan_world_view)
install(TARGETS neupan_planner neupan_world_view
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# Vendor the DUNE weights into this package's share so a deployed workspace does
# not depend on libneupan's install layout.  The ROS parameter still wins.
install(FILES ${libneupan_MODELS_DIR}/diff_0p7x0p6.bin
        DESTINATION share/${PROJECT_NAME}/models)

# Simulation parameter overlay for this plugin (loaded alongside
# config/hello_finenav.yaml by hello_finenav.launch.py), plus
# the matching RViz2 view: fixed frame `map`, and its "2D Goal Pose" tool
# publishes on /goal_pose — the topic hello_finenav consumes for a goal.
install(FILES neupan_sim.yaml neupan_sim.rviz
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING AND ament_cmake_gtest_FOUND)
    ament_add_gtest(test_planner_safety test/test_planner_safety.cpp)
    target_include_directories(test_planner_safety PRIVATE include)
    ament_add_gtest(test_neupan_plugin test/test_neupan_plugin.cpp
        ENV "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:$ENV{LD_LIBRARY_PATH}")
    target_link_libraries(test_neupan_plugin neupan_planner neupan::neupan)
    target_compile_definitions(test_neupan_plugin PRIVATE
        TEST_MODEL="${libneupan_MODELS_DIR}/diff_0p7x0p6.bin")
    ament_add_gtest(test_neupan_world_view test/test_neupan_world_view.cpp
        ENV "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}:${CMAKE_BINARY_DIR}/create_custom_map:$ENV{LD_LIBRARY_PATH}")
    target_link_libraries(test_neupan_world_view neupan_world_view)
    # Mirror the installed layout for the runtime default-model test.
    file(MAKE_DIRECTORY "${CMAKE_BINARY_DIR}/share/${PROJECT_NAME}/models")
    configure_file("${libneupan_MODELS_DIR}/diff_0p7x0p6.bin"
        "${CMAKE_BINARY_DIR}/share/${PROJECT_NAME}/models/diff_0p7x0p6.bin" COPYONLY)
endif()
