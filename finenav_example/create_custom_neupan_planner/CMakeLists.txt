# ==============================================================================
#  create_custom_neupan_planner
#
#  NeuPAN (via neupan_cpp/libneupan) as a FineNav-Engine Planner plugin, plus
#  the WorldView bridge that feeds it obstacle points.  Both halves ship in this
#  one directory because the bridge exists only to serve this planner.
# ==============================================================================

find_package(generate_parameter_library REQUIRED)

# libneupan is a plain-CMake package (its package.xml declares
# <build_type>cmake</build_type>), so it is deliberately absent from the ament
# index.  It does install a CMake package config, which is what this finds; it
# provides the imported target neupan::neupan and the variable
# libneupan_MODELS_DIR (the directory holding diff_default.bin).
find_package(libneupan QUIET)

if(NOT libneupan_FOUND)
  # finenav_example's package.xml does not declare libneupan (it depends on it
  # only here), so colcon does not add its install prefix to CMAKE_PREFIX_PATH
  # for this package.  When the workspace overlay has not been sourced, the
  # prefix has to be guessed: <ws>/FineNav-Engine/finenav_example/<this dir>
  # walked up three levels is <ws>, and the lib installs under <ws>/install.
  #
  # The permanent fix is to add <depend>libneupan</depend> to
  # finenav_example/package.xml; with that, colcon builds libneupan first and
  # exports its prefix, and this branch never runs.
  get_filename_component(_finenav_ws "${CMAKE_CURRENT_SOURCE_DIR}/../../.." ABSOLUTE)
  list(APPEND CMAKE_PREFIX_PATH "${_finenav_ws}/install/libneupan")
  find_package(libneupan REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

# finenav_example's package.xml declares none of these, so they are found here.
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
# name anyway — loadAlgoPlugins() dlopens every .so in the directory.
add_library(neupan_planner SHARED src/neupan_planner.cpp)
target_include_directories(neupan_planner PUBLIC include)
target_link_libraries(neupan_planner
    neupan_planner_params
    neupan::neupan)
ament_target_dependencies(neupan_planner finenav_core finenav_visualizer)

# The DUNE checkpoint used when neupan_planner.dune_checkpoint is left empty.
#
# diff_0p7x0p6.bin is a DUNE trained for THIS example's robot: the simulator's
# 0.7 x 0.6 m XY bounds including its wheels
# (finenav_simulator/resource/robot/model.sdf.xmacro).
# libneupan compares the weights' recorded footprint against robot_length /
# robot_width at load time and throws on a mismatch, so the default model and
# the default footprint in neupan_planner.yaml describe the same robot.  (The
# stock diff_default.bin is trained for 1.6 x 2.0 m; using it here forced an
# oversized, far too conservative footprint.)
#
# Chain: empty parameter -> this build-time default -> existence check in
# configure().  An explicit parameter always wins, so relocating the models does
# not require a rebuild.
target_compile_definitions(neupan_planner PRIVATE
    NEUPAN_DEFAULT_CHECKPOINT="${libneupan_MODELS_DIR}/diff_0p7x0p6.bin")

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
# Both .so must land in this package's lib/ next to the other example plugins:
# engine->loadAlgoPlugins({"finenav_example/lib"}) dlopens every .so it finds
# there, which is how the self-registrars fire without the executable linking
# either library.
install(TARGETS neupan_planner neupan_world_view
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# Vendor the DUNE weights into this package's share so a deployed workspace does
# not depend on libneupan's install layout.  The ROS parameter still wins.
install(DIRECTORY ${libneupan_MODELS_DIR}/
        DESTINATION share/${PROJECT_NAME}/models)

# Simulation parameter overlay for this plugin (loaded alongside
# getting_started/params/sim_localizer.yaml via a second --params-file), plus
# the matching RViz2 view: fixed frame `map`, and its "2D Goal Pose" tool
# publishes on /goal_pose — the topic hello_finenav consumes for a goal.
install(FILES neupan_sim.yaml neupan_sim.rviz
        DESTINATION share/${PROJECT_NAME})
