cmake_minimum_required(VERSION 3.22)
project(finenav_route_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(finenav_cmake REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})
ament_auto_find_build_dependencies()
find_package(generate_parameter_library REQUIRED)

# ── parameter libraries (generated from YAML) ──────────────────────────────
generate_parameter_library(astar_params  astar.yaml)
generate_parameter_library(bfs_params    bfs.yaml)
generate_parameter_library(dstarlite_params dstarlite.yaml)
generate_parameter_library(detour_params    detour.yaml)

# ── planner library ─────────────────────────────────────────────────────────
ament_auto_add_library(finenav_route_planner_lib SHARED
    # Grid search engine (A* / Dijkstra / BFS all handled by AStarAlgorithm)
    src/astar/astar_algorithm.cpp
    # D* Lite incremental search engine
    src/dstarlite/dstarlite_algorithm.cpp
    # Grid planner wrappers (FineNav PlannerConcept)
    src/astar_path_search.cpp
    src/bfs_path_search.cpp          # BFS = AStarAlgorithm(Heuristic::Zero)
    src/dstarlite_path_search.cpp    # D* Lite planner wrapper
    # Local detour planner (FineNav PlannerConcept)
    src/detour_planner.cpp
)
target_link_libraries(finenav_route_planner_lib
    astar_params
    bfs_params
    dstarlite_params
    detour_params
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

# ── unit tests ──────────────────────────────────────────────────────────────
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(astar_test
        test/astar_test.cpp
        src/astar/astar_algorithm.cpp
        src/astar_path_search.cpp
    )
    ament_target_dependencies(astar_test finenav_core finenav_common finenav_interfaces rclcpp)
    target_include_directories(astar_test PRIVATE test)
    target_link_libraries(astar_test astar_params)

    ament_add_gtest(bfs_test
        test/bfs_test.cpp
        src/astar/astar_algorithm.cpp      # BFS reuses AStarAlgorithm
        src/bfs_path_search.cpp
    )
    ament_target_dependencies(bfs_test finenav_core finenav_common finenav_interfaces)
    target_include_directories(bfs_test PRIVATE test)
    target_link_libraries(bfs_test bfs_params)

    ament_add_gtest(dstarlite_test
        test/dstarlite_test.cpp
        src/dstarlite/dstarlite_algorithm.cpp
        src/dstarlite_path_search.cpp
    )
    ament_target_dependencies(dstarlite_test finenav_core finenav_common finenav_interfaces rclcpp geometry_msgs)
    target_include_directories(dstarlite_test PRIVATE test)
    target_link_libraries(dstarlite_test dstarlite_params)

    ament_add_gtest(detour_test
        test/detour_test.cpp
        src/astar/astar_algorithm.cpp      # obstacle-aware snap search (findPathToAny)
        src/detour_planner.cpp
    )
    ament_target_dependencies(detour_test finenav_core finenav_common finenav_interfaces rclcpp geometry_msgs)
    target_include_directories(detour_test PRIVATE test)
    target_link_libraries(detour_test detour_params)

    # performance benchmark (google benchmark, no ROS runtime deps)
    add_subdirectory(benchmark)
endif ()

# ── standalone eval (always built, no ROS deps) ─────────────────────────────
add_subdirectory(eval)

install(TARGETS astar_params bfs_params dstarlite_params detour_params
        EXPORT export_${PROJECT_NAME}_params
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
ament_export_targets(export_${PROJECT_NAME}_params HAS_LIBRARY_TARGET)
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})

finenav_register_algo_plugin(finenav_route_planner_lib)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
