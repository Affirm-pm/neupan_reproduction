cmake_minimum_required(VERSION 3.22)
project(finenav_route_planner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(include ${OpenCV_INCLUDE_DIRS})
ament_auto_find_build_dependencies()

# ── parameter libraries (generated from YAML) ──────────────────────────────
generate_parameter_library(astar_params  astar.yaml)
generate_parameter_library(bfs_params    bfs.yaml)
generate_parameter_library(route_params  route.yaml)

# ── planner library ─────────────────────────────────────────────────────────
ament_auto_add_library(finenav_route_planner_lib SHARED
    # Grid search engine (A* / Dijkstra / BFS all handled by AStarAlgorithm)
    src/astar_algorithm.cpp
    # Grid planner wrappers (FineNav PlannerConcept)
    src/astar_path_search.cpp
    src/bfs_path_search.cpp          # BFS = AStarAlgorithm(Heuristic::Zero)
    # Topological route planner (FineNav PlannerConcept)
    src/route_path_search.cpp
    # Route internals — graph loading (GeoJSON) + Dijkstra search
    src/route/route_graph.cpp
    src/route/route_search.cpp
)
target_link_libraries(finenav_route_planner_lib
    astar_params
    bfs_params
    route_params
    ${OpenCV_LIBS}
    Eigen3::Eigen
)

# ── unit tests ──────────────────────────────────────────────────────────────
if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)

    ament_add_gtest(astar_test
        test/astar_test.cpp
        src/astar_algorithm.cpp
        src/astar_path_search.cpp
    )
    ament_target_dependencies(astar_test finenav_core finenav_util finenav_interfaces rclcpp)
    target_include_directories(astar_test PRIVATE test)
    target_link_libraries(astar_test astar_params)

    ament_add_gtest(bfs_test
        test/bfs_test.cpp
        src/astar_algorithm.cpp      # BFS reuses AStarAlgorithm
        src/bfs_path_search.cpp
    )
    ament_target_dependencies(bfs_test finenav_core finenav_util finenav_interfaces)
    target_include_directories(bfs_test PRIVATE test)
    target_link_libraries(bfs_test bfs_params)

    ament_add_gtest(route_test
        test/route_test.cpp
        src/astar_algorithm.cpp        # used internally for obstacle-aware node snapping
        src/route_path_search.cpp
        src/route/route_graph.cpp
        src/route/route_search.cpp
    )
    ament_target_dependencies(route_test finenav_core finenav_util finenav_interfaces geometry_msgs std_msgs)
    target_include_directories(route_test PRIVATE test)
    target_link_libraries(route_test route_params)
endif ()

ament_auto_package()

