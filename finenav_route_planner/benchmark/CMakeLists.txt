find_package(benchmark REQUIRED)

set(BENCHMARK_NAMES
  route_planner_benchmark
)

foreach(name IN LISTS BENCHMARK_NAMES)
  add_executable(${name} ${name}.cpp)

  target_link_libraries(${name}
    benchmark::benchmark
    finenav_route_planner_lib
  )

  target_include_directories(${name} PRIVATE
    ${PROJECT_SOURCE_DIR}/test      # for grid_map.hpp
  )
endforeach()
