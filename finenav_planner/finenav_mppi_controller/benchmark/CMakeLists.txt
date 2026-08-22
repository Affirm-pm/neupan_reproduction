find_package(benchmark REQUIRED)

add_executable(eigen_mppi_benchmark eigen_mppi_benchmark.cpp)
ament_target_dependencies(eigen_mppi_benchmark ${dependencies_pkgs})
target_include_directories(eigen_mppi_benchmark PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(eigen_mppi_benchmark
  mppi_controller
  mppi_critics
  benchmark::benchmark
  Eigen3::Eigen
)

add_executable(mppi_practical_comparison practical_comparison_benchmark.cpp)
ament_target_dependencies(mppi_practical_comparison ${dependencies_pkgs})
target_include_directories(mppi_practical_comparison PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(mppi_practical_comparison
  mppi_controller
  mppi_critics
  benchmark::benchmark
  Eigen3::Eigen
)

add_executable(cost_critic_benchmark cost_critic_benchmark.cpp)
ament_target_dependencies(cost_critic_benchmark ${dependencies_pkgs})
target_include_directories(cost_critic_benchmark PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(cost_critic_benchmark
  mppi_critics
  benchmark::benchmark
  Eigen3::Eigen
)
