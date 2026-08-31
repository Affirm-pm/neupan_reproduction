cmake_minimum_required(VERSION 3.5)
project(finenav_mppi_controller)

find_package(ament_cmake REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenMP REQUIRED)
include_directories(
  include
)


set(dependencies_pkgs
  rclcpp
  builtin_interfaces
  tf2
  geometry_msgs
  visualization_msgs
  nav_msgs
  std_msgs
  tf2_geometry_msgs
  tf2_eigen
  tf2_ros
  angles
  OpenCV
        finenav_collision_model
        finenav_util
        finenav_core
        finenav_visualizer
        generate_parameter_library
        finenav_interfaces
)

foreach(pkg IN LISTS dependencies_pkgs)
  find_package(${pkg} REQUIRED)
endforeach()


# ↓ 覆盖 nav2_package() 设置
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-error=deprecated-declarations)
endif()
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# ↑ 覆盖 nav2_package() 设置

# ↓ 生成参数
generate_parameter_library(
        finenav_mppi_controller_params
        src/finenav_mppi_controller.yaml
)
# ↑ 生成参数

include(CheckCXXCompilerFlag)

check_cxx_compiler_flag("-mno-avx512f" COMPILER_SUPPORTS_AVX512)
check_cxx_compiler_flag("-msse4.2" COMPILER_SUPPORTS_SSE4)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
check_cxx_compiler_flag("-mfma" COMPILER_SUPPORTS_FMA)

if(COMPILER_SUPPORTS_AVX512)
  add_compile_options(-mno-avx512f)
endif()

if(COMPILER_SUPPORTS_SSE4)
  add_compile_options(-msse4.2)
endif()

if(COMPILER_SUPPORTS_AVX2)
  add_compile_options(-mavx2)
endif()

if(COMPILER_SUPPORTS_FMA)
  add_compile_options(-mfma)
endif()

# If building one the same hardware to be deployed on, try `-march=native`!
add_compile_options(-O3 -finline-limit=10000000 -ffp-contract=fast -ffast-math -mtune=generic)

option(ENABLE_COVERAGE "Enable GCC/Clang coverage instrumentation" OFF)
if(ENABLE_COVERAGE AND CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(--coverage -O0 -g)
  add_link_options(--coverage)
endif()

add_library(mppi_controller SHARED
  src/controller.cpp
  src/critic_manager.cpp
  src/noise_generator.cpp
  src/trajectory_visualizer.cpp
)

add_library(mppi_critics SHARED
  src/critics/cost_critic.cpp
  src/critics/goal_critic.cpp
  src/critics/goal_angle_critic.cpp
  src/critics/path_align_critic.cpp
  src/critics/path_follow_critic.cpp
  src/critics/path_angle_critic.cpp
  src/critics/prefer_forward_critic.cpp
  src/critics/twirling_critic.cpp
  src/critics/constraint_critic.cpp
  src/critics/velocity_deadband_critic.cpp
)

set(libraries mppi_controller mppi_critics)

foreach(lib IN LISTS libraries)
  target_compile_options(${lib} PUBLIC -fconcepts)
  if(lib STREQUAL "mppi_critics")
    target_link_libraries(${lib} Eigen3::Eigen finenav_mppi_controller_params OpenMP::OpenMP_CXX)
  else()
    target_link_libraries(${lib} Eigen3::Eigen finenav_mppi_controller_params)
  endif()
  target_include_directories(${lib} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )
  ament_target_dependencies(${lib} ${dependencies_pkgs})
endforeach()

install(
  TARGETS ${libraries} finenav_mppi_controller_params
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)




find_package(pcl_conversions REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common filters)


set(test_dependencies
        ${dependencies_pkgs}
        pcl_conversions
        pcl_msgs
)






install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(noise_generator_test
    test/noise_generator_test.cpp
  )
  ament_target_dependencies(noise_generator_test ${dependencies_pkgs})
  target_include_directories(noise_generator_test PRIVATE include)
  target_link_libraries(
    noise_generator_test
    mppi_controller
    mppi_critics
    finenav_mppi_controller_params
  )
  ament_add_gtest(critic_function_test
    test/critic_function_test.cpp
  )
  ament_target_dependencies(critic_function_test ${dependencies_pkgs})
  target_include_directories(critic_function_test PRIVATE include)
  target_link_libraries(
    critic_function_test
    mppi_critics
    finenav_mppi_controller_params
  )
  ament_add_gtest(optimizer_result_test
    test/optimizer_result_test.cpp
  )
  ament_target_dependencies(optimizer_result_test ${dependencies_pkgs})
  target_include_directories(optimizer_result_test PRIVATE include)
  target_link_libraries(
    optimizer_result_test
    mppi_controller
    mppi_critics
    finenav_mppi_controller_params
  )
  ament_add_gtest(controller_plan_test
    test/controller_plan_test.cpp
  )
  ament_target_dependencies(controller_plan_test ${dependencies_pkgs})
  target_include_directories(controller_plan_test PRIVATE include)
  target_link_libraries(
    controller_plan_test
    mppi_controller
    mppi_critics
    finenav_mppi_controller_params
  )
  ament_add_gtest(critic_score_test
    test/critic_score_test.cpp
  )
  ament_target_dependencies(critic_score_test ${dependencies_pkgs})
  target_include_directories(critic_score_test PRIVATE include)
  target_link_libraries(
    critic_score_test
    mppi_critics
    finenav_mppi_controller_params
  )
  ament_add_gtest(models_eigen_test
    test/models_eigen_test.cpp
  )
  ament_target_dependencies(models_eigen_test ${dependencies_pkgs})
  target_include_directories(models_eigen_test PRIVATE include)
  ament_add_gtest(utils_test
    test/utils_test.cpp
  )
  ament_target_dependencies(utils_test ${dependencies_pkgs})
  target_include_directories(utils_test PRIVATE include)

  ament_add_gtest(trajectory_visualizer_test
    test/trajectory_visualizer_test.cpp
  )
  ament_target_dependencies(trajectory_visualizer_test ${dependencies_pkgs})
  target_include_directories(trajectory_visualizer_test PRIVATE include)
  target_link_libraries(
    trajectory_visualizer_test
    mppi_controller
    mppi_critics
    finenav_mppi_controller_params
  )
  ament_add_gtest(trajectory_visualizer_channel_test
    test/trajectory_visualizer_channel_test.cpp
  )
  ament_target_dependencies(trajectory_visualizer_channel_test ${dependencies_pkgs})
  target_include_directories(trajectory_visualizer_channel_test PRIVATE include)
  target_link_libraries(
    trajectory_visualizer_channel_test
    mppi_controller
    mppi_critics
    finenav_mppi_controller_params
  )
  ament_add_gtest(critic_breakdown_test
    test/critic_breakdown_test.cpp
  )
  ament_target_dependencies(critic_breakdown_test ${dependencies_pkgs})
  target_include_directories(critic_breakdown_test PRIVATE include)
  target_link_libraries(
    critic_breakdown_test
    mppi_controller
    mppi_critics
    finenav_mppi_controller_params
  )
  ament_add_gtest(optimizer_visualized_reset_test
    test/optimizer_visualized_reset_test.cpp
  )
  ament_target_dependencies(optimizer_visualized_reset_test ${dependencies_pkgs})
  target_include_directories(optimizer_visualized_reset_test PRIVATE include benchmark)
  target_link_libraries(
    optimizer_visualized_reset_test
    mppi_controller
    mppi_critics
    finenav_mppi_controller_params
  )
endif()

option(BUILD_BENCHMARKS "Build Eigen MPPI benchmarks" OFF)
if(BUILD_BENCHMARKS)
  add_subdirectory(benchmark)
endif()

ament_export_libraries(${libraries})
ament_export_dependencies(${dependencies_pkgs} eigen3_cmake_module Eigen3)
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${dependencies_pkgs})
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
