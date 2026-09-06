cmake_minimum_required(VERSION 3.22)
project(finenav_rotate_shim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)

find_package(xtensor REQUIRED)
find_package(xsimd REQUIRED)

set(dependencies
  rclcpp
  geometry_msgs
  finenav_core
  generate_parameter_library
)

foreach(pkg IN LISTS dependencies)
  find_package(${pkg} REQUIRED)
endforeach()

# Generate parameter library
generate_parameter_library(
  finenav_rotate_shim_params
  rotate_shim.yaml
)

# Header-only library (mostly inline) + small source
add_library(finenav_rotate_shim SHARED
  src/rotate_shim.cpp
)
target_include_directories(finenav_rotate_shim PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(finenav_rotate_shim ${dependencies})
target_link_libraries(finenav_rotate_shim finenav_rotate_shim_params)

install(
  TARGETS finenav_rotate_shim finenav_rotate_shim_params
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_libraries(finenav_rotate_shim)
ament_export_dependencies(${dependencies})
ament_package()
