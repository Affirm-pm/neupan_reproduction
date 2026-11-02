ament_add_gtest(test_integrator test_integrator.cpp)
target_include_directories(test_integrator PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)
target_link_libraries(test_integrator finenav_localizer)

ament_add_gtest(test_velocity_provider test_velocity_provider.cpp)
target_include_directories(test_velocity_provider PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>)
target_link_libraries(test_velocity_provider finenav_localizer)
