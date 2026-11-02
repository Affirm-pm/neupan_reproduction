cmake_minimum_required(VERSION 3.22)
project(finenav_common)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(finenav_util REQUIRED)
find_package(fmt REQUIRED)

# A shared library, not a header-only interface, and deliberately so: this is
# where the ONE definition of frames() lives.  A header-only frames() would be
# instantiated once per DSO and each plugin would read its own copy — the exact
# failure the cross-DSO singleton discipline exists to prevent.  See
# include/finenav_common/frames.hpp.
add_library(${PROJECT_NAME} SHARED
        src/frame_names.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
ament_target_dependencies(${PROJECT_NAME} SYSTEM
        rclcpp
        finenav_util
        fmt
)

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
)
install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp finenav_util fmt)

ament_package()
