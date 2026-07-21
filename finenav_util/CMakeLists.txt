cmake_minimum_required(VERSION 3.22)
project(finenav_util)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
        src/node_thread.cpp
        src/cloud_publish_helper.cpp
        src/task_channel.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
