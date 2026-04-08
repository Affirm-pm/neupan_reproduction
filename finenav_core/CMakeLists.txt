cmake_minimum_required(VERSION 3.22)
project(finenav_core)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED) # TODO: Eigen 应该放在 package.xml 中，方便 rosdep 统一管理
find_package(eigen3_cmake_module REQUIRED) # TODO: 能不能删
find_package(fmt REQUIRED)

# Sophus
include(FetchContent)
FetchContent_Declare(
        Sophus
        GIT_REPOSITORY https://github.com/strasdat/Sophus.git
        GIT_TAG 1.24.6
        GIT_SHALLOW TRUE
)
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Disable Sophus tests" FORCE) # skipping tests to speed up the build process
message(STATUS "Fetching Sophus...")
FetchContent_MakeAvailable(Sophus)
message(STATUS "Sophus (1.24.6) integrated successfully.")

include_directories(include)

ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

ament_auto_add_library(test_localizer src/localizer.cpp)
target_link_libraries(test_localizer SHARED
        Sophus::Sophus
        fmt::fmt
)

#if (BUILD_TESTING)
#    find_package(ament_cmake_gtest REQUIRED)
#    ament_find_gtest()
#    add_subdirectory(test)
#endif ()

ament_export_dependencies( # TODO: 以后查一下编译的问题，是否提供了过多或过少的 denpendency
        eigen3_cmake_module
        Eigen3
)

ament_auto_package() # TODO: 之后讨论下是否添加 USE_SCOPED_HEADER_INSTALL_DIR 的问题