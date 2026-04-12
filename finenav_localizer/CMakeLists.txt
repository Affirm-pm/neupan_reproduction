cmake_minimum_required(VERSION 3.22)
project(finenav_localizer)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)

# Sophus
include(FetchContent)
FetchContent_Declare(
        Sophus
        GIT_REPOSITORY https://github.com/strasdat/Sophus.git
        GIT_TAG 1.22.10
        GIT_SHALLOW TRUE
)
set(BUILD_SOPHUS_TESTS OFF CACHE BOOL "Disable Sophus tests" FORCE) # skipping tests to speed up the build process
message(STATUS "Fetching Sophus...")
FetchContent_MakeAvailable(Sophus)
message(STATUS "Sophus (1.22.10) integrated successfully.")

include_directories(include)

ament_auto_find_build_dependencies()


# FineNav Localizer (ESKF-based localization module)
ament_auto_add_library(${PROJECT_NAME}  SHARED
        src/finenav_localizer.cpp
)
target_link_libraries(${PROJECT_NAME}
        Sophus::Sophus
        Eigen3::Eigen
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
    add_subdirectory(eval)
endif ()

ament_auto_package() # TODO: 之后讨论下是否添加 USE_SCOPED_HEADER_INSTALL_DIR 的问题