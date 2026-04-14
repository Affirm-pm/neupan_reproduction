cmake_minimum_required(VERSION 3.22)
project(finenav_localizer)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(ament_cmake_auto REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

include_directories(include)

ament_auto_find_build_dependencies()

# FineNav Localizer (ESKF-based localization module)
ament_auto_add_library(${PROJECT_NAME} SHARED
        src/finenav_localizer.cpp
)
target_link_libraries(${PROJECT_NAME}
        Eigen3::Eigen
        Sophus::Sophus
)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    add_subdirectory(test)
    add_subdirectory(eval)
endif ()

ament_export_dependencies(
        Eigen3
        Sophus
)
ament_auto_package() # TODO: 之后讨论下是否添加 USE_SCOPED_HEADER_INSTALL_DIR 的问题