cmake_minimum_required(VERSION 3.22)
project(finenav_collision_model)

set(CMAKE_CXX_STANDARD 20)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(ament_cmake_auto REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

ament_auto_find_build_dependencies()

add_library(${PROJECT_NAME} SHARED
    src/obb_collision_model.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
    Eigen3::Eigen
)

# OBBCollisionModel exposes Eigen fixed-size types in its public ABI. Keep the
# ABI compatible with AVX-enabled consumers without requiring AVX everywhere.
target_compile_definitions(${PROJECT_NAME} PUBLIC EIGEN_MAX_ALIGN_BYTES=16)

if (BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_find_gtest()
    ament_add_gtest(${PROJECT_NAME}_test
        test/obb_collision_model_test.cpp
    )
    target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME})
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64|amd64|i[3-6]86)$" AND
        NOT CMAKE_CROSSCOMPILING AND EXISTS "/proc/cpuinfo")
        file(READ "/proc/cpuinfo" FINENAV_CPUINFO)
        string(REGEX MATCH "flags[^\n]*avx2" FINENAV_CPU_SUPPORTS_AVX2 "${FINENAV_CPUINFO}")
        string(REGEX MATCH "flags[^\n]*fma" FINENAV_CPU_SUPPORTS_FMA "${FINENAV_CPUINFO}")
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-msse4.2" FINENAV_COMPILER_SUPPORTS_SSE42)
        check_cxx_compiler_flag("-mavx2" FINENAV_COMPILER_SUPPORTS_AVX2)
        check_cxx_compiler_flag("-mfma" FINENAV_COMPILER_SUPPORTS_FMA)
    endif ()

    if (FINENAV_CPU_SUPPORTS_AVX2 AND FINENAV_CPU_SUPPORTS_FMA AND
        FINENAV_COMPILER_SUPPORTS_SSE42 AND FINENAV_COMPILER_SUPPORTS_AVX2 AND
        FINENAV_COMPILER_SUPPORTS_FMA)
        ament_add_gtest(${PROJECT_NAME}_avx_consumer_test
            test/obb_collision_model_test.cpp
        )
        target_compile_options(${PROJECT_NAME}_avx_consumer_test PRIVATE
            -msse4.2
            -mavx2
            -mfma
        )
        target_link_libraries(${PROJECT_NAME}_avx_consumer_test ${PROJECT_NAME})
    else ()
        message(STATUS "Skipping AVX consumer test: AVX2/FMA host or compiler support unavailable")
    endif ()
endif ()

install(
    TARGETS ${PROJECT_NAME}
    EXPORT export_${PROJECT_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
    eigen3_cmake_module
    Eigen3
)

ament_auto_package(USE_SCOPED_HEADER_INSTALL_DIR)
