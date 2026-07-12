find_package(benchmark REQUIRED)

# ── Core ESKF math benchmark (header-only estimator, no ROS runtime) ──────────
add_executable(${PROJECT_NAME}_bench_eskf
        bench_eskf.cpp
)
target_include_directories(${PROJECT_NAME}_bench_eskf PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(${PROJECT_NAME}_bench_eskf
        ${PROJECT_NAME}
        benchmark::benchmark
)

# ── Full-component benchmark (EskfLocalizer engine, driven synchronously) ─────
add_executable(${PROJECT_NAME}_bench_localizer
        bench_localizer.cpp
)
target_include_directories(${PROJECT_NAME}_bench_localizer PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(${PROJECT_NAME}_bench_localizer
        ${PROJECT_NAME}
        benchmark::benchmark
)
