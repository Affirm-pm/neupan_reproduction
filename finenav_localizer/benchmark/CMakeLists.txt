find_package(benchmark)

add_executable(${PROJECT_NAME}_bench
        bench_eskf.cpp
)
target_include_directories(${PROJECT_NAME}_bench PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(${PROJECT_NAME}_bench
        ${PROJECT_NAME}
        benchmark::benchmark
)
