add_executable(hello_finenav src/hello_finenav.cpp)
ament_target_dependencies(hello_finenav ${dependencies})

# terrain_mapview.hpp uses OpenCV and PCL
target_link_libraries(hello_finenav
        ${OpenCV_LIBRARIES}
        ${PCL_LIBRARIES}
)

target_include_directories(hello_finenav PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

install(TARGETS hello_finenav
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
        DESTINATION include
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)