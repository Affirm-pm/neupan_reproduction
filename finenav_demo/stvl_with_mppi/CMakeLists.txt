add_executable(hello_finenav src/hello_finenav.cpp)
ament_target_dependencies(hello_finenav ${dependencies})

install(TARGETS hello_finenav
        DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)