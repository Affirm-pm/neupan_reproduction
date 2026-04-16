add_executable(stvl_with_mppi_node src/hello_finenav.cpp)
ament_target_dependencies(stvl_with_mppi_node ${dependencies})

install(DIRECTORY launch
        DESTINATION share/${PROJECT_NAME}
)