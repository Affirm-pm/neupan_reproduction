ament_auto_add_executable(${PROJECT_NAME}_eval
        eval_scenario.cpp
)
target_link_libraries(${PROJECT_NAME}_eval
        finenav_localizer
)
