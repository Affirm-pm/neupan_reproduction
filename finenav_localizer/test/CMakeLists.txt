# ── FineNavLocalizer unit tests ─────────────────────────────────────────────────

ament_add_gtest(test_localizer
        test_localizer.cpp
)
target_include_directories(test_localizer PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(test_localizer
        finenav_localizer
)
