# ── filter_model (pure math, no ROS) ──────────────────────────────────────────
ament_add_gtest(test_filter_model
        test_filter_model.cpp
)
target_include_directories(test_filter_model PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(test_filter_model
        finenav_localizer
)

# ── kf_math free functions ─────────────────────────────────────────────────────
ament_add_gtest(test_kf_math
        test_kf_math.cpp
)
target_include_directories(test_kf_math PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(test_kf_math
        finenav_localizer
)

# ── TimeDelayKalmanFilter standalone ────────────────────────────────────────────
ament_add_gtest(test_time_delay_kf
        test_time_delay_kf.cpp
)
target_include_directories(test_time_delay_kf PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(test_time_delay_kf
        finenav_localizer
)

# ── ESKF standalone ────────────────────────────────────────────────────────────
ament_add_gtest(test_eskf
        test_eskf.cpp
)
target_include_directories(test_eskf PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(test_eskf
        finenav_localizer
)

# ── AgedObservationQueue ────────────────────────────────────────────────────────
ament_add_gtest(test_observation_queue
        test_observation_queue.cpp
)
target_include_directories(test_observation_queue PRIVATE
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>
)
target_link_libraries(test_observation_queue
        finenav_localizer
)
