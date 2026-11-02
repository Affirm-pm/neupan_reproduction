// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <fmt/format.h>
#include <rclcpp/logging.hpp>


// ── Macros for logging ────────────────────────────────────────────────

#define FINENAV_LOG_DEBUG(logger, ...) \
    RCLCPP_DEBUG(logger, "%s", ::fmt::format(__VA_ARGS__).c_str())

#define FINENAV_LOG_INFO(logger, ...) \
    RCLCPP_INFO(logger, "%s", ::fmt::format(__VA_ARGS__).c_str())

#define FINENAV_LOG_WARN(logger, ...) \
    RCLCPP_WARN(logger, "%s", ::fmt::format(__VA_ARGS__).c_str())

#define FINENAV_LOG_ERROR(logger, ...) \
    RCLCPP_ERROR(logger, "%s", ::fmt::format(__VA_ARGS__).c_str())

#define FINENAV_LOG_DEBUG_THROTTLE(logger, duration_ms, ...) \
    do { \
        static auto _fn_throttle_ = rclcpp::Clock{RCL_STEADY_TIME}; \
        RCLCPP_DEBUG_THROTTLE(logger, _fn_throttle_, duration_ms, \
            "%s", ::fmt::format(__VA_ARGS__).c_str()); \
    } while (0)

#define FINENAV_LOG_INFO_THROTTLE(logger, duration_ms, ...) \
    do { \
        static auto _fn_throttle_ = rclcpp::Clock{RCL_STEADY_TIME}; \
        RCLCPP_INFO_THROTTLE(logger, _fn_throttle_, duration_ms, \
            "%s", ::fmt::format(__VA_ARGS__).c_str()); \
    } while (0)

#define FINENAV_LOG_WARN_THROTTLE(logger, duration_ms, ...) \
    do { \
        static auto _fn_throttle_ = rclcpp::Clock{RCL_STEADY_TIME}; \
        RCLCPP_WARN_THROTTLE(logger, _fn_throttle_, duration_ms, \
            "%s", ::fmt::format(__VA_ARGS__).c_str()); \
    } while (0)

#define FINENAV_LOG_ERROR_THROTTLE(logger, duration_ms, ...) \
    do { \
        static auto _fn_throttle_ = rclcpp::Clock{RCL_STEADY_TIME}; \
        RCLCPP_ERROR_THROTTLE(logger, _fn_throttle_, duration_ms, \
            "%s", ::fmt::format(__VA_ARGS__).c_str()); \
    } while (0)

#define FINENAV_LOG_DEBUG_ONCE(logger, ...) \
    RCLCPP_DEBUG_ONCE(logger, "%s", ::fmt::format(__VA_ARGS__).c_str())

#define FINENAV_LOG_INFO_ONCE(logger, ...) \
    RCLCPP_INFO_ONCE(logger, "%s", ::fmt::format(__VA_ARGS__).c_str())

#define FINENAV_LOG_WARN_ONCE(logger, ...) \
    RCLCPP_WARN_ONCE(logger, "%s", ::fmt::format(__VA_ARGS__).c_str())

#define FINENAV_LOG_ERROR_ONCE(logger, ...) \
    RCLCPP_ERROR_ONCE(logger, "%s", ::fmt::format(__VA_ARGS__).c_str())

