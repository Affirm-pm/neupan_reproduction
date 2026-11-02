// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <chrono>
#include <string>

#include <rclcpp/logging.hpp>

#include "finenav_util/common/log_macros.hpp"

namespace finenav::util {

/**
 * @brief RAII scope timer. Construction records the start time; destruction logs the elapsed duration at DEBUG level.
 * Usage:
 * @code
 *  void foo() {
 *      Stopwatch sw(logger, "foo");
 *      // ... work ...
 *  }  // auto-logs:  "foo: 12.34 ms"
 * @endcode
 */

class Stopwatch {
public:
    Stopwatch(rclcpp::Logger logger, std::string name)
        : logger_(std::move(logger)), name_(std::move(name)),
          start_(std::chrono::steady_clock::now()) {}

    ~Stopwatch() {
        FINENAV_LOG_DEBUG(logger_, "{}: {:.2f} ms", name_, elapsed_ms());
    }

    /**
     * @brief Elapsed wall-clock time in milliseconds since construction.
     * @return Elapsed time in milliseconds.
     */
    double elapsed_ms() const {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::milli>(now - start_).count();
    }

    /**
     * @brief WARN_THROTTLE if elapsed exceeds the given threshold (in ms).
     * @return true if the threshold was exceeded.
     */
    bool warn_if_over(double threshold_ms) {
        double elapsed = elapsed_ms();
        if (elapsed > threshold_ms) {
            FINENAV_LOG_WARN_THROTTLE(logger_, 1000,
                "{}: {:.2f} ms > {:.1f} ms threshold — timer overrun",
                name_, elapsed, threshold_ms);
            return true;
        }
        return false;
    }

    Stopwatch(const Stopwatch&) = delete;
    Stopwatch& operator=(const Stopwatch&) = delete;
    Stopwatch(Stopwatch&&) = delete;
    Stopwatch& operator=(Stopwatch&&) = delete;

private:
    rclcpp::Logger logger_;
    std::string name_;
    std::chrono::steady_clock::time_point start_;
};

}  // namespace finenav::util
