// Copyright (c) 2026. IWIN-FINS Lab, Shanghai Jiao Tong University.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace finenav::example {

// Keep the best distance across local recoveries. Travelling in a circle, or
// returning after a detour, must not refill the recovery budget repeatedly.
struct GoalProgress {
    double checkpoint{std::numeric_limits<double>::infinity()};

    void reset() { checkpoint = std::numeric_limits<double>::infinity(); }

    bool update(double distance) {
        if (!std::isfinite(distance)) return false;
        if (!std::isfinite(checkpoint) || checkpoint - distance >= 0.10) {
            checkpoint = distance;
            return true;
        }
        return false;
    }
};

// Slow the reference itself, so the optimizer can turn toward a nearby goal
// before passing it. Clamping only its output would invalidate its prediction.
inline double approachReferenceSpeed(double speed, double distance,
                                     double gain, double min_speed) {
    return std::min(speed, std::max(min_speed, gain * distance));
}

}  // namespace finenav::example
