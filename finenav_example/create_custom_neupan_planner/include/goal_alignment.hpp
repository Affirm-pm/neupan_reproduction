// Copyright (c) 2026. IWIN-FINS Lab, Shanghai Jiao Tong University.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace finenav::example {

// Terminal rotation is separate from NeuPAN's position-only path arrival.
// It uses the original goal quaternion, not the path's final tangent.
struct GoalAlignment {
    struct Options {
        double xy_tolerance{0.15};
        double yaw_tolerance{0.05};
        double gain{1.5};
        double max_speed{0.6};
        double min_speed{0.12};
    } options;
    bool active{false};

    void reset() { active = false; }

    bool update(double distance) {
        // Hysteresis prevents position noise from alternating tracking/rotation.
        active = distance < options.xy_tolerance * (active ? 1.5 : 1.0);
        return active;
    }

    double angularVelocity(double yaw_error, double measured_w,
                           double robot_max_speed, double robot_max_accel,
                           double step_time) const {
        yaw_error = std::atan2(std::sin(yaw_error), std::cos(yaw_error));
        if (!std::isfinite(yaw_error) || !std::isfinite(measured_w) ||
            std::abs(yaw_error) < options.yaw_tolerance) return 0.0;
        const double limit = std::min(options.max_speed, robot_max_speed);
        const double speed = std::clamp(options.gain * std::abs(yaw_error),
                                       std::min(options.min_speed, limit), limit);
        const double desired = std::copysign(speed, yaw_error);
        const double delta = robot_max_accel * step_time;
        return std::clamp(std::clamp(desired, measured_w - delta, measured_w + delta),
                          -limit, limit);
    }

    // Check the rectangle throughout the next control interval and braking arc.
    // Points are in the same world frame as the robot. Account for the gaps
    // between sampled angles using the footprint's circumradius (a distance
    // lower bound), so a corner cannot slip through between the samples.
    template<class Points>
    static double rotationClearance(const Points& points, double px, double py,
                                    double yaw, double w, double length,
                                    double width, double wheelbase,
                                    double accel, double step_time) {
        const double sweep = std::copysign(
            std::abs(w) * step_time + w * w / (2.0 * accel), w);
        const int steps = std::max(1, static_cast<int>(std::ceil(std::abs(sweep) / 0.02)));
        const double increment = sweep / steps;
        const double radius = std::hypot((length + std::abs(wheelbase)) / 2.0,
                                         width / 2.0);
        const double sample_margin = radius * std::abs(increment) / 2.0;
        double clearance = std::numeric_limits<double>::infinity();
        for (int i = 0; i <= steps; ++i) {
            const double angle = yaw + i * increment;
            const double c = std::cos(angle), s = std::sin(angle);
            for (const auto& p : points) {
                const double dx = p.x - px, dy = p.y - py;
                const double x = c * dx + s * dy - wheelbase / 2.0;
                const double y = -s * dx + c * dy;
                const double ex = std::max(std::abs(x) - length / 2.0, 0.0);
                const double ey = std::max(std::abs(y) - width / 2.0, 0.0);
                clearance = std::min(clearance, std::hypot(ex, ey) - sample_margin);
            }
        }
        return clearance;
    }
};

}  // namespace finenav::example
