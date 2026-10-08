// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "neupan_planner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

// The libneupan core.  Its header is heavy (Eigen + OsqpEigen) and must stay
// out of neupan_planner.hpp, which is why the implementation is behind a pimpl.
#include "neupan/neupan_planner.hpp"

#include "finenav_core/algo_register_macros.hpp"

namespace finenav::example {

namespace {

/// The libneupan core, bound once under a name that cannot be confused with
/// finenav::example::NeuPanPlanner (this class).  Never spell the core's own
/// name again in this file.
using CorePlanner = ::neupan::NeuPANPlanner;

/// Snapshot of every parameter that forces a core-planner rebuild.
///
/// neupan::NeuPANPlanner is immutable once constructed: the horizon and the
/// robot geometry fix the OSQP decision-variable layout, the NRMP weights fix
/// the Hessian, and the footprint must match the weights inside the DUNE model.
/// So "did anything structural change?" is answered by comparing this snapshot
/// rather than by rebuilding on every `ros2 param set`.
struct StructuralKey {
    int    receding{10};
    double step_time{0.1};
    double collision_threshold{0.1};
    double robot_length{1.6};
    double robot_width{2.0};
    double robot_wheelbase{0.0};
    double max_speed_linear{8.0};
    double max_speed_angular{1.0};
    double max_acce_linear{8.0};
    double max_acce_angular{3.0};
    double arrive_threshold{0.1};
    double close_threshold{0.1};
    int    ind_range{10};
    int    arrive_index_threshold{1};
    double path_interval{-1.0};
    int    iter_num{2};
    double iter_threshold{0.1};
    int    dune_max_num{100};
    int    nrmp_max_num{10};
    double q_s{1.0};
    double p_u{1.0};
    double eta{15.0};
    double d_max{1.0};
    double d_min{0.1};
    double ro_obs{400.0};
    double bk{0.1};

    bool operator==(const StructuralKey& other) const = default;
};

/// Yaw from a quaternion, valid over the whole (-pi, pi] range (the R[2][0] /
/// R[2][2] form).  Identical to the upstream neupan_cpp_ros helper, so both
/// stacks read a pose the same way.
double yawFromQuat(const geometry_msgs::msg::Quaternion& q) {
    return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

/// FineNav poses -> libneupan path points (x, y, theta, gear).
///
/// theta is taken from the NEXT point's gradient, which is what the upstream
/// ROS node does when `include_initial_path_direction` is false.  The last
/// point keeps the previous heading; a single-segment path falls back to the
/// pose's own quaternion.  gear is always 1 (line paths only).
std::vector<::neupan::InitialPath::PathPoint> toPathPoints(
    const std::vector<geometry_msgs::msg::Pose>& poses)
{
    std::vector<::neupan::InitialPath::PathPoint> out;
    out.reserve(poses.size());

    double prev_theta = 0.0;
    for (std::size_t i = 0; i < poses.size(); ++i) {
        const auto& p = poses[i].position;
        double theta;
        if (i + 1 < poses.size()) {
            const auto& q = poses[i + 1].position;
            const double dx = q.x - p.x;
            const double dy = q.y - p.y;
            theta = (dx == 0.0 && dy == 0.0)
                        ? yawFromQuat(poses[i].orientation)
                        : std::atan2(dy, dx);
        } else {
            // Last point: nothing to differentiate against.
            theta = prev_theta;
        }
        out.emplace_back(p.x, p.y, theta, 1.0);
        prev_theta = theta;
    }

    // A one-point path never reaches the loop above with a usable heading, so
    // seed it from the pose.  (Callers only take this branch for >= 2 poses.)
    if (out.size() == 1) {
        out.front()(2) = yawFromQuat(poses.front().orientation);
    }
    return out;
}

/// Element-wise comparison with the same tolerance the upstream node uses when
/// deciding whether a republished reference is actually new.
bool samePath(const std::vector<::neupan::InitialPath::PathPoint>& a,
              const std::vector<::neupan::InitialPath::PathPoint>& b)
{
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if ((a[i] - b[i]).cwiseAbs().maxCoeff() > 1e-9) return false;
    }
    return true;
}

/// Where the DUNE weights live.
///
/// The ROS parameter always wins.  Only when it is empty do we fall back to the
/// path baked in at build time from libneupan's installed models directory
/// (${libneupan_MODELS_DIR}), so a default build runs without any configuration
/// but a machine that moved the models can point at them without rebuilding.
std::string resolveCheckpoint(const std::string& configured) {
    if (!configured.empty()) return configured;
#ifdef NEUPAN_DEFAULT_CHECKPOINT
    return NEUPAN_DEFAULT_CHECKPOINT;
#else
    return {};
#endif
}

}  // namespace

/// Everything that needs libneupan (and therefore Eigen) to be a complete type.
struct NeuPanPlanner::Impl {
    std::unique_ptr<CorePlanner> planner;
    StructuralKey                key;
    std::string                  resolved_checkpoint;
    bool                         ready{false};

    /// Obstacle points handed to the core, (2, N), world-fixed frame.
    /// Reused every cycle: plan() runs at the layer's frequency and must not
    /// allocate for the common case.
    ::neupan::Mat2X points_buf;

    /// Reference bookkeeping.  `last_path` is kept so a republished path can be
    /// detected without resetting the core's warm start.
    std::vector<::neupan::InitialPath::PathPoint> last_path;
    bool   have_path{false};
    bool   have_goal{false};
    double last_goal_x{0.0};
    double last_goal_y{0.0};
    double last_goal_yaw{0.0};

    int  consecutive_unsolved{0};
    bool progress_started{false};
    std::chrono::steady_clock::time_point last_progress{};

    /// Throttling for the recoverable-condition warnings.  System time, not
    /// ROS time: a warning must still be rate-limited before /clock exists.
    rclcpp::Clock clock{RCL_SYSTEM_TIME};
};

namespace {

/// The result every recoverable condition returns: one zero-velocity command.
///
/// Deliberately not std::nullopt.  ControlLayer's periodic body treats nullopt
/// as terminal — "Periodic: plan() faulted — aborting", publishFaulted(), the
/// worker thread returns — so returning it for a transient condition (model not
/// loaded yet, solver struggling, reference momentarily empty) would kill the
/// layer instead of merely pausing the robot.
NeuPanPlanner::Result zeroTrajectory() {
    using Trajectory = finenav::core::Trajectory<NeuPanPlanner::OutputProfile>;
    geometry_msgs::msg::Twist cmd;   // zero-initialised
    Trajectory result;
    result.twists.push_back(cmd);
    return result;
}

}  // namespace

NeuPanPlanner::NeuPanPlanner() : impl_(std::make_unique<Impl>()) {}
NeuPanPlanner::~NeuPanPlanner() = default;

// ==============================================================================
//  Parameter channel — called at startup and on every `ros2 param set`
// ==============================================================================
void NeuPanPlanner::configure(const ConfigType& cfg) {
    // ── Values that never require touching the core planner ────────────────
    ref_speed_         = cfg.ref_speed;
    solver_fail_grace_ = static_cast<int>(cfg.solver_fail_grace);
    stall_speed_       = cfg.stall_speed;
    stall_timeout_     = cfg.stall_timeout;

    // ── Structural snapshot ────────────────────────────────────────────────
    StructuralKey key;
    key.receding               = static_cast<int>(cfg.receding);
    key.step_time              = cfg.step_time;
    key.collision_threshold    = cfg.collision_threshold;
    key.robot_length           = cfg.robot_length;
    key.robot_width            = cfg.robot_width;
    key.robot_wheelbase        = cfg.robot_wheelbase;
    key.max_speed_linear       = cfg.max_speed_linear;
    key.max_speed_angular      = cfg.max_speed_angular;
    key.max_acce_linear        = cfg.max_acce_linear;
    key.max_acce_angular       = cfg.max_acce_angular;
    key.arrive_threshold       = cfg.arrive_threshold;
    key.close_threshold        = cfg.close_threshold;
    key.ind_range              = static_cast<int>(cfg.ind_range);
    key.arrive_index_threshold = static_cast<int>(cfg.arrive_index_threshold);
    key.path_interval          = cfg.path_interval;
    key.iter_num               = static_cast<int>(cfg.iter_num);
    key.iter_threshold         = cfg.iter_threshold;
    key.dune_max_num           = static_cast<int>(cfg.dune_max_num);
    key.nrmp_max_num           = static_cast<int>(cfg.nrmp_max_num);
    key.q_s                    = cfg.q_s;
    key.p_u                    = cfg.p_u;
    key.eta                    = cfg.eta;
    key.d_max                  = cfg.d_max;
    key.d_min                  = cfg.d_min;
    key.ro_obs                 = cfg.ro_obs;
    key.bk                     = cfg.bk;

    const std::string checkpoint = resolveCheckpoint(cfg.dune_checkpoint);

    if (impl_->ready && impl_->key == key &&
        impl_->resolved_checkpoint == checkpoint) {
        // Nothing structural moved — a `ros2 param set` on ref_speed (or on a
        // parameter this planner ignores) must not reload the DUNE model.
        if (impl_->planner) impl_->planner->setReferenceSpeed(ref_speed_);
        return;
    }

    // ── Rebuild ────────────────────────────────────────────────────────────
    impl_->planner.reset();
    impl_->ready = false;
    impl_->key = key;
    impl_->resolved_checkpoint = checkpoint;
    impl_->have_path = false;
    impl_->have_goal = false;
    impl_->last_path.clear();
    impl_->consecutive_unsolved = 0;

    if (checkpoint.empty()) {
        RCLCPP_ERROR(logger_,
            "neupan_planner.dune_checkpoint is empty and no build-time default was "
            "configured. Running in the safe state (zero velocity) until it is set: "
            "ros2 param set <node> neupan_planner.dune_checkpoint <path/to/model.bin>");
        return;
    }

    std::error_code ec;
    if (!std::filesystem::exists(checkpoint, ec) || ec) {
        RCLCPP_ERROR(logger_,
            "DUNE checkpoint '%s' does not exist. Running in the safe state (zero "
            "velocity) until a valid path is set.", checkpoint.c_str());
        return;
    }

    CorePlanner::Config core;
    core.receding            = key.receding;
    core.step_time           = key.step_time;
    core.ref_speed           = ref_speed_;
    core.collision_threshold = key.collision_threshold;
    core.max_speed           = ::neupan::Vec2(key.max_speed_linear, key.max_speed_angular);
    core.max_acce            = ::neupan::Vec2(key.max_acce_linear, key.max_acce_angular);
    core.length              = key.robot_length;
    core.width               = key.robot_width;
    core.wheelbase           = key.robot_wheelbase;

    // The initial path is owned by the reference trajectory, never by YAML: a
    // waypoint list baked in here would silently win over the navigation goal.
    core.ipath.waypoints              = {};
    core.ipath.loop                   = false;
    core.ipath.interval               = key.path_interval;
    core.ipath.arrive_threshold       = key.arrive_threshold;
    core.ipath.close_threshold        = key.close_threshold;
    core.ipath.ind_range              = key.ind_range;
    core.ipath.arrive_index_threshold = key.arrive_index_threshold;

    core.pan.iter_num       = key.iter_num;
    core.pan.dune_max_num   = key.dune_max_num;
    core.pan.iter_threshold = key.iter_threshold;

    core.adjust.max_num = key.nrmp_max_num;
    core.adjust.q_s     = ::neupan::Vec3::Constant(key.q_s);
    core.adjust.p_u     = key.p_u;
    core.adjust.eta     = key.eta;
    core.adjust.d_max   = key.d_max;
    core.adjust.d_min   = key.d_min;
    core.adjust.ro_obs  = key.ro_obs;
    core.adjust.bk      = key.bk;

    core.dune_checkpoint = checkpoint;

    try {
        impl_->planner = std::make_unique<CorePlanner>(core);
        impl_->ready = true;
    } catch (const std::exception& e) {
        // Two things throw here, both user errors that must NOT abort the layer:
        //   - validate(): self-contradictory parameters (d_max < d_min, eta > 0
        //     with d_max <= 0, collision_threshold <= 0, receding < 1, ...)
        //   - the DUNE loader: weights whose recorded length/width do not match
        //     robot_length/robot_width.
        // Log, stay in the safe state, and let the operator fix the parameter.
        RCLCPP_ERROR(logger_, "NeuPAN rejected the configuration: %s", e.what());
        impl_->planner.reset();
        impl_->ready = false;
        return;
    }

    RCLCPP_INFO(logger_,
        "NeuPAN ready: checkpoint '%s', horizon %d, dt %.3f s, footprint %.2f x %.2f m, "
        "ref_speed %.2f m/s.",
        checkpoint.c_str(), key.receding, key.step_time, key.robot_length,
        key.robot_width, ref_speed_);
}

// ==============================================================================
//  Episode boundary
// ==============================================================================
void NeuPanPlanner::reset() {
    // Episode state only.  The core planner object — and with it the loaded
    // DUNE model — is never torn down here: a new navigation goal must not cost
    // a model reload.
    if (impl_->planner) impl_->planner->reset();

    impl_->have_path = false;
    impl_->have_goal = false;
    impl_->last_path.clear();
    impl_->consecutive_unsolved = 0;
    impl_->progress_started = false;

    RCLCPP_DEBUG(logger_, "reset(): episode state cleared.");
}

// ==============================================================================
//  Core
// ==============================================================================
NeuPanPlanner::Result NeuPanPlanner::plan(const Context& ctx, const IWorldView& world) {
    // ── 0. Not ready ───────────────────────────────────────────────────────
    if (!impl_->ready || !impl_->planner) {
        RCLCPP_WARN_THROTTLE(logger_, impl_->clock, 2000,
            "NeuPAN is not ready (no usable DUNE checkpoint); commanding zero velocity.");
        return zeroTrajectory();
    }

    const auto& poses = ctx.ref_traj.poses;
    if (poses.empty()) {
        RCLCPP_WARN_THROTTLE(logger_, impl_->clock, 2000,
            "Empty reference trajectory; commanding zero velocity.");
        return zeroTrajectory();
    }

    // Both the state and the obstacle points are world-fixed, which is exactly
    // what the core's forward() expects — no frame conversion on this side.
    const auto& pose = ctx.robot_state.pose;
    const ::neupan::Vec3 state(pose.position.x, pose.position.y,
                               yawFromQuat(pose.orientation));

    // ── 1. Reference -> initial path ───────────────────────────────────────
    if (poses.size() >= 2) {
        auto path = toPathPoints(poses);
        if (!impl_->have_path) {
            impl_->planner->setInitialPath(path);
            impl_->planner->reset();
            impl_->last_path = std::move(path);
            impl_->have_path = true;
            impl_->have_goal = false;
            impl_->consecutive_unsolved = 0;
            RCLCPP_INFO(logger_, "NeuPAN initial path installed (%zu poses).", poses.size());
        } else if (!samePath(path, impl_->last_path)) {
            // replaceInitialPath re-projects the robot onto the new path, so a
            // replanned reference continues the episode instead of cold-starting
            // it.  setInitialPath + reset() would discard the warm start.
            impl_->planner->replaceInitialPath(path, state);
            impl_->last_path = std::move(path);
            impl_->have_goal = false;
            impl_->consecutive_unsolved = 0;
            RCLCPP_INFO(logger_, "NeuPAN initial path replaced (%zu poses).", poses.size());
        }
    } else {
        // A single pose is a GOAL, which is what the framework normally sends
        // (PipelineController::publishNavigationGoal).  Re-issuing it every
        // cycle would reset the core every cycle, so this is gated on the goal
        // actually having changed — the guard the upstream ROS node gets for
        // free from its message callback, which a periodic pull does not have.
        const double gx   = poses.back().position.x;
        const double gy   = poses.back().position.y;
        const double gyaw = yawFromQuat(poses.back().orientation);

        const bool goal_changed =
            !impl_->have_goal ||
            std::abs(gx - impl_->last_goal_x) > 1e-9 ||
            std::abs(gy - impl_->last_goal_y) > 1e-9 ||
            std::abs(gyaw - impl_->last_goal_yaw) > 1e-9;

        if (goal_changed) {
            impl_->planner->updateInitialPathFromGoal(state, ::neupan::Vec3(gx, gy, gyaw));
            impl_->planner->reset();
            impl_->have_goal = true;
            impl_->have_path = false;
            impl_->last_path.clear();
            impl_->consecutive_unsolved = 0;
            impl_->last_goal_x = gx;
            impl_->last_goal_y = gy;
            impl_->last_goal_yaw = gyaw;
            RCLCPP_INFO(logger_, "NeuPAN goal set to (%.2f, %.2f, yaw %.2f).", gx, gy, gyaw);
        }
    }

    // Without a path the core's checkArrive() would synthesise one from the
    // current state, which is never what we want on the first cycle after a
    // goal arrives but before the path exists.
    if (!impl_->planner->ipath().hasPath()) {
        RCLCPP_WARN_THROTTLE(logger_, impl_->clock, 2000,
            "No usable initial path yet; commanding zero velocity.");
        return zeroTrajectory();
    }

    // ── 2. Obstacle points, (2, N) world-fixed ─────────────────────────────
    const auto& obstacles = world.obstaclePoints();
    impl_->points_buf.resize(2, static_cast<Eigen::Index>(obstacles.size()));
    for (std::size_t i = 0; i < obstacles.size(); ++i) {
        const auto col = static_cast<Eigen::Index>(i);
        impl_->points_buf(0, col) = obstacles[i].x;
        impl_->points_buf(1, col) = obstacles[i].y;
    }

    // ── 3. Solve ───────────────────────────────────────────────────────────
    CorePlanner::Info info;
    const ::neupan::Vec2 action = impl_->planner->forward(state, impl_->points_buf, info);

    // ── 4. Safety gate (mirrors the upstream node's run()) ─────────────────
    if (info.solved) {
        impl_->consecutive_unsolved = 0;
    } else {
        ++impl_->consecutive_unsolved;
    }

    const double v = action(0);
    const double w = action(1);
    const bool finite = std::isfinite(v) && std::isfinite(w);
    const bool solver_gave_up =
        !info.solved && impl_->consecutive_unsolved > solver_fail_grace_;

    if (info.arrive) {
        // Arrival skips the solve, so test it before anything derived from the
        // solver's verdict.
        RCLCPP_INFO_THROTTLE(logger_, impl_->clock, 1000,
            "NeuPAN: arrived at the end of the path.");
    } else if (!finite) {
        RCLCPP_WARN_THROTTLE(logger_, impl_->clock, 500,
            "Non-finite command from NeuPAN (v=%f, w=%f); commanding zero velocity.", v, w);
    } else if (info.stop) {
        RCLCPP_WARN_THROTTLE(logger_, impl_->clock, 500,
            "NeuPAN stop: closest obstacle %.3f m is below collision_threshold %.3f m.",
            info.min_distance, impl_->planner->collisionThreshold());
    } else if (solver_gave_up) {
        RCLCPP_WARN_THROTTLE(logger_, impl_->clock, 500,
            "%d consecutive unsolved cycles (osqp status %d); commanding zero velocity. "
            "The nominal carries no obstacle avoidance, so it must not be followed.",
            impl_->consecutive_unsolved, info.solver_status);
    }

    const bool hold = info.arrive || info.stop || !finite || solver_gave_up;

    geometry_msgs::msg::Twist cmd;   // zero-initialised, so hold == all zeros
    if (!hold) {
        cmd.linear.x  = v;
        cmd.angular.z = w;
    }

    // ── 5. Stall diagnostics ───────────────────────────────────────────────
    // NeuPAN only avoids locally: with a single-pose reference it beelines for
    // the goal, so a concave obstacle can leave it idling at the clearance with
    // neither stop nor arrive set.  Reporting that is the whole point of this
    // counter — the remedy is a global replan, which this layer cannot do.
    const bool moving = std::abs(cmd.linear.x) > stall_speed_ ||
                        std::abs(cmd.angular.z) > stall_speed_;
    const auto now = std::chrono::steady_clock::now();
    if (moving || info.arrive || !impl_->progress_started) {
        impl_->last_progress = now;
        impl_->progress_started = true;
    } else {
        const double stalled_s =
            std::chrono::duration<double>(now - impl_->last_progress).count();
        if (stalled_s > stall_timeout_) {
            RCLCPP_WARN_THROTTLE(logger_, impl_->clock, 2000,
                "Stalled for %.1f s without arriving; the path is probably blocked and "
                "needs a global replan.", stalled_s);
        }
    }

    // ── 6. Package the result ──────────────────────────────────────────────
    using Trajectory = finenav::core::Trajectory<OutputProfile>;
    Trajectory result;
    result.twists.push_back(cmd);
    return result;
}

}  // namespace finenav::example

// One-line registration: parameter wiring + plugin self-registration.
// "neupan_planner" must match the top-level key in neupan_planner.yaml, and it
// is also the name the framework looks the layer up by.
FINENAV_REGISTER_PLANNER(finenav::example::NeuPanPlanner, neupan_planner)
