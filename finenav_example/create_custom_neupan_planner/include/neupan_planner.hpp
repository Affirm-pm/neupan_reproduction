// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

// The planner role's entry point — it re-exports the Planner concept, the
// geometry vocabulary (finenav::common::Position3D) and, via
// plan/planner_concept.hpp, Trajectory / IOProfile / InputTrajectoryT /
// OutputTrajectoryT.  Nothing from a finenav_core subdirectory is included
// here: only finenav_core/*.hpp is the plugin contract.
#include "finenav_core/plugin_planner.hpp"

#include "finenav_example/neupan_planner_params.hpp"

namespace finenav::example {

/**
 * @brief NeuPAN's view of the world: obstacle points in the world-fixed frame.
 *
 * This interface belongs to the PLANNER, not to the framework — a planner
 * declares how it wants to see the world, and a WorldView plugin implements
 * that declaration.  The implementation lives next to this file, in
 * neupan_world_view.hpp (class NeuPanWorldView, registered as
 * "neupan_world_view"); the separated create_custom_world_view example is the
 * same pattern with a trivial map behind it.
 *
 * Named ObstacleView rather than WorldView so that the planner-side interface
 * and the concrete bridge can be told apart by name: the interface is "the
 * obstacles this planner can see", the bridge is "the world as this robot sees
 * it".  The framework's naming (DummyWorldView in create_custom_planner) is not
 * followed here precisely because this plugin ships both halves in one folder.
 *
 * Deliberately one method.  NeuPAN consumes a point set and nothing else: it
 * has no walkability query, no inflation query and no cost query.  Declaring
 * methods the planner never calls would be the mistake DummyPlanner makes with
 * its pure-virtual `isWalkable`, so this interface states only what plan()
 * actually reads.
 *
 * The points are expected in the world-fixed frame
 * (finenav::common::frames().world_fixed), which is also the frame
 * ctx.robot_state.pose lives in — NeuPAN's forward() takes global-frame points
 * and a global-frame state, so no conversion is needed on this side.
 */
class NeuPanObstacleView {
public:
    virtual ~NeuPanObstacleView() = default;

    /// Obstacle points, world-fixed frame.  May be empty (NeuPAN then tracks
    /// the reference path with no avoidance).  Must be cheap to call: the
    /// heavy filtering belongs in the view's prepare().
    virtual const std::vector<finenav::common::Position3D>& obstaclePoints() const = 0;
};

/**
 * @brief NeuPanPlanner — FineNav-Engine Planner plugin wrapping libneupan.
 *
 * The algorithm is neupan_cpp/libneupan's neupan::NeuPANPlanner (DUNE + NRMP +
 * PAN).  This class is only the adapter: it turns FineNav's per-cycle
 * PlanningContext into the (state, points) pair the core's forward() expects,
 * applies the safety gate the upstream ROS node applies, and returns the
 * resulting velocity command as a one-element trajectory.
 *
 * === Naming warning ===
 * `finenav::example::NeuPanPlanner` (this class) and `neupan::NeuPANPlanner`
 * (the libneupan core) differ only in the capitalisation of two letters, and
 * both are in scope in the .cpp.  The .cpp therefore binds the core under the
 * alias `CorePlanner` and never spells its name again; keep it that way.
 *
 * === Parameter channel ===
 * Configured from neupan_planner.yaml under the ROS namespace `neupan_planner`
 * (see that file for why the namespace is not simply `neupan`).
 * configure() is called once at startup and again on every `ros2 param set`.
 * It is idempotent: a rebuild of the core planner happens only when a
 * *structural* parameter actually changed, so setting an unrelated parameter
 * does not reload the DUNE model.
 *
 * === Reference handling ===
 * FineNav hands the reference in as ctx.ref_traj every cycle.  Two shapes are
 * supported, distinguished by size:
 *   - 1 pose  -> treated as a GOAL: updateInitialPathFromGoal() is called once,
 *                and only when that goal pose actually changed.  The framework
 *                normally delivers a single goal pose (PipelineController::
 *                publishNavigationGoal), so this is the common case.
 *   - >=2 poses -> treated as a PATH: setInitialPath() on the first sight,
 *                replaceInitialPath() on every later change, which re-projects
 *                the robot onto the new path instead of cold-starting.
 *
 * === Failure policy ===
 * plan() returns a ZERO-VELOCITY trajectory, never std::nullopt, for every
 * recoverable condition (model not loaded, empty reference, core reported stop
 * or arrival, solver failing, non-finite output).  This is deliberate:
 * ControlLayer's periodic body treats a nullopt result as terminal — it
 * publishes FAULTED and the worker thread returns for good — so a transient
 * problem would kill the layer instead of just pausing the robot.
 *
 * === Not implemented ===
 * - No visualisation: setVisualizer()/viz_ are absent on purpose, so that
 *   finenav_visualizer.hpp (~3.3 s of compile time plus a PCL dependency)
 *   stays out of this plugin.  Add them only if you actually draw.
 * - The MPC horizon the core returns (Info::opt_s) is not republished; only
 *   the first control is.  OutputProfile therefore enables `vel` alone.
 */
class NeuPanPlanner {
public:
    // ── Compile-time contract (enforced by FINENAV_REGISTER_PLANNER) ────────
    using ConfigType = neupan_planner::Params;
    using IWorldView = NeuPanObstacleView;

    /// The reference is consumed as poses only; NeuPAN owns its own dynamics
    /// model, so reference velocities would be ignored rather than used.
    static constexpr finenav::core::IOProfile InputProfile {true,  false, false, false};
    /// One velocity command per cycle; see "Not implemented" above.
    static constexpr finenav::core::IOProfile OutputProfile{false, true,  false, false};

    NeuPanPlanner();
    ~NeuPanPlanner();

    NeuPanPlanner(const NeuPanPlanner&)            = delete;
    NeuPanPlanner& operator=(const NeuPanPlanner&) = delete;

    /**
     * @brief Apply parameter configuration.
     * @details Called by the framework at startup AND on every ros2 param
     *          update.  Hot parameters are stored in place; a change to any
     *          structural parameter rebuilds the wrapped core planner, and a
     *          failed rebuild leaves the planner in the "not ready" state
     *          (zero velocity) rather than aborting the layer.
     * @param cfg Full parameter struct generated from neupan_planner.yaml.
     */
    void configure(const ConfigType& cfg);

    // ── Framework injectors ────────────────────────────────────────────────
    // Detected by the framework via `requires { setLogger(declval<const
    // rclcpp::Logger&>()); }` — the name and the parameter type must match
    // exactly, otherwise the injection is silently skipped.
    void setLogger(const rclcpp::Logger& logger) { logger_ = logger; }

    /// Called by the framework when a new navigation episode starts.
    /// Clears episode state only — the core planner object (and the DUNE model
    /// inside it) is never unloaded here.
    void reset();

    using Context = finenav::core::PlanningContext<NeuPanPlanner>;
    using Result  = std::optional<finenav::core::OutputTrajectoryT<NeuPanPlanner>>;

    /// Plan one cycle.
    /// @param ctx   read-only views of the robot state and the reference
    ///              trajectory; neither may be retained past this call.
    /// @param world cheap obstacle-point accessor, already prepared.
    Result plan(const Context& ctx, const IWorldView& world);

private:
    /// Everything that requires libneupan types (the core planner, the point
    /// buffer, the cached path) lives behind this opaque handle, so the header
    /// never includes libneupan or Eigen.
    struct Impl;
    std::unique_ptr<Impl> impl_;

    // Cached parameter values that plan() needs.  Only the fields actually
    // used are kept: neupan_planner::Params carries an rclcpp::Time stamp, so
    // a whole-struct copy would be both misleading and pointless.
    double ref_speed_{4.0};
    int    solver_fail_grace_{5};
    double stall_speed_{0.02};
    double stall_timeout_{3.0};

    // Fallback used only until setLogger() injects the layer's child logger.
    rclcpp::Logger logger_{rclcpp::get_logger("NeuPanPlanner")};
};

}  // namespace finenav::example
