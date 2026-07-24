// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.
//
// ============================================================================
//  bench_localizer.cpp  —  Full-component benchmark for the EskfLocalizer engine
// ============================================================================
//
//  Measures the real per-tick cost of the whole localizer engine (ESKF math PLUS
//  message extraction, observation queues, smoothing reprocessing, Mahalanobis
//  gate, observe* Jacobians, delay-step mapping, cache + state buffer).
//
//  It drives the ROS-free inner engine `EskfLocalizer` directly — construct,
//  feed synthetic observations, and call `step(now)` on a virtual clock — exactly
//  like nav2 MPPI's `computeVelocityCommands` benchmark.  No node / timer /
//  executor / /clock: fully synchronous on the calling thread, so the reported
//  CPU/Time columns reflect the compute (not ROS dispatch).
//
//  Compare against bench_eskf (pure ESKF math): the delta is the wrapper overhead
//  (extract / observe / gate / smoothing / cache / buffer).  Build -O3 -DNDEBUG.
//  d (max_delay_step) is swept via max_delay_sec at 50 Hz.
// ============================================================================

#include <benchmark/benchmark.h>

#include <cmath>
#include <cstdint>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>

#include "finenav_localizer/eskf_localizer.hpp"

namespace {

namespace fn = finenav::localizer;

// The engine uses rclcpp::get_logger / rclcpp::Clock; init the context once so
// logging infrastructure is available (the engine itself starts no node/thread).
struct RosLockGuard {
    RosLockGuard()  { rclcpp::init(0, nullptr); }
    ~RosLockGuard() { rclcpp::shutdown(); }
};
RosLockGuard g_rclcpp;

constexpr int    kDelaySteps[] = {2, 5, 10, 16, 25, 50};
constexpr double kRateHz       = 50.0;

fn::EskfLocalizer::PoseMsg makePose(double x, double y, double yaw, const rclcpp::Time & stamp)
{
    fn::EskfLocalizer::PoseMsg m;
    m.header.stamp    = stamp;
    m.header.frame_id = "map";
    m.pose.pose.position.x    = x;
    m.pose.pose.position.y    = y;
    m.pose.pose.orientation.w = std::cos(yaw * 0.5);
    m.pose.pose.orientation.z = std::sin(yaw * 0.5);
    m.pose.covariance.fill(0.0);
    m.pose.covariance[0]  = 1e-2; m.pose.covariance[7]  = 1e-2;
    m.pose.covariance[14] = 1e-4; m.pose.covariance[21] = 1e-4;
    m.pose.covariance[28] = 1e-4; m.pose.covariance[35] = 1e-2;
    return m;
}

fn::EskfLocalizer::TwistMsg makeTwist(double vx, double wz, const rclcpp::Time & stamp)
{
    fn::EskfLocalizer::TwistMsg m;
    m.header.stamp    = stamp;
    m.header.frame_id = "base_link";
    m.twist.twist.linear.x  = vx;
    m.twist.twist.angular.z = wz;
    m.twist.covariance.fill(0.0);
    m.twist.covariance[0]  = 1e-2; m.twist.covariance[7]  = 1e-4;
    m.twist.covariance[14] = 1e-4; m.twist.covariance[21] = 1e-4;
    m.twist.covariance[28] = 1e-4; m.twist.covariance[35] = 1e-2;
    return m;
}

finenav_localizer::Params makeConfig(int d)
{
    finenav_localizer::Params p;
    p.update_rate_hz = kRateHz;
    p.max_delay_sec  = static_cast<double>(d - 1) / kRateHz;  // → max_delay_step = d
    return p;
}

// A warmed-up engine with a virtual clock; drive() = feed + one step().
class Fixture {
public:
    explicit Fixture(int d)
        : dt_ns_(static_cast<int64_t>(1e9 / kRateHz))
        , now_ns_(1'000'000'000LL)  // start at t = 1 s
        , loc_(std::make_unique<fn::EskfLocalizer>(makeConfig(d)))
    {
        loc_->setInitialPose(makePose(0.0, 0.0, 0.0, rclcpp::Time(now_ns_, RCL_ROS_TIME)));
        for (int i = 0; i < d + 2; ++i) {
            drive(true, true);  // fill covariance + history + smoothing queues
        }
    }

    void drive(bool feed_pose, bool feed_twist)
    {
        now_ns_ += dt_ns_;
        const rclcpp::Time stamp(now_ns_, RCL_ROS_TIME);
        if (feed_pose)  loc_->feedObservation(makePose(0.0, 0.0, 0.0, stamp));
        if (feed_twist) loc_->feedObservation(makeTwist(0.3, 0.05, stamp));
        loc_->step(stamp);
        benchmark::ClobberMemory();
    }

private:
    int64_t                            dt_ns_;
    int64_t                            now_ns_;
    std::unique_ptr<fn::EskfLocalizer> loc_;
};

void BM_Step_Empty(benchmark::State& state)      // predict + cache only (baseline)
{
    Fixture f(static_cast<int>(state.range(0)));
    for (auto _ : state) { f.drive(false, false); }
}

void BM_Step_Pose(benchmark::State& state)        // + 1 pose / tick
{
    Fixture f(static_cast<int>(state.range(0)));
    for (auto _ : state) { f.drive(true, false); }
}

void BM_Step_PoseTwist(benchmark::State& state)   // + 1 pose + 1 twist / tick (eval-like)
{
    Fixture f(static_cast<int>(state.range(0)));
    for (auto _ : state) { f.drive(true, true); }
}

void RegisterAll()
{
    auto add = [](const char* name, benchmark::internal::Function* fn) {
        auto* b = benchmark::RegisterBenchmark(name, fn)->Unit(benchmark::kMicrosecond);
        for (int d : kDelaySteps) { b->Arg(d); }
    };
    add("BM_Step_Empty",     BM_Step_Empty);
    add("BM_Step_Pose",      BM_Step_Pose);
    add("BM_Step_PoseTwist", BM_Step_PoseTwist);
}

}  // namespace

int main(int argc, char** argv)
{
    // Silence the engine's per-tick logging so console I/O never enters a
    // timed region (see the eval WARN-storm lesson).
    static_cast<void>(rcutils_logging_set_logger_level(
        "finenav.EskfLocalizer", RCUTILS_LOG_SEVERITY_ERROR));
    static_cast<void>(rcutils_logging_set_logger_level(
        "finenav.ESKF", RCUTILS_LOG_SEVERITY_ERROR));

    RegisterAll();
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
        return 1;
    }
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
