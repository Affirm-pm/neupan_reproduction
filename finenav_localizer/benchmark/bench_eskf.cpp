// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.


//  SETUP / WORKLOAD
//  ─────────────────────────────────────────────────────────────────────────
//  • Type under test:  ESKFd = ESKF<NavigationModel<double>>.
//  • Sweep parameter:  d = max_delay_step ∈ {2, 5, 10, 25, 50, 100}.
//        The internal covariance is an extended matrix of dimension N = 12·d,
//        so d is the knob that drives the dominant cost.  The "/d" suffix in
//        each row name is this value.
//  • Warm-up:  each filter is advanced with `predict()` (d + 2) times before
//        timing, so the extended covariance AND the nominal history are fully
//        populated — update() then exercises the real cross-covariance and
//        history-application paths, not a cold-start shortcut.
//  • Observations:  synthetic, fixed matrices (SingleObs / DualObs).  Only the
//        matrix SHAPES matter for timing; the numeric values do not.  R is kept
//        well-conditioned so update() never hits its NaN / degenerate-gain
//        early-return and always runs the full update path every iteration.
//  • Build:  compile with -O3 -DNDEBUG (Release).  Debug builds are meaningless
//        for these numbers (Eigen asserts + no inlining).


#include <benchmark/benchmark.h>

#include <Eigen/Core>

#include "finenav_localizer/eskf.hpp"

namespace {

using finenav::localizer::ESKFd;
using PMat = ESKFd::PMat;                 // 12×12 leading covariance block
constexpr int kDim = ESKFd::kDim;         // 12

// Delay-slot sweep.  Extended covariance dimension is N = kDim · d.
constexpr int kDelaySteps[] = {2, 5, 10, 25, 50, 100};

// ── Synthetic, representative observation payloads ────────────────────────────
// Values are irrelevant to timing (cost is dimension-driven); only the matrix
// shapes matter.  R is kept well-conditioned so update() never hits its NaN /
// degenerate-gain early-return and always exercises the full update path.

struct SingleObs {
    Eigen::Matrix<double, 6, 1>    innov;
    Eigen::Matrix<double, 6, kDim> H;
    Eigen::Matrix<double, 6, 6>    R;
    SingleObs()
    {
        H.setZero();
        H.block<3, 3>(0, 0).setIdentity();   // pose-like: ∂p/∂δp
        H.block<3, 3>(3, 3).setIdentity();   //            ∂θ/∂δθ
        innov.setConstant(0.01);
        R = Eigen::Matrix<double, 6, 6>::Identity() * 0.05;
    }
};

struct DualObs {
    Eigen::Matrix<double, 6, 1>    innov;
    Eigen::Matrix<double, 6, kDim> H_to;
    Eigen::Matrix<double, 6, kDim> H_from;
    Eigen::Matrix<double, 6, 6>    R;
    DualObs()
    {
        H_to.setZero();
        H_to.block<3, 3>(0, 0).setIdentity();
        H_to.block<3, 3>(3, 3).setIdentity();

        H_from.setZero();
        H_from.block<3, 3>(0, 0) = -Eigen::Matrix3d::Identity();
        H_from.block<3, 3>(0, 3) =  Eigen::Matrix3d::Identity() * 0.5;  // stand-in skew
        H_from.block<3, 3>(3, 3) = -Eigen::Matrix3d::Identity();

        innov.setConstant(0.01);
        R = Eigen::Matrix<double, 6, 6>::Identity() * 0.05;
    }
};

// Construct an ESKF with d delay slots and warm it up so that both the extended
// covariance and the nominal history are fully populated — this makes update()
// exercise the delayed-slot cross-covariance and history-application paths.
ESKFd makeWarmFilter(int d)
{
    ESKFd eskf({}, d, PMat::Identity() * 0.1);
    for (int i = 0; i < d + 2; ++i) {
        eskf.predict(0.01);
    }
    return eskf;
}

// ── predict(dt) ───────────────────────────────────────────────────────────────
void BM_Predict(benchmark::State& state)
{
    const int d = static_cast<int>(state.range(0));
    ESKFd eskf = makeWarmFilter(d);

    for (auto _ : state) {
        eskf.predict(0.01);
        double leak = eskf.getCovariance()(0, 0);
        benchmark::DoNotOptimize(leak);
    }
}

// ── single-slot update<6>(), current tick (delay_step = 0) ────────────────────
void BM_UpdateSingle(benchmark::State& state)
{
    const int d = static_cast<int>(state.range(0));
    ESKFd eskf = makeWarmFilter(d);
    const SingleObs o;

    for (auto _ : state) {
        bool ok = eskf.update<6>(o.innov, o.H, o.R, /*delay_step=*/0);
        benchmark::DoNotOptimize(ok);
        benchmark::ClobberMemory();
    }
}

// ── dual-epoch update<6>() (relative-pose / odometry) ─────────────────────────
void BM_UpdateDual(benchmark::State& state)
{
    const int d       = static_cast<int>(state.range(0));
    const int ds_to   = std::min(d - 1, 1);
    const int ds_from = std::min(d - 1, d / 2 + 1);
    ESKFd eskf = makeWarmFilter(d);
    const DualObs o;

    for (auto _ : state) {
        bool ok = eskf.update<6>(o.innov, o.H_to, o.H_from, o.R, ds_to, ds_from);
        benchmark::DoNotOptimize(ok);
        benchmark::ClobberMemory();
    }
}

// ── representative production tick: predict + pose update + twist update ──────
void BM_FullTick(benchmark::State& state)
{
    const int d = static_cast<int>(state.range(0));
    ESKFd eskf = makeWarmFilter(d);
    const SingleObs pose;
    const SingleObs twist;

    for (auto _ : state) {
        eskf.predict(0.01);
        eskf.update<6>(pose.innov,  pose.H,  pose.R,  0);
        eskf.update<6>(twist.innov, twist.H, twist.R, 0);
        double leak = eskf.getState().p.x();
        benchmark::DoNotOptimize(leak);
        benchmark::ClobberMemory();
    }
}

// ── registration ──────────────────────────────────────────────────────────────
void RegisterAll()
{
    auto add = [](const char* name, benchmark::internal::Function* fn) {
        auto* b = benchmark::RegisterBenchmark(name, fn)
                      ->Unit(benchmark::kMicrosecond);
        for (int d : kDelaySteps) {
            b->Arg(d);
        }
    };
    add("BM_Predict",      BM_Predict);
    add("BM_UpdateSingle", BM_UpdateSingle);
    add("BM_UpdateDual",   BM_UpdateDual);
    add("BM_FullTick",     BM_FullTick);
}

}  // namespace

int main(int argc, char** argv)
{
    RegisterAll();
    benchmark::Initialize(&argc, argv);
    if (benchmark::ReportUnrecognizedArguments(argc, argv)) {
        return 1;
    }
    benchmark::RunSpecifiedBenchmarks();
    benchmark::Shutdown();
    return 0;
}
