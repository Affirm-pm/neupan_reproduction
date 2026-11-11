// Copyright (c) 2026. IWIN-FINS Lab, Shanghai Jiao Tong University.
#pragma once
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>

namespace finenav::example {
// First failure always produces a stop; only a sustained outage faults the layer.
struct FailureWatch {
    using Clock = std::chrono::steady_clock;
    bool active{false};
    Clock::time_point since{};
    void reset() { active=false; }
    bool expired(Clock::time_point now, double timeout) {
        if (!active) { active=true; since=now; return false; }
        return std::chrono::duration<double>(now-since).count()>=timeout;
    }
};

// Check every point along a constant command for one control interval followed
// by braking both velocity components at the configured acceleration bounds.
// The circumradius bounds corner motion between samples. A conservative bound
// also covers the midpoint integration error in the translating braking arc.
template<class Points>
double sweptClearance(const Points& points, double x, double y, double yaw,
                      double v, double w, double length, double width,
                      double wheelbase, double linear_accel, double angular_accel,
                      double interval) {
    if (!std::isfinite(x)||!std::isfinite(y)||!std::isfinite(yaw)||
        !std::isfinite(v)||!std::isfinite(w)) return 0.0;
    const double brake=std::max(std::abs(v)/linear_accel,std::abs(w)/angular_accel);
    const double duration=interval+brake;
    const double radius=std::hypot((length+std::abs(wheelbase))/2.0,width/2.0);
    const double rate=std::abs(v)+radius*std::abs(w);
    const int steps=std::max(1,static_cast<int>(std::ceil(std::max(duration/0.02,rate*duration/0.01))));
    const double h=duration/steps;
    const double margin=rate*h + std::abs(v)*std::abs(w)*h*duration;
    double clearance=std::numeric_limits<double>::infinity();
    for(int i=0;i<=steps;++i) {
        const double c=std::cos(yaw),s=std::sin(yaw);
        for(const auto& p:points) {
            const double dx=p.x-x,dy=p.y-y;
            if (!std::isfinite(dx)||!std::isfinite(dy)) return 0.0;
            const double bx=c*dx+s*dy-wheelbase/2.0,by=-s*dx+c*dy;
            clearance=std::min(clearance,std::hypot(std::max(std::abs(bx)-length/2.0,0.0),
                                                  std::max(std::abs(by)-width/2.0,0.0))-margin);
        }
        const double t=(i+0.5)*h;
        const double braking=std::max(t-interval,0.0);
        const double vt=std::copysign(std::max(0.0,std::abs(v)-linear_accel*braking),v);
        const double wt=std::copysign(std::max(0.0,std::abs(w)-angular_accel*braking),w);
        x+=h*vt*std::cos(yaw+wt*h/2.0);
        y+=h*vt*std::sin(yaw+wt*h/2.0);
        yaw+=h*wt;
    }
    return clearance;
}
} // namespace finenav::example
