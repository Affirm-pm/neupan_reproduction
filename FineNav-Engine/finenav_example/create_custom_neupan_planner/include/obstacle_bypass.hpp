// Copyright (c) 2026. IWIN-FINS Lab, Shanghai Jiao Tong University.
#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace finenav::example {
// Small, static-obstacle reference repair for single goals. The MPC remains in
// charge of motion and the independent footprint gate remains in charge of safety.
struct ObstacleBypass {
    struct Point { double x{}, y{}; };
    struct Options {
        bool enabled{true};
        double lookahead{2.5};
        double margin{0.10};
        double max_offset{1.5};
    } options;
    enum class Change { None, Installed, Cleared };
    bool active{false};
    std::array<Point,4> route{};
    Point origin{}, direction{};
    double exit_projection{};
    int side{0};

    void reset() { active=false; side=0; }
    static double distance(Point a,Point b) { return std::hypot(a.x-b.x,a.y-b.y); }
    static double segmentDistance(Point p,Point a,Point b) {
        const double dx=b.x-a.x,dy=b.y-a.y,den=dx*dx+dy*dy;
        const double t=den>1e-12 ? std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/den,0.0,1.0) : 0.0;
        return std::hypot(p.x-a.x-t*dx,p.y-a.y-t*dy);
    }
    template<class Points>
    static bool clearSegment(Point a,Point b,const Points& points,double radius) {
        for(const auto& p:points)
            if(!std::isfinite(p.x)||!std::isfinite(p.y)||segmentDistance({p.x,p.y},a,b)<radius)
                return false;
        return true;
    }
    template<class Points>
    Change update(Point here,Point goal,double yaw,const Points& points,
                  double length,double width,double wheelbase) {
        // Circumradius safely encloses the rectangle for any reference heading.
        const double radius=std::hypot((length+std::abs(wheelbase))/2.0,width/2.0)+options.margin;
        if(active) {
            // No frame-by-frame path replacement or left/right switching. Wait
            // until past the obstacle AND the direct rejoin is geometrically clear.
            const double progress=(here.x-origin.x)*direction.x+(here.y-origin.y)*direction.y;
            if((!options.enabled || progress>=exit_projection-0.15) &&
                clearSegment(here,goal,points,radius)) {
                reset(); return Change::Cleared;
            }
            return Change::None;
        }
        const double distance_to_goal=distance(here,goal);
        if(!options.enabled || distance_to_goal<1.0) return Change::None;
        const Point dir{(goal.x-here.x)/distance_to_goal,(goal.y-here.y)/distance_to_goal};
        double first=std::numeric_limits<double>::infinity();
        // Restrict detection to the forward corridor, not unrelated side walls.
        for(const auto& p:points) {
            const double dx=p.x-here.x,dy=p.y-here.y;
            const double along=dx*dir.x+dy*dir.y,across=-dx*dir.y+dy*dir.x;
            if(along>radius && along<std::min(options.lookahead,distance_to_goal-radius) &&
                std::abs(across)<radius) first=std::min(first,along);
        }
        if(!std::isfinite(first)) return Change::None;
        // Bound the obstacle group; a longer wall will fail candidate validation
        // and remain the responsibility of a global planner / fault recovery.
        double last=first;
        for(const auto& p:points) {
            const double dx=p.x-here.x,dy=p.y-here.y;
            const double along=dx*dir.x+dy*dir.y,across=-dx*dir.y+dy*dir.x;
            if(along>=first && along<=first+1.0 && std::abs(across)<radius)
                last=std::max(last,along);
        }
        const double entry=std::max(0.25,first-0.35);
        const double exit=last+radius+0.15;
        if(exit>=distance_to_goal-0.2) return Change::None;
        double best=std::numeric_limits<double>::infinity();
        std::array<Point,4> candidate;
        for(int sign:{1,-1}) {
            for(double offset=0.5;offset<=options.max_offset+1e-9;offset+=0.2) {
                const auto waypoint=[&](double along){return Point{
                    here.x+along*dir.x-sign*offset*dir.y,
                    here.y+along*dir.y+sign*offset*dir.x};};
                candidate={here,waypoint(entry),waypoint(exit),goal};
                bool clear=true;double cost=0;
                for(int i=0;i<3;++i) {
                    if(!clearSegment(candidate[i],candidate[i+1],points,radius)){clear=false;break;}
                    cost+=distance(candidate[i],candidate[i+1]);
                }
                if(!clear) continue;
                const double heading=std::atan2(candidate[1].y-here.y,candidate[1].x-here.x);
                cost+=0.25*std::abs(std::atan2(std::sin(heading-yaw),std::cos(heading-yaw)));
                if(cost<best-1e-6){best=cost;route=candidate;side=sign;}
            }
        }
        if(!std::isfinite(best)) return Change::None;
        active=true;origin=here;direction=dir;exit_projection=exit;
        return Change::Installed;
    }
};
} // namespace finenav::example
