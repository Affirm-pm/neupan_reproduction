// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>

namespace finenav::core {

struct Position3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

struct Region3D {
    Position3D min;
    Position3D max;
};

constexpr Position3D operator+(const Position3D& a, const Position3D& b) noexcept {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
constexpr Position3D operator-(const Position3D& a, const Position3D& b) noexcept {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
constexpr Position3D operator*(const Position3D& a, double s) noexcept {
    return {a.x * s, a.y * s, a.z * s};
}
constexpr Position3D operator*(double s, const Position3D& a) noexcept {
    return {a.x * s, a.y * s, a.z * s};
}

inline double norm(const Position3D& a) noexcept {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

inline Position3D abs(const Position3D& a) noexcept {
    return {std::abs(a.x), std::abs(a.y), std::abs(a.z)};
}

constexpr bool contains(const Region3D& r, const Position3D& p) noexcept {
    return p.x >= r.min.x && p.x <= r.max.x &&
           p.y >= r.min.y && p.y <= r.max.y &&
           p.z >= r.min.z && p.z <= r.max.z;
}

constexpr Position3D center(const Region3D& r) noexcept {
    return {(r.min.x + r.max.x) * 0.5,
            (r.min.y + r.max.y) * 0.5,
            (r.min.z + r.max.z) * 0.5};
}

constexpr Position3D sizes(const Region3D& r) noexcept {
    return {r.max.x - r.min.x, r.max.y - r.min.y, r.max.z - r.min.z};
}

}  // namespace finenav::core
