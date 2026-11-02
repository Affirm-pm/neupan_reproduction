// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <cmath>

namespace finenav::common {

/**
 * @brief The framework's geometry vocabulary: a point, and the box around it.
 *
 * Two plain structs and the handful of operations that read naturally on them.
 * They are the types every plugin writes in its own signatures — an IMapView
 * takes a Position3D and answers with one, a MapConcept returns a Region3D for
 * its window — so they are part of the plugin contract and cannot live
 * anywhere a plugin is unable to name.
 *
 * Deliberately POD and deliberately not Eigen.  A plugin author should be able
 * to write
 *
 *     struct MyMap {
 *         bool isInside(const finenav::common::Position3D& p) const { ... }
 *     };
 *
 * without Eigen being part of what they must understand, and the framework
 * converts at the one boundary where dense linear algebra actually happens
 * (finenav_core/robot_state/eigen_conversions.hpp).  Braced initialization —
 * Position3D{x, y, z} — is how these are meant to be written.
 *
 * It lives here, in finenav_common, because it is a convention ABOUT the
 * framework rather than a piece of its machinery: finenav_core, the collision
 * model, every example plugin and the visualizer all agree on the same two
 * types, and none of them owns them.  This is the ONLY spelling — a plugin
 * writes finenav::common::Position3D, and finenav_core deliberately does not
 * mirror it under a second name.
 */
struct Position3D {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

/**
 * @brief An axis-aligned box, given by its two extreme corners.
 *
 * Inclusive on all six faces: contains() is true for a point exactly on max.
 * That is the reading that makes a grid cell's box contain its own far corner,
 * so a point is never inside two neighbouring cells' boxes without being inside
 * the cell between them.
 */
struct Region3D {
    Position3D min;
    Position3D max;
};

// ── Operations ────────────────────────────────────────────────────────────
//
// Free functions, not members: they are how the vocabulary is USED, not what
// the types ARE, and a plugin that never does vector arithmetic on a
// Position3D should not have to look past the two fields above.  They are
// found by argument-dependent lookup, so no using-declaration is needed at a
// call site: `abs(a - b)` resolves here from anywhere a Position3D is named,
// finenav::core's own internals included.

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

/// Euclidean length.  Named rather than written `a.norm()` so that it reads the
/// same on a Position3D as it does on the Eigen types used elsewhere.
inline double norm(const Position3D& a) noexcept {
    return std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
}

/// Component-wise absolute value.
inline Position3D abs(const Position3D& a) noexcept {
    return {std::abs(a.x), std::abs(a.y), std::abs(a.z)};
}

/// True when @p p lies in @p r, faces inclusive.
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

/// Extent along each axis — never negative, since min is the low corner.
constexpr Position3D sizes(const Region3D& r) noexcept {
    return {r.max.x - r.min.x, r.max.y - r.min.y, r.max.z - r.min.z};
}

}  // namespace finenav::common
