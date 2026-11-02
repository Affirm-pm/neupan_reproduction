// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/color.hpp"

#include <algorithm>
#include <random>
#include <utility>
#include <vector>

namespace finenav::viz {

namespace {

/// Keeps the table below readable: what a reader checks per case is the three
/// channels, so each case is exactly that and nothing else.  The channels are
/// written as float literals so the palette converts nothing on its way in.
Color rgb(float r, float g, float b, float a = 1.0f) {
    return Color{r, g, b, a};
}

/// Canonical 5-stop anchors, positions in [0,1].  Representative anchors, not
/// the full 256-entry table — the point is a recognisable, monotonic-luminance
/// ramp, not pixel-exact fidelity.
std::vector<std::pair<double, Color>> colormapStops(ColorMap map) {
    using C = Color;
    switch (map) {
    case ColorMap::VIRIDIS:
        return {{0.00, C{0.267f, 0.005f, 0.329f}},
                {0.25, C{0.230f, 0.322f, 0.545f}},
                {0.50, C{0.127f, 0.568f, 0.556f}},
                {0.75, C{0.369f, 0.789f, 0.382f}},
                {1.00, C{0.993f, 0.906f, 0.144f}}};
    case ColorMap::TURBO:
        return {{0.00, C{0.190f, 0.072f, 0.232f}},
                {0.25, C{0.132f, 0.551f, 0.892f}},
                {0.50, C{0.237f, 0.874f, 0.500f}},
                {0.75, C{0.953f, 0.731f, 0.108f}},
                {1.00, C{0.480f, 0.054f, 0.164f}}};
    }
    return {};
}

/// Linear interpolation of all four channels, so alpha is carried too.
///
/// The delta is taken in double, not float: a float subtraction re-rounds
/// before the mix, which leaves the t == 1 endpoint a few ULP short of the stop
/// it was built from.  A ramp that does not quite reach the colour it
/// advertises is the one error a colormap cannot afford, so the subtraction is
/// deliberately the wide one.
Color lerp(const Color& a, const Color& b, double t) {
    return Color{static_cast<float>(a.r + (static_cast<double>(b.r) - a.r) * t),
                 static_cast<float>(a.g + (static_cast<double>(b.g) - a.g) * t),
                 static_cast<float>(a.b + (static_cast<double>(b.b) - a.b) * t),
                 static_cast<float>(a.a + (static_cast<double>(b.a) - a.a) * t)};
}

}  // namespace

Color::Color(ColorName name) noexcept {
    switch (name) {
    case ColorName::BLACK:              *this = rgb(0.0f, 0.0f, 0.0f);        return;
    case ColorName::BROWN:              *this = rgb(0.6f, 0.3f, 0.1f);        return;
    case ColorName::BLUE:               *this = rgb(0.1f, 0.3f, 1.0f);        return;
    case ColorName::CYAN:               *this = rgb(0.0f, 1.0f, 1.0f);        return;
    case ColorName::GREY:               *this = rgb(0.5f, 0.5f, 0.5f);        return;
    case ColorName::DARK_GREY:          *this = rgb(0.3f, 0.3f, 0.3f);        return;
    case ColorName::GREEN:              *this = rgb(0.1f, 0.9f, 0.1f);        return;
    case ColorName::LIME_GREEN:         *this = rgb(0.5f, 1.0f, 0.3f);        return;
    case ColorName::MAGENTA:            *this = rgb(1.0f, 0.1f, 0.6f);        return;
    case ColorName::ORANGE:             *this = rgb(1.0f, 0.6f, 0.1f);        return;
    case ColorName::PURPLE:             *this = rgb(0.6f, 0.2f, 0.8f);        return;
    case ColorName::RED:                *this = rgb(1.0f, 0.1f, 0.1f);        return;
    case ColorName::PINK:               *this = rgb(1.0f, 0.5f, 0.6f);        return;
    case ColorName::WHITE:              *this = rgb(1.0f, 1.0f, 1.0f);        return;
    case ColorName::YELLOW:             *this = rgb(1.0f, 0.9f, 0.1f);        return;
    case ColorName::TRANSLUCENT:        *this = rgb(0.1f, 0.1f, 0.1f, 0.25f); return;
    case ColorName::TRANSLUCENT_LIGHT:  *this = rgb(0.1f, 0.1f, 0.1f, 0.10f); return;
    case ColorName::TRANSLUCENT_DARK:   *this = rgb(0.1f, 0.1f, 0.1f, 0.50f); return;
    case ColorName::CLEAR:              *this = rgb(1.0f, 1.0f, 1.0f, 0.0f);  return;
    case ColorName::RAND: {
        // Per-thread, because a colour is drawn from algorithm threads: a single
        // process-global engine would be raced on by every thread that staged a
        // random-coloured shape at once.  Seeded from random_device rather than a
        // constant so two threads do not replay the identical sequence — the
        // sequence only has to be uncorrelated, not reproducible.
        thread_local std::mt19937 rng(std::random_device{}());
        std::uniform_real_distribution<float> d(0.1f, 0.9f);
        *this = rgb(d(rng), d(rng), d(rng));
        return;
    }
    case ColorName::DEFAULT:            *this = rgb(0.7f, 0.7f, 0.7f);        return;
    }
    // Unreachable while every enumerator is handled above; a name added to the
    // palette without a case here lands on the default member values (grey),
    // which is this type's "unset" colour.
}

ColorScale::ColorScale(double vmin, double vmax, const Color& low, const Color& high)
    : positions_{0.0, 1.0}, colors_{low, high}, vmin_(vmin), vmax_(vmax) {}

ColorScale::ColorScale(double vmin, double vmax, ColorMap map)
    : vmin_(vmin), vmax_(vmax) {
    const auto stops = colormapStops(map);
    positions_.reserve(stops.size());
    colors_.reserve(stops.size());
    for (const auto& [position, color] : stops) {
        positions_.push_back(position);
        colors_.push_back(color);
    }
}

Color ColorScale::at(double value) const {
    double t;
    if (vmax_ > vmin_) {
        t = std::clamp((value - vmin_) / (vmax_ - vmin_), 0.0, 1.0);
    } else {
        t = 0.5;  // degenerate range: midpoint, matching the old addColoredPoint
    }
    // Linear scan — the stop list is tiny (2 or 5).
    std::size_t i = 0;
    while (i + 1 < positions_.size() && positions_[i + 1] < t) {
        ++i;
    }
    const double span = positions_[i + 1] - positions_[i];
    const double u = span > 0.0 ? (t - positions_[i]) / span : 0.0;
    return lerp(colors_[i], colors_[i + 1], u);
}

}  // namespace finenav::viz
