// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/color.hpp"

#include <random>

namespace finenav::viz {

namespace {

/// Keeps the table below readable: what a reader checks per case is the three
/// channels, so each case is exactly that and nothing else.  The channels are
/// written as float literals so the palette converts nothing on its way in.
Color rgb(float r, float g, float b, float a = 1.0f) {
    return Color{r, g, b, a};
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
        static std::mt19937 rng(42);
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

}  // namespace finenav::viz
