// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "finenav_visualizer/color.hpp"

#include <random>

namespace finenav::viz {

namespace {

std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a) {
    std_msgs::msg::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

}  // namespace

std_msgs::msg::ColorRGBA getColor(ColorName c) {
    switch (c) {
    case ColorName::BLACK:              return makeColor(0.0f, 0.0f, 0.0f, 1.0f);
    case ColorName::BROWN:              return makeColor(0.6f, 0.3f, 0.1f, 1.0f);
    case ColorName::BLUE:               return makeColor(0.1f, 0.3f, 1.0f, 1.0f);
    case ColorName::CYAN:               return makeColor(0.0f, 1.0f, 1.0f, 1.0f);
    case ColorName::GREY:               return makeColor(0.5f, 0.5f, 0.5f, 1.0f);
    case ColorName::DARK_GREY:          return makeColor(0.3f, 0.3f, 0.3f, 1.0f);
    case ColorName::GREEN:              return makeColor(0.1f, 0.9f, 0.1f, 1.0f);
    case ColorName::LIME_GREEN:         return makeColor(0.5f, 1.0f, 0.3f, 1.0f);
    case ColorName::MAGENTA:            return makeColor(1.0f, 0.1f, 0.6f, 1.0f);
    case ColorName::ORANGE:             return makeColor(1.0f, 0.6f, 0.1f, 1.0f);
    case ColorName::PURPLE:             return makeColor(0.6f, 0.2f, 0.8f, 1.0f);
    case ColorName::RED:                return makeColor(1.0f, 0.1f, 0.1f, 1.0f);
    case ColorName::PINK:               return makeColor(1.0f, 0.5f, 0.6f, 1.0f);
    case ColorName::WHITE:              return makeColor(1.0f, 1.0f, 1.0f, 1.0f);
    case ColorName::YELLOW:             return makeColor(1.0f, 0.9f, 0.1f, 1.0f);
    case ColorName::TRANSLUCENT:        return makeColor(0.1f, 0.1f, 0.1f, 0.25f);
    case ColorName::TRANSLUCENT_LIGHT:  return makeColor(0.1f, 0.1f, 0.1f, 0.10f);
    case ColorName::TRANSLUCENT_DARK:   return makeColor(0.1f, 0.1f, 0.1f, 0.50f);
    case ColorName::CLEAR:              return makeColor(1.0f, 1.0f, 1.0f, 0.0f);
    case ColorName::RAND: {
        static std::mt19937 rng(42);
        std::uniform_real_distribution<float> d(0.1f, 0.9f);
        return makeColor(d(rng), d(rng), d(rng), 1.0f);
    }
    case ColorName::DEFAULT:            return makeColor(0.7f, 0.7f, 0.7f, 1.0f);
    }
    return makeColor(0.7f, 0.7f, 0.7f, 1.0f);
}

}  // namespace finenav::viz
