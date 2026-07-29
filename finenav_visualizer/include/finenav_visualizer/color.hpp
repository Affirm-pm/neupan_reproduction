// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <std_msgs/msg/color_rgba.hpp>

namespace finenav::viz {

enum class ColorName {
    BLACK, BROWN, BLUE, CYAN, GREY, DARK_GREY, GREEN, LIME_GREEN,
    MAGENTA, ORANGE, PURPLE, RED, PINK, WHITE, YELLOW,
    TRANSLUCENT, TRANSLUCENT_LIGHT, TRANSLUCENT_DARK,
    RAND, CLEAR, DEFAULT
};

/// Convert ColorName to rviz ColorRGBA.
std_msgs::msg::ColorRGBA getColor(ColorName c);

}  // namespace finenav::viz
