// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include <gtest/gtest.h>

#include "finenav_visualizer/color.hpp"

namespace {

using finenav::viz::Color;
using finenav::viz::ColorMap;
using finenav::viz::ColorName;
using finenav::viz::ColorScale;

// Pins the three channels and nothing else — what a reader checks per case is
// exactly what the scale promises, so a failure names the channel that moved.
void expectRgb(const Color& c, float r, float g, float b) {
    EXPECT_FLOAT_EQ(c.r, r);
    EXPECT_FLOAT_EQ(c.g, g);
    EXPECT_FLOAT_EQ(c.b, b);
}

}  // namespace

// A two-stop ramp is a property of its endpoints: the ends are exactly the
// colours that were named, with no rounding sneaking in through the stop table.
TEST(ColorScaleTest, TwoStopEndpointsAreTheNamedColors) {
    const ColorScale scale{0.0, 1.0, ColorName::BLACK, ColorName::WHITE};

    expectRgb(scale.at(0.0), 0.0f, 0.0f, 0.0f);
    expectRgb(scale.at(1.0), 1.0f, 1.0f, 1.0f);
}

TEST(ColorScaleTest, TwoStopMidpointIsHalfway) {
    const ColorScale scale{0.0, 1.0, ColorName::BLACK, ColorName::WHITE};

    expectRgb(scale.at(0.5), 0.5f, 0.5f, 0.5f);
}

// Outside the range is not an error and not extrapolation: the endpoints are
// saturated, so a dataset with an outlier still draws entirely inside its band.
TEST(ColorScaleTest, TwoStopClampsOutsideTheRange) {
    const ColorScale scale{0.0, 1.0, ColorName::BLACK, ColorName::WHITE};

    const Color below = scale.at(-1.0);
    const Color above = scale.at(2.0);
    const Color low = scale.at(0.0);
    const Color high = scale.at(1.0);

    expectRgb(below, low.r, low.g, low.b);
    expectRgb(above, high.r, high.g, high.b);
}

// The range is a pair of values, not a normalized [0, 1] position: a scale over
// [10, 20] puts the midpoint colour at 15, and clamps at 5 and 25.
TEST(ColorScaleTest, MapsAnArbitraryRange) {
    const ColorScale scale{10.0, 20.0, ColorName::BLACK, ColorName::WHITE};

    expectRgb(scale.at(15.0), 0.5f, 0.5f, 0.5f);
    expectRgb(scale.at(5.0), 0.0f, 0.0f, 0.0f);
    expectRgb(scale.at(25.0), 1.0f, 1.0f, 1.0f);
}

// A degenerate range has no gradient to draw, so every value lands on the
// midpoint colour rather than dividing by zero or drawing an endpoint at random.
TEST(ColorScaleTest, DegenerateRangeDrawsTheMidpoint) {
    const ColorScale scale{1.0, 1.0, ColorName::BLACK, ColorName::WHITE};

    expectRgb(scale.at(0.0), 0.5f, 0.5f, 0.5f);
    expectRgb(scale.at(5.0), 0.5f, 0.5f, 0.5f);
}

TEST(ColorScaleTest, ViridisEndpointsAreItsAnchors) {
    const ColorScale scale{0.0, 1.0, ColorMap::VIRIDIS};

    expectRgb(scale.at(0.0), 0.267f, 0.005f, 0.329f);
    expectRgb(scale.at(1.0), 0.993f, 0.906f, 0.144f);
}

TEST(ColorScaleTest, TurboEndpointsAreItsAnchors) {
    const ColorScale scale{0.0, 1.0, ColorMap::TURBO};

    expectRgb(scale.at(0.0), 0.190f, 0.072f, 0.232f);
    expectRgb(scale.at(1.0), 0.480f, 0.054f, 0.164f);
}

// The midpoint of a multi-stop map is midway between two anchors, not one of
// them: TURBO's band passes through green there, which is what distinguishes a
// real colormap from a two-colour fade.
TEST(ColorScaleTest, TurboMidpointIsGreenish) {
    const ColorScale scale{0.0, 1.0, ColorMap::TURBO};

    const Color mid = scale.at(0.5);

    EXPECT_GT(mid.g, mid.r);
    EXPECT_GT(mid.g, mid.b);
}
