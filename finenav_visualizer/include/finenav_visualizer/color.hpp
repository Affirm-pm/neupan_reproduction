// Copyright (c) 2026.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#pragma once

#include <vector>

namespace finenav::viz {

/// The named palette.  A name is a colour *source*, not a colour: Color is what
/// carries values, and every colour parameter in this package takes Color.
/// A ColorName converts to it implicitly, so a name is still written plainly at
/// the call site.
enum class ColorName {
    BLACK, BROWN, BLUE, CYAN, GREY, DARK_GREY, GREEN, LIME_GREEN,
    MAGENTA, ORANGE, PURPLE, RED, PINK, WHITE, YELLOW,
    TRANSLUCENT, TRANSLUCENT_LIGHT, TRANSLUCENT_DARK,
    RAND, CLEAR, DEFAULT
};

/// An RGBA colour, one component per channel, each in [0, 1].
///
/// This is the single colour type on the public surface — markers and point
/// clouds both take it, so a colour is written the same way wherever it goes.
/// It is a plain value: owns nothing, copies freely, and carries its own alpha
/// (there is no separate buffer-wide alpha to interact with).
///
/// Both constructors convert implicitly, which is what keeps call sites plain:
///
///     markers.addSphere(goal, 0.15, ColorName::RED);             // from palette
///     markers.addLineStrip(path, Color{0.1f, 0.4f, 0.9f});       // computed
///     markers.addLineStrip(path, {0.1f, 0.4f, 0.9f, 0.35f});     // …with alpha
///
/// Why each is safe: ColorName is a closed enum, so the first is a total
/// mapping with no surprises.  The second takes several arguments, and a
/// multi-argument constructor is never an implicit-conversion source — it
/// answers a braced list at the call site and nothing else.  So neither can
/// swallow an unrelated value that was not written as a colour.
struct Color {
    float r{0.7f};
    float g{0.7f};
    float b{0.7f};
    float a{1.0f};

    /// An unset colour is grey, matching ColorName::DEFAULT.
    Color() = default;

    Color(ColorName name) noexcept;

    /// One type end to end: the parameters are @c float and so is the storage,
    /// so a channel never converts between the call site and the colour a shape
    /// carries — there is no cast in this constructor to get wrong.  @c float
    /// is also what the wire format holds, so a caller reads back exactly what
    /// was sent, with no second rounding at the message boundary.
    ///
    /// Braced lists still work at every call site.  The narrowing diagnostic
    /// applies to aggregate initialization; @c Color has constructors, so
    /// @c Color{0.1,0.4,0.9} and double channels from a computation resolve by
    /// ordinary overload resolution and convert here, once, on the way in.
    constexpr Color(float red, float green, float blue, float alpha = 1.0f) noexcept
        : r(red), g(green), b(blue), a(alpha) {}
};

/// A named multi-stop colormap — a fixed palette a scalar field is drawn
/// through.  TURBO is the modern replacement for the rainbow "jet" look
/// (monotonic luminance, no false banding); VIRIDIS is the perceptually
/// uniform default for ordered data such as height or cost.
enum class ColorMap {
    TURBO,
    VIRIDIS,
};

/// A scalar-to-colour rule: the range a dataset spans, and the colour band it
/// is drawn through.  A value object — configure it once per dataset, then hand
/// it to every point so the range and the band are said once, not per point.
class ColorScale {
public:
    /// Two-stop linear ramp: @p low at vmin, @p high at vmax.
    ColorScale(double vmin, double vmax,
               const Color& low = ColorName::BLUE,
               const Color& high = ColorName::RED);

    /// A named multi-stop colormap over [vmin, vmax].
    explicit ColorScale(double vmin, double vmax, ColorMap map);

    /// The colour @p value is drawn in.  The value is mapped through [vmin,
    /// vmax] and clamped, so anything outside the range saturates at the
    /// endpoint colour.  A degenerate range (vmin >= vmax) draws the midpoint
    /// colour.
    Color at(double value) const;

private:
    // Stop positions in [0, 1], ascending, one colour each; size >= 2.
    std::vector<double> positions_;
    std::vector<Color> colors_;
    double vmin_;
    double vmax_;
};

}  // namespace finenav::viz
