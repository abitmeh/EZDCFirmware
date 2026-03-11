#include "Utilities/LinearPiecewiseLinearFunction.hpp"

#include <gtest/gtest.h>

#include <array>

// ─────────────────────────────────────────────
// lerp and invLerp
// ─────────────────────────────────────────────

TEST(Lerp, HalfwayFloat) {
    EXPECT_FLOAT_EQ(bldc::lerp(0.0f, 1.0f, 0.5f), 0.5f);
}

TEST(Lerp, AtStart) {
    EXPECT_FLOAT_EQ(bldc::lerp(2.0f, 4.0f, 0.0f), 2.0f);
}

TEST(Lerp, AtEnd) {
    EXPECT_FLOAT_EQ(bldc::lerp(2.0f, 4.0f, 1.0f), 4.0f);
}

TEST(Lerp, ExtrapolatesBeyondEnd) {
    EXPECT_FLOAT_EQ(bldc::lerp(0.0f, 1.0f, 2.0f), 2.0f);
}

TEST(Lerp, NegativeRange) {
    EXPECT_FLOAT_EQ(bldc::lerp(-1.0f, 1.0f, 0.5f), 0.0f);
}

TEST(InvLerp, HalfwayFloat) {
    EXPECT_FLOAT_EQ(bldc::invLerp(0.0f, 1.0f, 0.5f), 0.5f);
}

TEST(InvLerp, AtStart) {
    EXPECT_FLOAT_EQ(bldc::invLerp(2.0f, 4.0f, 2.0f), 0.0f);
}

TEST(InvLerp, AtEnd) {
    EXPECT_FLOAT_EQ(bldc::invLerp(2.0f, 4.0f, 4.0f), 1.0f);
}

TEST(InvLerp, NegativeRange) {
    EXPECT_FLOAT_EQ(bldc::invLerp(-1.0f, 1.0f, 0.0f), 0.5f);
}

TEST(LerpInvLerp, Roundtrip) {
    const float a = 3.0f, b = 7.0f, v = 5.0f;
    EXPECT_NEAR(bldc::lerp(a, b, bldc::invLerp(a, b, v)), v, 1e-5f);
}

// ─────────────────────────────────────────────
// Construction
// ─────────────────────────────────────────────

TEST(PiecewiseLinearFunction, ConstructFromArray) {
    using PLF = bldc::PiecewiseLinearFunction<float, 3>;
    std::array<PLF::Point, 3> pts = {{{0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f.startX(), 0.0f);
    EXPECT_FLOAT_EQ(f.endX(), 2.0f);
}

TEST(PiecewiseLinearFunction, ConstructFromCArray) {
    using PLF = bldc::PiecewiseLinearFunction<float, 3>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f.startX(), 0.0f);
    EXPECT_FLOAT_EQ(f.endX(), 2.0f);
}

TEST(PiecewiseLinearFunction, ConstructTwoPoints) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f.startX(), 0.0f);
    EXPECT_FLOAT_EQ(f.endX(), 1.0f);
}

// ─────────────────────────────────────────────
// Evaluation — boundary clamping
// ─────────────────────────────────────────────

TEST(PiecewiseLinearFunction, ClampsBelowStart) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{1.0f, 10.0f}, {2.0f, 20.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f(0.0f), 10.0f);
    EXPECT_FLOAT_EQ(f(-100.0f), 10.0f);
}

TEST(PiecewiseLinearFunction, ClampsAboveEnd) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{1.0f, 10.0f}, {2.0f, 20.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f(3.0f), 20.0f);
    EXPECT_FLOAT_EQ(f(100.0f), 20.0f);
}

TEST(PiecewiseLinearFunction, AtStartPoint) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 5.0f}, {1.0f, 10.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f(0.0f), 5.0f);
}

TEST(PiecewiseLinearFunction, AtEndPoint) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 5.0f}, {1.0f, 10.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f(1.0f), 10.0f);
}

// ─────────────────────────────────────────────
// Evaluation — interpolation
// ─────────────────────────────────────────────

TEST(PiecewiseLinearFunction, LinearMidpoint) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    PLF f(pts);
    EXPECT_NEAR(f(0.5f), 0.5f, 1e-5f);
}

TEST(PiecewiseLinearFunction, LinearQuarter) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {4.0f, 8.0f}};
    PLF f(pts);
    EXPECT_NEAR(f(1.0f), 2.0f, 1e-5f);
}

TEST(PiecewiseLinearFunction, NegativeSlope) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 10.0f}, {1.0f, 0.0f}};
    PLF f(pts);
    EXPECT_NEAR(f(0.5f), 5.0f, 1e-5f);
}

TEST(PiecewiseLinearFunction, FlatSegment) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 3.0f}, {1.0f, 3.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f(0.5f), 3.0f);
}

// ─────────────────────────────────────────────
// Evaluation — multiple segments
// ─────────────────────────────────────────────

TEST(PiecewiseLinearFunction, ThreePointsMidFirstSegment) {
    using PLF = bldc::PiecewiseLinearFunction<float, 3>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}};
    PLF f(pts);
    EXPECT_NEAR(f(0.5f), 0.5f, 1e-5f);
}

TEST(PiecewiseLinearFunction, ThreePointsMidSecondSegment) {
    using PLF = bldc::PiecewiseLinearFunction<float, 3>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 0.0f}};
    PLF f(pts);
    EXPECT_NEAR(f(1.5f), 0.5f, 1e-5f);
}

TEST(PiecewiseLinearFunction, ThreePointsAtKnot) {
    using PLF = bldc::PiecewiseLinearFunction<float, 3>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 2.0f}, {2.0f, 0.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f(1.0f), 2.0f);
}

TEST(PiecewiseLinearFunction, ManySegments) {
    using PLF = bldc::PiecewiseLinearFunction<float, 5>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 1.0f}, {2.0f, 1.0f}, {3.0f, 0.0f}, {4.0f, 2.0f}};
    PLF f(pts);
    EXPECT_NEAR(f(0.5f), 0.5f, 1e-5f);  // first segment
    EXPECT_NEAR(f(1.5f), 1.0f, 1e-5f);  // flat segment
    EXPECT_NEAR(f(2.5f), 0.5f, 1e-5f);  // falling segment
    EXPECT_NEAR(f(3.5f), 1.0f, 1e-5f);  // rising segment
}

TEST(PiecewiseLinearFunction, UnevenSegmentSpacing) {
    using PLF = bldc::PiecewiseLinearFunction<float, 3>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {0.1f, 1.0f}, {10.0f, 2.0f}};
    PLF f(pts);
    EXPECT_NEAR(f(0.05f), 0.5f, 1e-4f);
    EXPECT_NEAR(f(5.05f), 1.5f, 1e-4f);
}

// ─────────────────────────────────────────────
// Non-float types
// ─────────────────────────────────────────────

TEST(PiecewiseLinearFunction, DoubleXAndY) {
    using PLF = bldc::PiecewiseLinearFunction<double, 2, double, double>;
    PLF::Point pts[] = {{0.0, 0.0}, {1.0, 1.0}};
    PLF f(pts);
    EXPECT_NEAR(f(0.5), 0.5, 1e-10);
}

// ─────────────────────────────────────────────
// startX and endX
// ─────────────────────────────────────────────

TEST(PiecewiseLinearFunction, StartX) {
    using PLF = bldc::PiecewiseLinearFunction<float, 3>;
    PLF::Point pts[] = {{-5.0f, 0.0f}, {0.0f, 1.0f}, {5.0f, 0.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f.startX(), -5.0f);
}

TEST(PiecewiseLinearFunction, EndX) {
    using PLF = bldc::PiecewiseLinearFunction<float, 3>;
    PLF::Point pts[] = {{-5.0f, 0.0f}, {0.0f, 1.0f}, {5.0f, 0.0f}};
    PLF f(pts);
    EXPECT_FLOAT_EQ(f.endX(), 5.0f);
}

// ─────────────────────────────────────────────
// Copy and move
// ─────────────────────────────────────────────

TEST(PiecewiseLinearFunction, CopyConstructor) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    PLF f(pts);
    PLF g(f);
    EXPECT_NEAR(g(0.5f), 0.5f, 1e-5f);
}

TEST(PiecewiseLinearFunction, CopyAssignment) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts1[] = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    PLF::Point pts2[] = {{0.0f, 1.0f}, {1.0f, 0.0f}};
    PLF f(pts1);
    PLF g(pts2);
    g = f;
    EXPECT_NEAR(g(0.5f), 0.5f, 1e-5f);
}

TEST(PiecewiseLinearFunction, MoveConstructor) {
    using PLF = bldc::PiecewiseLinearFunction<float, 2>;
    PLF::Point pts[] = {{0.0f, 0.0f}, {1.0f, 1.0f}};
    PLF f(pts);
    PLF g(std::move(f));
    EXPECT_NEAR(g(0.5f), 0.5f, 1e-5f);
}
