#include "Utilities/Angle.hpp"

#include <gtest/gtest.h>

#include <cmath>
#include <numbers>

using namespace bldc;
using namespace bldc::literals;

// ─────────────────────────────────────────────
// Convenience constants
// ─────────────────────────────────────────────

static constexpr float kPi = std::numbers::pi_v<float>;
static constexpr float kTol = 1.5f / 64.0f;    // one step of fixed<64,int8_t>
static constexpr float kTol2 = 1.5f / 128.0f;  // one step of fixed<128,uint8_t>

// ─────────────────────────────────────────────
// UDL construction
// ─────────────────────────────────────────────

TEST(Angle, UDLDegrees) {
    constexpr auto a = 0.0_deg;
    EXPECT_NEAR(a.degrees(), 0.0f, kTol);
}

TEST(Angle, UDLDegrees90) {
    constexpr auto a = 90.0_deg;
    EXPECT_NEAR(a.degrees(), 90.0f, 2.0f);  // 256 steps over 360 = 1.4 deg resolution
}

TEST(Angle, UDLDegrees180) {
    constexpr auto a = 180.0_deg;
    EXPECT_NEAR(a.degrees(), 180.0f, 2.0f);
}

TEST(Angle, UDLDegrees360WrapsToZero) {
    constexpr auto a = 360.0_deg;
    EXPECT_NEAR(a.degrees(), 0.0f, 2.0f);
}

TEST(Angle, UDLRadians) {
    constexpr auto a = 0.0_rad;
    EXPECT_NEAR(a.radians(), 0.0f, kTol);
}

TEST(Angle, UDLRadiansPiHalf) {
    constexpr auto a = Angle(kPi / 2.0f, AngleUnit::Radians);
    EXPECT_NEAR(a.radians(), kPi / 2.0f, 0.05f);
}

TEST(Angle, UDLRadiansPi) {
    constexpr auto a = Angle(kPi, AngleUnit::Radians);
    EXPECT_NEAR(a.radians(), kPi, 0.05f);
}

TEST(Angle, UDLRadiansTwoPiWraps) {
    constexpr auto a = Angle(2.0f * kPi, AngleUnit::Radians);
    EXPECT_NEAR(a.radians(), 0.0f, 0.05f);
}

// ─────────────────────────────────────────────
// Float constructor
// ─────────────────────────────────────────────

TEST(Angle, FloatConstructorDegrees) {
    Angle a(90.0f, AngleUnit::Degrees);
    EXPECT_NEAR(a.degrees(), 90.0f, 2.0f);
}

TEST(Angle, FloatConstructorRadians) {
    Angle a(kPi / 2.0f, AngleUnit::Radians);
    EXPECT_NEAR(a.radians(), kPi / 2.0f, 0.05f);
}

TEST(Angle, FloatConstructorDegrees360Wraps) {
    Angle a(360.0f, AngleUnit::Degrees);
    EXPECT_NEAR(a.degrees(), 0.0f, 2.0f);
}

TEST(Angle, FloatConstructorNegativeDegrees) {
    Angle a(-90.0f, AngleUnit::Degrees);
    EXPECT_NEAR(a.degrees(), 270.0f, 2.0f);
}

// ─────────────────────────────────────────────
// Accessors
// ─────────────────────────────────────────────

TEST(Angle, DegreesAndRadiansConsistent) {
    Angle a(45.0f, AngleUnit::Degrees);
    EXPECT_NEAR(a.degrees(), a.radians() * 180.0f / kPi, 2.0f);
}

TEST(Angle, InUnitDegreesMatchesDegrees) {
    Angle a(120.0f, AngleUnit::Degrees);
    EXPECT_NEAR(a.inUnit(AngleUnit::Degrees), a.degrees(), 1e-5f);
}

TEST(Angle, InUnitRadiansMatchesRadians) {
    Angle a(120.0f, AngleUnit::Degrees);
    EXPECT_NEAR(a.inUnit(AngleUnit::Radians), a.radians(), 1e-5f);
}

TEST(Angle, ProportionAroundCircle) {
    Angle a(180.0f, AngleUnit::Degrees);
    // 180 degrees = half way around, proportion should be ~0.5
    EXPECT_NEAR(static_cast<float>(a.proportionAroundCircle()), 0.5f, 1.0f / 128.0f);
}

// ─────────────────────────────────────────────
// Arithmetic
// ─────────────────────────────────────────────

TEST(Angle, Addition) {
    Angle a(90.0f, AngleUnit::Degrees);
    Angle b(90.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a + b).degrees(), 180.0f, 2.0f);
}

TEST(Angle, AdditionWraps) {
    Angle a(270.0f, AngleUnit::Degrees);
    Angle b(180.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a + b).degrees(), 90.0f, 2.0f);
}

TEST(Angle, Subtraction) {
    Angle a(180.0f, AngleUnit::Degrees);
    Angle b(90.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a - b).degrees(), 90.0f, 2.0f);
}

TEST(Angle, SubtractionWraps) {
    Angle a(0.0f, AngleUnit::Degrees);
    Angle b(90.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a - b).degrees(), 270.0f, 2.0f);
}

TEST(Angle, CompoundAddAssign) {
    Angle a(90.0f, AngleUnit::Degrees);
    a += Angle(90.0f, AngleUnit::Degrees);
    EXPECT_NEAR(a.degrees(), 180.0f, 2.0f);
}

TEST(Angle, CompoundSubAssign) {
    Angle a(180.0f, AngleUnit::Degrees);
    a -= Angle(90.0f, AngleUnit::Degrees);
    EXPECT_NEAR(a.degrees(), 90.0f, 2.0f);
}

TEST(Angle, MultiplyByFloat) {
    Angle a(90.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a * 2.0f).degrees(), 180.0f, 2.0f);
}

TEST(Angle, FloatMultiplyByAngle) {
    Angle a(90.0f, AngleUnit::Degrees);
    EXPECT_NEAR((2.0f * a).degrees(), 180.0f, 2.0f);
}

TEST(Angle, MultiplyByInt) {
    Angle a(45.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a * 4).degrees(), 180.0f, 2.0f);
}

TEST(Angle, MultiplyByFixed) {
    Angle a(90.0f, AngleUnit::Degrees);
    bldc::fixed<64, int8_t> half(0.5f);
    EXPECT_NEAR((a * half).degrees(), 45.0f, 2.0f);
}

TEST(Angle, MultiplyWraps) {
    Angle a(180.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a * 2.0f).degrees(), 0.0f, 2.0f);
}

TEST(Angle, DivideByFloat) {
    Angle a(180.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a / 2.0f).degrees(), 90.0f, 2.0f);
}

TEST(Angle, DivideByInt) {
    Angle a(180.0f, AngleUnit::Degrees);
    EXPECT_NEAR((a / 2).degrees(), 90.0f, 2.0f);
}

TEST(Angle, DivideByFixed) {
    Angle a(180.0f, AngleUnit::Degrees);
    bldc::fixed<64, int8_t> two(2.0f);
    EXPECT_NEAR((a / two).degrees(), 90.0f, 2.0f);
}

// ─────────────────────────────────────────────
// Comparison
// ─────────────────────────────────────────────

TEST(Angle, EqualAngles) {
    Angle a(90.0f, AngleUnit::Degrees);
    Angle b(90.0f, AngleUnit::Degrees);
    EXPECT_EQ(a <=> b, std::strong_ordering::equal);
}

TEST(Angle, LessThan) {
    Angle a(90.0f, AngleUnit::Degrees);
    Angle b(180.0f, AngleUnit::Degrees);
    EXPECT_EQ(a <=> b, std::strong_ordering::less);
}

TEST(Angle, GreaterThan) {
    Angle a(270.0f, AngleUnit::Degrees);
    Angle b(180.0f, AngleUnit::Degrees);
    EXPECT_EQ(a <=> b, std::strong_ordering::greater);
}

// ─────────────────────────────────────────────
// sin / cos
// ─────────────────────────────────────────────

TEST(Angle, SinZero) {
    EXPECT_NEAR(static_cast<float>(sin(0.0_deg)), 0.0f, kTol);
}

TEST(Angle, SinNinety) {
    EXPECT_NEAR(static_cast<float>(sin(90.0_deg)), 1.0f, kTol);
}

TEST(Angle, SinOneEighty) {
    EXPECT_NEAR(static_cast<float>(sin(180.0_deg)), 0.0f, kTol);
}

TEST(Angle, SinTwoSeventy) {
    EXPECT_NEAR(static_cast<float>(sin(270.0_deg)), -1.0f, kTol);
}

TEST(Angle, SinThreeSixty) {
    EXPECT_NEAR(static_cast<float>(sin(360.0_deg)), 0.0f, kTol);
}

TEST(Angle, SinFortyFive) {
    EXPECT_NEAR(static_cast<float>(sin(45.0_deg)), std::sin(kPi / 4.0f), kTol);
}

TEST(Angle, CosZero) {
    EXPECT_NEAR(static_cast<float>(cos(0.0_deg)), 1.0f, kTol);
}

TEST(Angle, CosNinety) {
    EXPECT_NEAR(static_cast<float>(cos(90.0_deg)), 0.0f, kTol);
}

TEST(Angle, CosOneEighty) {
    EXPECT_NEAR(static_cast<float>(cos(180.0_deg)), -1.0f, kTol);
}

TEST(Angle, CosTwoSeventy) {
    EXPECT_NEAR(static_cast<float>(cos(270.0_deg)), 0.0f, kTol);
}

TEST(Angle, SinCosPhaseRelationship) {
    // cos(x) == sin(x + 90)
    for (int i = 0; i < 256; i++) {
        Angle a(static_cast<float>(i * 360) / 256.0f, AngleUnit::Degrees);
        Angle b(static_cast<float>((i + 64) * 360) / 256.0f, AngleUnit::Degrees);
        EXPECT_NEAR(static_cast<float>(cos(a)), static_cast<float>(sin(b)), kTol) << "Failed at step " << i;
    }
}

// ─────────────────────────────────────────────
// sin2 / cos2
// ─────────────────────────────────────────────

TEST(Angle, Sin2Zero) {
    EXPECT_NEAR(static_cast<float>(sin2(0.0_deg)), 0.0f, kTol2);
}

TEST(Angle, Sin2Ninety) {
    EXPECT_NEAR(static_cast<float>(sin2(90.0_deg)), 1.0f, kTol2);
}

TEST(Angle, Sin2FortyFive) {
    EXPECT_NEAR(static_cast<float>(sin2(45.0_deg)), 0.5f, kTol2);
}

TEST(Angle, Sin2NonNegative) {
    for (int i = 0; i < 256; i++) {
        Angle a(static_cast<float>(i * 360) / 256.0f, AngleUnit::Degrees);
        EXPECT_GE(static_cast<float>(sin2(a)), 0.0f) << "Failed at step " << i;
    }
}

TEST(Angle, Cos2Zero) {
    EXPECT_NEAR(static_cast<float>(cos2(0.0_deg)), 1.0f, kTol2);
}

TEST(Angle, Cos2Ninety) {
    EXPECT_NEAR(static_cast<float>(cos2(90.0_deg)), 0.0f, kTol2);
}

TEST(Angle, Cos2NonNegative) {
    for (int i = 0; i < 256; i++) {
        Angle a(static_cast<float>(i * 360) / 256.0f, AngleUnit::Degrees);
        EXPECT_GE(static_cast<float>(cos2(a)), 0.0f) << "Failed at step " << i;
    }
}

TEST(Angle, PythagoreanIdentity) {
    // sin²(x) + cos²(x) == 1 for all angles
    // Both are fixed<128, uint8_t> so we can check raw storage values sum to 128
    for (int i = 0; i < 256; i++) {
        Angle a(static_cast<float>(i * 360) / 256.0f, AngleUnit::Degrees);
        const float s2 = static_cast<float>(sin2(a));
        const float c2 = static_cast<float>(cos2(a));
        EXPECT_NEAR(s2 + c2, 1.0f, kTol2 * 2.0f) << "Failed at step " << i;
    }
}

// ─────────────────────────────────────────────
// tan
// ─────────────────────────────────────────────

TEST(Angle, TanZero) {
    EXPECT_NEAR(static_cast<float>(tan(0.0_deg)), 0.0f, 1.5f / 64.0f);
}

TEST(Angle, TanFortyFive) {
    EXPECT_NEAR(static_cast<float>(tan(45.0_deg)), 1.0f, 1.5f / 64.0f);
}

TEST(Angle, TanNegFortyFive) {
    EXPECT_NEAR(static_cast<float>(tan(315.0_deg)), -1.0f, 1.5f / 64.0f);
}

TEST(Angle, TanOneEighty) {
    EXPECT_NEAR(static_cast<float>(tan(180.0_deg)), 0.0f, 1.5f / 64.0f);
}

TEST(Angle, TanSignInQ1Positive) {
    EXPECT_GT(static_cast<float>(tan(45.0_deg)), 0.0f);
}

TEST(Angle, TanSignInQ2Negative) {
    EXPECT_LT(static_cast<float>(tan(135.0_deg)), 0.0f);
}

TEST(Angle, TanSignInQ3Positive) {
    EXPECT_GT(static_cast<float>(tan(225.0_deg)), 0.0f);
}

TEST(Angle, TanSignInQ4Negative) {
    EXPECT_LT(static_cast<float>(tan(315.0_deg)), 0.0f);
}

TEST(Angle, TanConsistentWithSinOverCos) {
    // tan(x) == sin(x) / cos(x) for angles away from singularity
    const std::vector<float> testAngles = {0.0f, 30.0f, 45.0f, 60.0f, 120.0f, 135.0f, 150.0f, 180.0f, 210.0f, 225.0f, 240.0f, 300.0f, 315.0f, 330.0f};
    for (float deg : testAngles) {
        Angle a(deg, AngleUnit::Degrees);
        const float s = static_cast<float>(sin(a));
        const float c = static_cast<float>(cos(a));
        if (std::abs(c) > 0.1f) {  // avoid near-singularity
            EXPECT_NEAR(static_cast<float>(tan(a)), s / c, 0.1f) << "Failed at " << deg << " degrees";
        }
    }
}

// ─────────────────────────────────────────────
// asin / acos / atan
// ─────────────────────────────────────────────

TEST(Angle, AsinZero) {
    EXPECT_NEAR(asin(bldc::fixed<64, int8_t>(0.0f)).degrees(), 0.0f, 2.0f);
}

TEST(Angle, AsinOne) {
    EXPECT_NEAR(asin(bldc::fixed<64, int8_t>(1.0f)).degrees(), 90.0f, 2.0f);
}

TEST(Angle, AsinNegOne) {
    EXPECT_NEAR(asin(bldc::fixed<64, int8_t>(-1.0f)).degrees(), 270.0f, 2.0f);
}

TEST(Angle, AsinRoundtrip) {
    const std::vector<float> testAngles = {0.0f, 30.0f, 45.0f, 60.0f, 90.0f};
    for (float deg : testAngles) {
        Angle a(deg, AngleUnit::Degrees);
        EXPECT_NEAR(asin(sin(a)).degrees(), deg, 4.0f) << "Failed at " << deg;
    }
}

TEST(Angle, AcosOne) {
    EXPECT_NEAR(acos(bldc::fixed<64, int8_t>(1.0f)).degrees(), 0.0f, 2.0f);
}

TEST(Angle, AcosZero) {
    EXPECT_NEAR(acos(bldc::fixed<64, int8_t>(0.0f)).degrees(), 90.0f, 2.0f);
}

TEST(Angle, AcosNegOne) {
    EXPECT_NEAR(acos(bldc::fixed<64, int8_t>(-1.0f)).degrees(), 180.0f, 2.0f);
}

TEST(Angle, AcosRoundtrip) {
    const std::vector<float> testAngles = {0.0f, 30.0f, 45.0f, 60.0f, 90.0f, 120.0f, 150.0f, 180.0f};
    for (float deg : testAngles) {
        Angle a(deg, AngleUnit::Degrees);
        EXPECT_NEAR(acos(cos(a)).degrees(), deg, 4.0f) << "Failed at " << deg;
    }
}

TEST(Angle, AtanZero) {
    EXPECT_NEAR(atan(bldc::fixed<64, int16_t>(0.0f)).degrees(), 0.0f, 2.0f);
}

TEST(Angle, AtanOne) {
    EXPECT_NEAR(atan(bldc::fixed<64, int16_t>(1.0f)).degrees(), 45.0f, 2.0f);
}

TEST(Angle, AtanNegOne) {
    EXPECT_NEAR(atan(bldc::fixed<64, int16_t>(-1.0f)).degrees(), 315.0f, 2.0f);
}

TEST(Angle, AtanRoundtrip) {
    const std::vector<float> testAngles = {0.0f, 15.0f, 30.0f, 45.0f, 60.0f, 75.0f};
    for (float deg : testAngles) {
        Angle a(deg, AngleUnit::Degrees);
        EXPECT_NEAR(atan(tan(a)).degrees(), deg, 4.0f) << "Failed at " << deg;
    }
}
