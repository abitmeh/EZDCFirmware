#include "Utilities/fixed.hpp"

#include <gtest/gtest.h>

#include <sstream>

// Convenience aliases used throughout
using Q16 = bldc::fixed<100, int16_t>;   // 2 decimal places, range ~[-327, 327]
using Q32 = bldc::fixed<100, int32_t>;   // 2 decimal places, wider range
using Q16p = bldc::fixed<256, int16_t>;  // power-of-two scale, for shift tests

// ─────────────────────────────────────────────
// Construction from floating point
// ─────────────────────────────────────────────

TEST(FixedConstruction, FromFloat) {
    Q16 a(1.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 1.0f);
}

TEST(FixedConstruction, FromFloatFraction) {
    Q16 a(0.5f);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 0.5f);
}

TEST(FixedConstruction, FromDouble) {
    Q16 a(1.0);
    EXPECT_DOUBLE_EQ(static_cast<double>(a), 1.0);
}

TEST(FixedConstruction, FromLongDouble) {
    Q16 a(1.0L);
    EXPECT_DOUBLE_EQ(static_cast<double>(a), 1.0);
}

TEST(FixedConstruction, FromNegativeFloat) {
    Q16 a(-1.5f);
    EXPECT_FLOAT_EQ(static_cast<float>(a), -1.5f);
}

TEST(FixedConstruction, FromZero) {
    Q16 a(0.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 0.0f);
}

TEST(FixedConstruction, ClampPositiveOverflow) {
    // int16_t max raw is 32767, so max representable is 327.67
    Q16 a(10000.0f);
    EXPECT_LE(static_cast<float>(a), 327.68f);
}

TEST(FixedConstruction, ClampNegativeOverflow) {
    Q16 a(-10000.0f);
    EXPECT_GE(static_cast<float>(a), -327.69f);
}

// ─────────────────────────────────────────────
// Construction from integer
// ─────────────────────────────────────────────

TEST(FixedConstruction, FromInt) {
    Q16 a(5);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 5.0f);
}

TEST(FixedConstruction, FromNegativeInt) {
    Q16 a(-3);
    EXPECT_FLOAT_EQ(static_cast<float>(a), -3.0f);
}

TEST(FixedConstruction, FromIntZero) {
    Q16 a(0);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 0.0f);
}

// ─────────────────────────────────────────────
// Conversion operators
// ─────────────────────────────────────────────

TEST(FixedConversion, ToFloat) {
    Q16 a(2.25f);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 2.25f);
}

TEST(FixedConversion, ToDouble) {
    Q16 a(2.25f);
    EXPECT_NEAR(static_cast<double>(a), 2.25, 0.01);
}

TEST(FixedConversion, ToInt) {
    Q16 a(3.7f);
    EXPECT_EQ(static_cast<int>(a), 3);  // truncates toward zero
}

TEST(FixedConversion, ToIntNegative) {
    Q16 a(-3.7f);
    EXPECT_EQ(static_cast<int>(a), -3);
}

// ─────────────────────────────────────────────
// Addition
// ─────────────────────────────────────────────

TEST(FixedArithmetic, Add) {
    Q16 a(1.5f), b(2.25f);
    EXPECT_FLOAT_EQ(static_cast<float>(a + b), 3.75f);
}

TEST(FixedArithmetic, AddNegative) {
    Q16 a(1.5f), b(-0.5f);
    EXPECT_FLOAT_EQ(static_cast<float>(a + b), 1.0f);
}

TEST(FixedArithmetic, AddAssign) {
    Q16 a(1.0f);
    a += Q16(2.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 3.0f);
}

TEST(FixedArithmetic, UnaryPlus) {
    Q16 a(1.5f);
    EXPECT_FLOAT_EQ(static_cast<float>(+a), 1.5f);
}

// ─────────────────────────────────────────────
// Subtraction
// ─────────────────────────────────────────────

TEST(FixedArithmetic, Subtract) {
    Q16 a(3.0f), b(1.25f);
    EXPECT_FLOAT_EQ(static_cast<float>(a - b), 1.75f);
}

TEST(FixedArithmetic, SubtractAssign) {
    Q16 a(3.0f);
    a -= Q16(1.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 2.0f);
}

TEST(FixedArithmetic, UnaryNegate) {
    Q16 a(1.5f);
    EXPECT_FLOAT_EQ(static_cast<float>(-a), -1.5f);
}

TEST(FixedArithmetic, UnaryNegateNegative) {
    Q16 a(-1.5f);
    EXPECT_FLOAT_EQ(static_cast<float>(-a), 1.5f);
}

// ─────────────────────────────────────────────
// Multiplication
// ─────────────────────────────────────────────

TEST(FixedArithmetic, Multiply) {
    Q16 a(2.0f), b(3.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(a * b), 6.0f);
}

TEST(FixedArithmetic, MultiplyFraction) {
    Q16 a(2.0f), b(0.5f);
    EXPECT_NEAR(static_cast<float>(a * b), 1.0f, 0.02f);
}

TEST(FixedArithmetic, MultiplyNegative) {
    Q16 a(2.0f), b(-3.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(a * b), -6.0f);
}

TEST(FixedArithmetic, MultiplyAssign) {
    Q16 a(2.0f);
    a *= Q16(3.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(a), 6.0f);
}

// ─────────────────────────────────────────────
// Division
// ─────────────────────────────────────────────

TEST(FixedArithmetic, Divide) {
    Q16 a(6.0f), b(2.0f);
    EXPECT_NEAR(static_cast<float>(a / b), 3.0f, 0.02f);
}

TEST(FixedArithmetic, DivideFraction) {
    Q16 a(1.0f), b(4.0f);
    EXPECT_NEAR(static_cast<float>(a / b), 0.25f, 0.02f);
}

TEST(FixedArithmetic, DivideNegative) {
    Q16 a(6.0f), b(-2.0f);
    EXPECT_NEAR(static_cast<float>(a / b), -3.0f, 0.02f);
}

TEST(FixedArithmetic, DivideAssign) {
    Q16 a(6.0f);
    a /= Q16(2.0f);
    EXPECT_NEAR(static_cast<float>(a), 3.0f, 0.02f);
}

// ─────────────────────────────────────────────
// Comparison
// ─────────────────────────────────────────────

TEST(FixedComparison, Equal) {
    Q16 a(1.0f), b(1.0f);
    EXPECT_EQ(a <=> b, std::strong_ordering::equal);
}

TEST(FixedComparison, Less) {
    Q16 a(1.0f), b(2.0f);
    EXPECT_EQ(a <=> b, std::strong_ordering::less);
}

TEST(FixedComparison, Greater) {
    Q16 a(2.0f), b(1.0f);
    EXPECT_EQ(a <=> b, std::strong_ordering::greater);
}

TEST(FixedComparison, NegativeValues) {
    Q16 a(-1.0f), b(-2.0f);
    EXPECT_EQ(a <=> b, std::strong_ordering::greater);
}

// ─────────────────────────────────────────────
// Bit shifts (power-of-two scale only)
// ─────────────────────────────────────────────

TEST(FixedShift, ShiftLeft) {
    Q16p a(1.0f);
    EXPECT_NEAR(static_cast<float>(a << 1), 2.0f, 0.02f);
}

TEST(FixedShift, ShiftRight) {
    Q16p a(2.0f);
    EXPECT_NEAR(static_cast<float>(a >> 1), 1.0f, 0.02f);
}

TEST(FixedShift, ShiftLeftAssign) {
    Q16p a(1.0f);
    a <<= 1;
    EXPECT_NEAR(static_cast<float>(a), 2.0f, 0.02f);
}

TEST(FixedShift, ShiftRightAssign) {
    Q16p a(2.0f);
    a >>= 1;
    EXPECT_NEAR(static_cast<float>(a), 1.0f, 0.02f);
}

// ─────────────────────────────────────────────
// Stream I/O
// ─────────────────────────────────────────────

TEST(FixedStream, Output) {
    Q16 a(1.5f);
    std::ostringstream oss;
    oss << a;
    EXPECT_FALSE(oss.str().empty());
}

TEST(FixedStream, InputOutput) {
    Q16 a(2.5f);
    std::ostringstream oss;
    oss << a;

    Q16 b(0.0f);
    std::istringstream iss(oss.str());
    iss >> b;

    EXPECT_NEAR(static_cast<float>(b), 2.5f, 0.02f);
}

// ─────────────────────────────────────────────
// Arithmetic identities
// ─────────────────────────────────────────────

TEST(FixedIdentities, AddZero) {
    Q16 a(1.5f), zero(0.0f);
    EXPECT_FLOAT_EQ(static_cast<float>(a + zero), static_cast<float>(a));
}

TEST(FixedIdentities, MultiplyByOne) {
    Q16 a(1.5f), one(1.0f);
    EXPECT_NEAR(static_cast<float>(a * one), static_cast<float>(a), 0.02f);
}

TEST(FixedIdentities, SubtractSelf) {
    Q16 a(1.5f);
    EXPECT_FLOAT_EQ(static_cast<float>(a - a), 0.0f);
}

TEST(FixedIdentities, DivideBySelf) {
    Q16 a(1.5f);
    EXPECT_NEAR(static_cast<float>(a / a), 1.0f, 0.02f);
}

TEST(FixedIdentities, AddSubtractRoundtrip) {
    Q16 a(1.5f), b(0.75f);
    EXPECT_FLOAT_EQ(static_cast<float>((a + b) - b), static_cast<float>(a));
}

TEST(FixedIdentities, MultiplyDivideRoundtrip) {
    Q32 a(3.0f), b(4.0f);
    EXPECT_NEAR(static_cast<float>((a * b) / b), static_cast<float>(a), 0.05f);
}
