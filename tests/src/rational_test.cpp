#include "Utilities/rational.hpp"

#include <gtest/gtest.h>

#include <sstream>

// ─────────────────────────────────────────────
// Convenience aliases
// ─────────────────────────────────────────────

using R16 = bldc::rational<int16_t>;
using R32 = bldc::rational<int32_t>;
using RU16 = bldc::rational<uint16_t>;
using RU32 = bldc::rational<uint32_t>;

// ─────────────────────────────────────────────
// Construction and reduction
// ─────────────────────────────────────────────

TEST(RationalConstruction, DefaultConstructor) {
    R32 r;
    EXPECT_EQ(r.numerator(), 0);
    EXPECT_EQ(r.denominator(), 1);
}

TEST(RationalConstruction, WholeNumber) {
    R32 r(3);
    EXPECT_EQ(r.numerator(), 3);
    EXPECT_EQ(r.denominator(), 1);
}

TEST(RationalConstruction, AlreadyReduced) {
    R32 r(1, 3);
    EXPECT_EQ(r.numerator(), 1);
    EXPECT_EQ(r.denominator(), 3);
}

TEST(RationalConstruction, ReducesOnConstruction) {
    R32 r(2, 4);
    EXPECT_EQ(r.numerator(), 1);
    EXPECT_EQ(r.denominator(), 2);
}

TEST(RationalConstruction, ReducesLargeGcd) {
    R32 r(100, 75);
    EXPECT_EQ(r.numerator(), 4);
    EXPECT_EQ(r.denominator(), 3);
}

TEST(RationalConstruction, NegativeNumerator) {
    R32 r(-1, 2);
    EXPECT_EQ(r.numerator(), -1);
    EXPECT_EQ(r.denominator(), 2);
}

TEST(RationalConstruction, NegativeDenominatorNormalized) {
    R32 r(1, -2);
    EXPECT_EQ(r.numerator(), -1);
    EXPECT_EQ(r.denominator(), 2);
}

TEST(RationalConstruction, BothNegativeNormalized) {
    R32 r(-1, -2);
    EXPECT_EQ(r.numerator(), 1);
    EXPECT_EQ(r.denominator(), 2);
}

TEST(RationalConstruction, ZeroNumerator) {
    R32 r(0, 5);
    EXPECT_EQ(r.numerator(), 0);
    EXPECT_EQ(r.denominator(), 1);
}

TEST(RationalConstruction, ConvertFromWider) {
    R32 r32(1, 3);
    R16 r16(r32);
    EXPECT_EQ(r16.numerator(), 1);
    EXPECT_EQ(r16.denominator(), 3);
}

TEST(RationalConstruction, Uint16) {
    RU16 r(3, 4);
    EXPECT_EQ(r.numerator(), 3);
    EXPECT_EQ(r.denominator(), 4);
}

TEST(RationalConstruction, Uint32) {
    RU32 r(6, 8);
    EXPECT_EQ(r.numerator(), 3);
    EXPECT_EQ(r.denominator(), 4);
}

// ─────────────────────────────────────────────
// Conversion operators
// ─────────────────────────────────────────────

TEST(RationalConversion, ToInt) {
    R32 r(7, 2);
    EXPECT_EQ(static_cast<int32_t>(r), 3);  // truncates
}

TEST(RationalConversion, ToFloat) {
    R32 r(1, 4);
    EXPECT_FLOAT_EQ(static_cast<float>(r), 0.25f);
}

TEST(RationalConversion, ToDouble) {
    R32 r(1, 3);
    EXPECT_NEAR(static_cast<double>(r), 0.3333, 0.0001);
}

TEST(RationalConversion, ToUint16) {
    RU16 r(3, 4);
    EXPECT_EQ(static_cast<uint16_t>(r), 0);  // truncates to 0
}

TEST(RationalConversion, ToUint32) {
    RU32 r(7, 2);
    EXPECT_EQ(static_cast<uint32_t>(r), 3);
}

// ─────────────────────────────────────────────
// Inverse
// ─────────────────────────────────────────────

TEST(RationalInverse, Inverse) {
    R32 r(2, 3);
    auto inv = r.inverse();
    EXPECT_EQ(inv.numerator(), 3);
    EXPECT_EQ(inv.denominator(), 2);
}

TEST(RationalInverse, InvertMutates) {
    R32 r(2, 3);
    r.invert();
    EXPECT_EQ(r.numerator(), 3);
    EXPECT_EQ(r.denominator(), 2);
}

TEST(RationalInverse, InverseOfWholeNumber) {
    R32 r(4);
    auto inv = r.inverse();
    EXPECT_EQ(inv.numerator(), 1);
    EXPECT_EQ(inv.denominator(), 4);
}

// ─────────────────────────────────────────────
// Addition
// ─────────────────────────────────────────────

TEST(RationalArithmetic, AddSameDenominator) {
    R32 a(1, 4), b(2, 4);
    auto result = a + b;
    EXPECT_EQ(result.numerator(), 3);
    EXPECT_EQ(result.denominator(), 4);
}

TEST(RationalArithmetic, AddDifferentDenominator) {
    R32 a(1, 3), b(1, 6);
    auto result = a + b;
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalArithmetic, AddInteger) {
    R32 a(1, 2);
    auto result = a + int32_t(1);
    EXPECT_EQ(result.numerator(), 3);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalArithmetic, IntegerPlusRational) {
    R32 a(1, 2);
    auto result = int32_t(1) + a;
    EXPECT_EQ(result.numerator(), 3);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalArithmetic, AddAssign) {
    R32 a(1, 4);
    a += R32(1, 4);
    EXPECT_EQ(a.numerator(), 1);
    EXPECT_EQ(a.denominator(), 2);
}

TEST(RationalArithmetic, AddAssignInteger) {
    R32 a(1, 2);
    a += int32_t(1);
    EXPECT_EQ(a.numerator(), 3);
    EXPECT_EQ(a.denominator(), 2);
}

TEST(RationalArithmetic, UnaryPlus) {
    R32 a(1, 2);
    auto result = +a;
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 2);
}

// ─────────────────────────────────────────────
// Subtraction
// ─────────────────────────────────────────────

TEST(RationalArithmetic, SubtractSameDenominator) {
    R32 a(3, 4), b(1, 4);
    auto result = a - b;
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalArithmetic, SubtractDifferentDenominator) {
    R32 a(1, 2), b(1, 3);
    auto result = a - b;
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 6);
}

TEST(RationalArithmetic, SubtractInteger) {
    R32 a(3, 2);
    auto result = a - int32_t(1);
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalArithmetic, SubtractAssign) {
    R32 a(3, 4);
    a -= R32(1, 4);
    EXPECT_EQ(a.numerator(), 1);
    EXPECT_EQ(a.denominator(), 2);
}

TEST(RationalArithmetic, UnaryNegate) {
    R32 a(1, 2);
    auto result = -a;
    EXPECT_EQ(result.numerator(), -1);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalArithmetic, UnaryNegateNegative) {
    R32 a(-1, 2);
    auto result = -a;
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 2);
}

// ─────────────────────────────────────────────
// Multiplication
// ─────────────────────────────────────────────

TEST(RationalArithmetic, MultiplyRationals) {
    R32 a(2, 3), b(3, 4);
    auto result = a * b;
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalArithmetic, MultiplyByInteger) {
    R32 a(1, 3);
    auto result = a * int32_t(3);
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 1);
}

TEST(RationalArithmetic, IntegerMultiplyRational) {
    R32 a(1, 3);
    auto result = int32_t(3) * a;
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 1);
}

TEST(RationalArithmetic, MultiplyAssign) {
    R32 a(2, 3);
    a *= R32(3, 4);
    EXPECT_EQ(a.numerator(), 1);
    EXPECT_EQ(a.denominator(), 2);
}

TEST(RationalArithmetic, MultiplyAssignInteger) {
    R32 a(1, 3);
    a *= int32_t(3);
    EXPECT_EQ(a.numerator(), 1);
    EXPECT_EQ(a.denominator(), 1);
}

TEST(RationalArithmetic, MultiplyByZero) {
    R32 a(2, 3);
    auto result = a * int32_t(0);
    EXPECT_EQ(result.numerator(), 0);
    EXPECT_EQ(result.denominator(), 1);
}

// ─────────────────────────────────────────────
// Division
// ─────────────────────────────────────────────

TEST(RationalArithmetic, DivideRationals) {
    R32 a(1, 2), b(1, 4);
    auto result = a / b;
    EXPECT_EQ(result.numerator(), 2);
    EXPECT_EQ(result.denominator(), 1);
}

TEST(RationalArithmetic, DivideByInteger) {
    R32 a(1, 2);
    auto result = a / int32_t(2);
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 4);
}

TEST(RationalArithmetic, IntegerDivideRational) {
    R32 a(1, 2);
    auto result = int32_t(2) / a;
    EXPECT_EQ(result.numerator(), 4);
    EXPECT_EQ(result.denominator(), 1);
}

TEST(RationalArithmetic, DivideAssign) {
    R32 a(1, 2);
    a /= R32(1, 4);
    EXPECT_EQ(a.numerator(), 2);
    EXPECT_EQ(a.denominator(), 1);
}

TEST(RationalArithmetic, DivideAssignInteger) {
    R32 a(1, 2);
    a /= int32_t(2);
    EXPECT_EQ(a.numerator(), 1);
    EXPECT_EQ(a.denominator(), 4);
}

// ─────────────────────────────────────────────
// Comparison
// ─────────────────────────────────────────────

TEST(RationalComparison, Equal) {
    R32 a(1, 2), b(2, 4);
    EXPECT_EQ(a <=> b, std::strong_ordering::equal);
}

TEST(RationalComparison, Less) {
    R32 a(1, 3), b(1, 2);
    EXPECT_EQ(a <=> b, std::strong_ordering::less);
}

TEST(RationalComparison, Greater) {
    R32 a(1, 2), b(1, 3);
    EXPECT_EQ(a <=> b, std::strong_ordering::greater);
}

TEST(RationalComparison, NegativeValues) {
    R32 a(-1, 2), b(-1, 3);
    EXPECT_EQ(a <=> b, std::strong_ordering::less);
}

TEST(RationalComparison, PositiveVsNegative) {
    R32 a(1, 2), b(-1, 2);
    EXPECT_EQ(a <=> b, std::strong_ordering::greater);
}

// ─────────────────────────────────────────────
// Stream I/O
// ─────────────────────────────────────────────

TEST(RationalStream, OutputFraction) {
    R32 r(1, 2);
    std::ostringstream oss;
    oss << r;
    EXPECT_EQ(oss.str(), "1/2");
}

TEST(RationalStream, OutputWholeNumber) {
    R32 r(3);
    std::ostringstream oss;
    oss << r;
    EXPECT_EQ(oss.str(), "3");
}

TEST(RationalStream, InputFraction) {
    R32 r;
    std::istringstream iss("1/2");
    iss >> r;
    EXPECT_EQ(r.numerator(), 1);
    EXPECT_EQ(r.denominator(), 2);
}

TEST(RationalStream, InputDecimal) {
    R32 r;
    std::istringstream iss("0.5");
    iss >> r;
    EXPECT_EQ(r.numerator(), 1);
    EXPECT_EQ(r.denominator(), 2);
}

TEST(RationalStream, InputWholeNumber) {
    R32 r;
    std::istringstream iss("3");
    iss >> r;
    EXPECT_EQ(r.numerator(), 3);
    EXPECT_EQ(r.denominator(), 1);
}

TEST(RationalStream, InputNegativeFraction) {
    R32 r;
    std::istringstream iss("-1/2");
    iss >> r;
    EXPECT_EQ(r.numerator(), -1);
    EXPECT_EQ(r.denominator(), 2);
}

TEST(RationalStream, InputInvalid) {
    R32 r;
    std::istringstream iss("abc");
    iss >> r;
    EXPECT_TRUE(iss.fail());
}

TEST(RationalStream, RoundtripFraction) {
    R32 a(3, 7);
    std::ostringstream oss;
    oss << a;
    R32 b;
    std::istringstream iss(oss.str());
    iss >> b;
    EXPECT_EQ(a <=> b, std::strong_ordering::equal);
}

// ─────────────────────────────────────────────
// std::abs and std::pow
// ─────────────────────────────────────────────

TEST(RationalMath, AbsPositive) {
    R32 r(1, 2);
    auto result = std::abs(r);
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalMath, AbsNegative) {
    R32 r(-1, 2);
    auto result = std::abs(r);
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 2);
}

TEST(RationalMath, PowInt) {
    R32 r(2, 3);
    auto result = std::pow(r, 2);
    EXPECT_EQ(result.numerator(), 4);
    EXPECT_EQ(result.denominator(), 9);
}

TEST(RationalMath, PowZero) {
    R32 r(2, 3);
    auto result = std::pow(r, 0);
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 1);
}

// ─────────────────────────────────────────────
// Structured bindings / tuple interface
// ─────────────────────────────────────────────

TEST(RationalTuple, GetNumerator) {
    R32 r(1, 2);
    EXPECT_EQ(get<0>(r), 1);
}

TEST(RationalTuple, GetDenominator) {
    R32 r(1, 2);
    EXPECT_EQ(get<1>(r), 2);
}

TEST(RationalTuple, GetMutableNumerator) {
    R32 r(1, 2);
    get<0>(r) = 3;
    EXPECT_EQ(r.numerator(), 3);
}

TEST(RationalTuple, GetMutableDenominator) {
    R32 r(1, 2);
    get<1>(r) = 4;
    EXPECT_EQ(r.denominator(), 4);
}

TEST(RationalTuple, StructuredBinding) {
    R32 r(3, 7);
    auto& [num, den] = r;
    EXPECT_EQ(num, 3);
    EXPECT_EQ(den, 7);
}

TEST(RationalTuple, StructuredBindingMutation) {
    R32 r(3, 7);
    auto& [num, den] = r;
    num = 5;
    EXPECT_EQ(r.numerator(), 5);
}

// ─────────────────────────────────────────────
// Arithmetic identities
// ─────────────────────────────────────────────

TEST(RationalIdentities, AddZero) {
    R32 a(1, 2), zero(0);
    auto result = a + zero;
    EXPECT_EQ(result <=> a, std::strong_ordering::equal);
}

TEST(RationalIdentities, MultiplyByOne) {
    R32 a(1, 2), one(1);
    auto result = a * one;
    EXPECT_EQ(result <=> a, std::strong_ordering::equal);
}

TEST(RationalIdentities, SubtractSelf) {
    R32 a(1, 2);
    auto result = a - a;
    EXPECT_EQ(result.numerator(), 0);
    EXPECT_EQ(result.denominator(), 1);
}

TEST(RationalIdentities, DivideBySelf) {
    R32 a(1, 2);
    auto result = a / a;
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 1);
}

TEST(RationalIdentities, MultiplyByInverse) {
    R32 a(2, 3);
    auto result = a * a.inverse();
    EXPECT_EQ(result.numerator(), 1);
    EXPECT_EQ(result.denominator(), 1);
}

TEST(RationalIdentities, AddSubtractRoundtrip) {
    R32 a(1, 3), b(1, 7);
    EXPECT_EQ(((a + b) - b) <=> a, std::strong_ordering::equal);
}

TEST(RationalIdentities, MultiplyDivideRoundtrip) {
    R32 a(2, 5), b(3, 7);
    EXPECT_EQ(((a * b) / b) <=> a, std::strong_ordering::equal);
}
