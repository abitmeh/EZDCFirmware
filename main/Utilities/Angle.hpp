#pragma once

#include "Utilities/fixed.hpp"

#include <cassert>
#include <cmath>
#include <compare>
#include <cstdint>
#include <numbers>
#include <utility>

namespace bldc {
    enum class AngleUnit : uint8_t {
        Degrees,
        Radians
    };

    static constexpr float tau = 2 * std::numbers::pi_v<float>;

    struct Angle;

    constexpr Angle asin(fixed<64, int8_t> x);
    constexpr Angle asin2(fixed<128, uint8_t> x);
    constexpr Angle acos(fixed<64, int8_t> x);
    constexpr Angle acos2(fixed<128, uint8_t> x);
    Angle atan(fixed<64, int16_t> x);
    Angle atan2(fixed<128, uint16_t> x);

    template <typename T>
        requires std::floating_point<T>
    constexpr fixed<256, uint8_t> mkFixed(const T& x) {
        return fixed<256, uint8_t>(std::fmod(T(1.0) + std::fmod(x, T(1.0)), T(1.0)));
    }

    struct Angle {
    public:
        constexpr Angle(fixed<256, uint8_t> proportionAroundCircle) : _proportionOfCircle(proportionAroundCircle) {}

        constexpr Angle(float angle, AngleUnit unit) {
            switch (unit) {
                case AngleUnit::Degrees:
                    _proportionOfCircle = mkFixed(angle / 360.0f);
                    break;
                case AngleUnit::Radians:
                    _proportionOfCircle = mkFixed(angle / tau);
                    break;
                default:
                    std::unreachable();
                    break;
            }
        }

        constexpr fixed<256, uint8_t> proportionAroundCircle() { return _proportionOfCircle; }

        constexpr float degrees() const { return (static_cast<float>(_proportionOfCircle) * 360.0f); }

        constexpr float radians() const { return (static_cast<float>(_proportionOfCircle) * 2.0f * std::numbers::pi_v<float>); }

        constexpr float inUnit(AngleUnit unit) const {
            switch (unit) {
                case AngleUnit::Degrees:
                    return degrees();
                case AngleUnit::Radians:
                    return radians();
                default:
                    assert(false);
                    return 0;
            }
        }

        constexpr Angle& operator+=(Angle rhs) {
            _proportionOfCircle += rhs._proportionOfCircle;
            return *this;
        }

        constexpr Angle& operator-=(Angle rhs) {
            _proportionOfCircle -= rhs._proportionOfCircle;
            return *this;
        }

        template <uintmax_t S, typename T>
        constexpr Angle& operator*=(const fixed<S, T>& rhs) {
            _proportionOfCircle = static_cast<fixed<256, uint8_t>>(static_cast<fixed<S, T>>(_proportionOfCircle) * rhs);
            return *this;
        }

        template <uintmax_t S, typename T>
        constexpr Angle& operator/=(const fixed<S, T>& rhs) {
            _proportionOfCircle = static_cast<fixed<256, uint8_t>>(static_cast<fixed<S, T>>(_proportionOfCircle) / rhs);
            return *this;
        }

        template <typename T>
            requires(std::integral<T>)
        constexpr Angle& operator*=(const T& rhs) {
            _proportionOfCircle = fixed<256, uint8_t>(static_cast<uint8_t>(static_cast<T>(_proportionOfCircle.rawValue) * rhs), RawValue::Value);
            return *this;
        }

        template <typename T>
            requires(std::floating_point<T>)
        constexpr Angle& operator*=(const T& rhs) {
            _proportionOfCircle = static_cast<fixed<256, uint8_t>>(static_cast<T>(_proportionOfCircle) * rhs);
            return *this;
        }

        template <typename T>
            requires(std::integral<T>)
        constexpr Angle& operator/=(const T& rhs) {
            _proportionOfCircle = fixed<256, uint8_t>(static_cast<uint8_t>(static_cast<T>(_proportionOfCircle.rawValue()) / rhs), RawValue::Value);
            return *this;
        }

        template <typename T>
            requires(std::floating_point<T>)
        constexpr Angle& operator/=(const T& rhs) {
            _proportionOfCircle = static_cast<fixed<256, uint8_t>>(static_cast<T>(_proportionOfCircle) / rhs);
            return *this;
        }

        friend constexpr std::strong_ordering operator<=>(Angle lhs, Angle rhs) { return lhs._proportionOfCircle <=> rhs._proportionOfCircle; }

        friend constexpr Angle operator+(Angle lhs, Angle rhs) { return Angle(lhs._proportionOfCircle + rhs._proportionOfCircle); }

        friend constexpr Angle operator-(Angle lhs, Angle rhs) { return Angle(lhs._proportionOfCircle - rhs._proportionOfCircle); }

        template <uintmax_t S, typename T>
        friend constexpr Angle operator*(Angle lhs, const fixed<S, T>& rhs) {
            return Angle(static_cast<fixed<256, uint8_t>>(static_cast<fixed<S, T>>(lhs._proportionOfCircle) * rhs));
        }

        template <uintmax_t S, typename T>
        friend constexpr Angle operator*(const fixed<S, T>& lhs, Angle rhs) {
            return Angle(static_cast<fixed<256, uint8_t>>(lhs * static_cast<fixed<S, T>>(rhs._proportionOfCircle)));
        }

        template <typename T>
            requires(std::integral<T>)
        friend constexpr Angle operator*(Angle lhs, const T& rhs) {
            return Angle(fixed<256, uint8_t>(static_cast<uint8_t>(static_cast<T>(lhs._proportionOfCircle.rawValue()) * rhs), RawValue::Value));
        }

        template <typename T>
            requires(std::floating_point<T>)
        friend constexpr Angle operator*(Angle lhs, const T& rhs) {
            return Angle(mkFixed(static_cast<T>(lhs._proportionOfCircle) * rhs));
        }

        template <typename T>
            requires(std::integral<T>)
        friend constexpr Angle operator*(const T& lhs, Angle rhs) {
            return Angle(static_cast<fixed<256, uint8_t>>(lhs * static_cast<T>(rhs._proportionOfCircle.rawValue()), RawValue::Value));
        }

        template <typename T>
            requires(std::floating_point<T>)
        friend constexpr Angle operator*(const T& lhs, Angle rhs) {
            return Angle(mkFixed(lhs * static_cast<T>(rhs._proportionOfCircle)));
        }

        template <uintmax_t S, typename T>
        friend constexpr Angle operator/(Angle lhs, const fixed<S, T>& rhs) {
            return Angle(static_cast<fixed<256, uint8_t>>(static_cast<fixed<S, T>>(lhs._proportionOfCircle) / rhs));
        }

        template <typename T>
            requires(std::integral<T>)
        friend constexpr Angle operator/(Angle lhs, const T& rhs) {
            return Angle(fixed<256, uint8_t>(static_cast<uint8_t>(static_cast<T>(lhs._proportionOfCircle.rawValue()) / rhs), RawValue::Value));
        }

        template <typename T>
            requires(std::floating_point<T>)
        friend constexpr Angle operator/(Angle lhs, const T& rhs) {
            return Angle(mkFixed(static_cast<T>(lhs._proportionOfCircle) / rhs));
        }

        template <typename T>
        friend constexpr T tableLookupFunction(Angle angle, T* table, bool negateSecondHalf = false, bool negatedOddQuadrants = false) {
            const uint8_t rawAngle = angle._proportionOfCircle.rawValue();
            const uint8_t quadrant = rawAngle >> 6;             // top 2 bits determine quadrant
            const uint8_t tableIndex = (rawAngle >> 2) & 0x0F;  // next 4 bits determine table index

            const T pos = table[tableIndex];
            const T mirr = table[16 - tableIndex];

            switch (quadrant) {
                case 0:
                    return pos;
                case 1:
                    return (negatedOddQuadrants ? T(-1) : T(1)) * mirr;
                case 2:
                    return (negateSecondHalf ? T(-1) : T(1)) * pos;
                case 3:
                    return (negatedOddQuadrants ? T(-1) : T(1)) * (negateSecondHalf ? T(-1) : T(1)) * mirr;
                default:
                    std::unreachable();
                    return T(0);
            }

            std::unreachable();
            return T(0);
        }

        friend constexpr fixed<64, int8_t> sin(Angle angle) { return fixed<64, int8_t>(tableLookupFunction(angle, kSinTable, true)); }

        friend constexpr fixed<128, uint8_t> sin2(Angle angle) { return fixed<128, uint8_t>(tableLookupFunction(angle, kSin2Table)); }

        friend constexpr fixed<64, int8_t> cos(Angle angle) { return sin(angle + Angle(90, AngleUnit::Degrees)); }

        friend constexpr fixed<128, uint8_t> cos2(Angle angle) { return sin2(angle + Angle(90, AngleUnit::Degrees)); }

        friend constexpr fixed<64, int16_t> tan(Angle angle) { return fixed<64, int16_t>(tableLookupFunction(angle, kTanTable, false, true)); }

        friend constexpr fixed<128, uint16_t> tan2(Angle angle) { return fixed<128, uint16_t>(tableLookupFunction(angle, kTan2Table)); }

        friend constexpr Angle asin(fixed<64, int8_t> x) {
            static constexpr fixed<64, int8_t> minusOne(-1.0f);
            static constexpr fixed<64, int8_t> one(1.0f);
            assert(x >= minusOne && x <= one);
            const int8_t xRaw = std::clamp(x.rawValue(), int8_t(-64), int8_t(64));
            if (xRaw < 0) {
                return Angle(fixed<256, uint8_t>(static_cast<uint8_t>(0u - kASinTable[-xRaw]), RawValue::Value));
            }
            return Angle(fixed<256, uint8_t>(kASinTable[xRaw], RawValue::Value));
        }

        friend constexpr Angle acos(fixed<64, int8_t> x) {
            static constexpr fixed<64, int8_t> minusOne(-1.0f);
            static constexpr fixed<64, int8_t> one(1.0f);
            assert(x >= minusOne && x <= one);
            const Angle asined = asin(x);
            return Angle(90.0f, AngleUnit::Degrees) - asined;
        }

        friend Angle atan(fixed<64, int16_t> x) {
            const fixed<64, int16_t> absX = std::abs(x);

            uint8_t result;
            if (absX >= kTanTable[16]) {
                result = 64;  // saturate to 90°
            } else {
                uint8_t idx = 0;
                while (idx < 15 && kTanTable[idx + 1] <= absX) {
                    idx++;
                }
                const fixed<64, int16_t> span = kTanTable[idx + 1] - kTanTable[idx];
                const fixed<64, int16_t> frac = (absX - kTanTable[idx]) * static_cast<fixed<64, int16_t>>(4);
                result = idx * 4 + static_cast<uint8_t>((frac + span / static_cast<fixed<64, int16_t>>(2)) / span);
            }

            const uint8_t angle = x < fixed<64, int16_t>(0) ? static_cast<uint8_t>(256 - result) : result;
            return Angle(fixed<256, uint8_t>(angle, RawValue::Value));
        }

        friend Angle atan2(fixed<128, uint16_t> x) {
            uint8_t result;
            if (x >= kTan2Table[16]) {
                result = 64;  // saturate to 90°
            } else {
                uint8_t idx = 0;
                while (idx < 15 && kTan2Table[idx + 1] <= x) {
                    idx++;
                }
                const fixed<128, uint16_t> span = kTan2Table[idx + 1] - kTan2Table[idx];
                const fixed<128, uint16_t> frac = (x - kTan2Table[idx]) * fixed<128, uint16_t>(4);
                result = idx * 4 + static_cast<uint8_t>((frac + span / fixed<128, uint16_t>(2)) / span);
            }

            return Angle(fixed<256, uint8_t>(result, RawValue::Value));
        }

    private:
        bldc::fixed<256, uint8_t> _proportionOfCircle{0};

        static constexpr fixed<64, int8_t> kSinTable[17] = {
            fixed<64, int8_t>(0, RawValue::Value),  fixed<64, int8_t>(6, RawValue::Value),  fixed<64, int8_t>(12, RawValue::Value),
            fixed<64, int8_t>(19, RawValue::Value), fixed<64, int8_t>(24, RawValue::Value), fixed<64, int8_t>(30, RawValue::Value),
            fixed<64, int8_t>(36, RawValue::Value), fixed<64, int8_t>(41, RawValue::Value), fixed<64, int8_t>(45, RawValue::Value),
            fixed<64, int8_t>(49, RawValue::Value), fixed<64, int8_t>(53, RawValue::Value), fixed<64, int8_t>(56, RawValue::Value),
            fixed<64, int8_t>(59, RawValue::Value), fixed<64, int8_t>(61, RawValue::Value), fixed<64, int8_t>(63, RawValue::Value),
            fixed<64, int8_t>(64, RawValue::Value), fixed<64, int8_t>(64, RawValue::Value),
        };
        static constexpr fixed<128, uint8_t> kSin2Table[17] = {
            fixed<128, uint8_t>(0, RawValue::Value),   fixed<128, uint8_t>(1, RawValue::Value),   fixed<128, uint8_t>(5, RawValue::Value),
            fixed<128, uint8_t>(11, RawValue::Value),  fixed<128, uint8_t>(19, RawValue::Value),  fixed<128, uint8_t>(28, RawValue::Value),
            fixed<128, uint8_t>(40, RawValue::Value),  fixed<128, uint8_t>(52, RawValue::Value),  fixed<128, uint8_t>(64, RawValue::Value),
            fixed<128, uint8_t>(76, RawValue::Value),  fixed<128, uint8_t>(88, RawValue::Value),  fixed<128, uint8_t>(100, RawValue::Value),
            fixed<128, uint8_t>(109, RawValue::Value), fixed<128, uint8_t>(117, RawValue::Value), fixed<128, uint8_t>(123, RawValue::Value),
            fixed<128, uint8_t>(127, RawValue::Value), fixed<128, uint8_t>(128, RawValue::Value),
        };
        static constexpr fixed<64, int16_t> kTanTable[17] = {
            fixed<64, int16_t>(0, RawValue::Value),   fixed<64, int16_t>(6, RawValue::Value),     fixed<64, int16_t>(13, RawValue::Value),
            fixed<64, int16_t>(19, RawValue::Value),  fixed<64, int16_t>(27, RawValue::Value),    fixed<64, int16_t>(34, RawValue::Value),
            fixed<64, int16_t>(43, RawValue::Value),  fixed<64, int16_t>(53, RawValue::Value),    fixed<64, int16_t>(64, RawValue::Value),
            fixed<64, int16_t>(78, RawValue::Value),  fixed<64, int16_t>(96, RawValue::Value),    fixed<64, int16_t>(120, RawValue::Value),
            fixed<64, int16_t>(155, RawValue::Value), fixed<64, int16_t>(211, RawValue::Value),   fixed<64, int16_t>(322, RawValue::Value),
            fixed<64, int16_t>(650, RawValue::Value), fixed<64, int16_t>(32767, RawValue::Value),
        };
        static constexpr fixed<128, uint16_t> kTan2Table[17] = {
            fixed<128, uint16_t>(0, RawValue::Value),     fixed<128, uint16_t>(1, RawValue::Value),     fixed<128, uint16_t>(5, RawValue::Value),
            fixed<128, uint16_t>(12, RawValue::Value),    fixed<128, uint16_t>(22, RawValue::Value),    fixed<128, uint16_t>(37, RawValue::Value),
            fixed<128, uint16_t>(57, RawValue::Value),    fixed<128, uint16_t>(86, RawValue::Value),    fixed<128, uint16_t>(128, RawValue::Value),
            fixed<128, uint16_t>(190, RawValue::Value),   fixed<128, uint16_t>(287, RawValue::Value),   fixed<128, uint16_t>(448, RawValue::Value),
            fixed<128, uint16_t>(746, RawValue::Value),   fixed<128, uint16_t>(1391, RawValue::Value),  fixed<128, uint16_t>(3235, RawValue::Value),
            fixed<128, uint16_t>(13195, RawValue::Value), fixed<128, uint16_t>(65535, RawValue::Value),
        };
        static constexpr uint8_t kASinTable[65] = {0,  1,  1,  2,  3,  3,  4,  4,  5,  6,  6,  7,  8,  8,  9,  10, 10, 11, 12, 12, 13, 14,
                                                   14, 15, 16, 16, 17, 18, 18, 19, 20, 21, 21, 22, 23, 24, 24, 25, 26, 27, 28, 28, 29, 30,
                                                   31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 45, 46, 48, 50, 51, 54, 57, 64};
    };

    namespace literals {
        consteval Angle operator""_deg(long double x);
        consteval Angle operator""_rad(long double x);

        // IMPLEMENTATION

        consteval Angle operator""_deg(long double x) {
            return Angle(x, AngleUnit::Degrees);
        }

        consteval Angle operator""_rad(long double x) {
            return Angle(x, AngleUnit::Radians);
        }
    }  // namespace literals
}  // namespace bldc
