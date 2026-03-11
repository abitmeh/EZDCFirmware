#pragma once

#include "Utilities/next_size.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>

namespace bldc {
    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    struct fixed;

    enum RawValue {
        Value
    };

    template <std::uintmax_t S, typename T = int16_t>
        requires std::integral<T>
    struct fixed {
    public:
        explicit constexpr fixed(float x);
        explicit constexpr fixed(double x);
        explicit constexpr fixed(long double x);
        template <typename U>
        explicit constexpr fixed(const U& x)
            requires std::integral<U>;
        constexpr fixed(T value, RawValue r);
        template <std::uintmax_t Q, typename R>
            requires std::is_convertible_v<R, T>
        explicit constexpr fixed(const fixed<Q, R>& other);

        explicit constexpr operator float() const;
        explicit constexpr operator double() const;
        explicit constexpr operator long double() const;
        template <typename U>
        explicit constexpr operator U() const
            requires std::integral<U>;

        const T& rawValue() const { return _value; };

        std::string to_string() const;

        constexpr const fixed<S, T>& operator+=(const fixed<S, T>& other);
        constexpr const fixed<S, T>& operator-=(const fixed<S, T>& other);
        constexpr const fixed<S, T>& operator*=(const fixed<S, T>& other)
            requires NextSizable<T>;
        constexpr const fixed<S, T>& operator/=(const fixed<S, T>& other)
            requires NextSizable<T>;

        constexpr const fixed<S, T>& operator<<=(size_t shift)
            requires(std::has_single_bit(S));
        constexpr const fixed<S, T>& operator>>=(size_t shift)
            requires(std::has_single_bit(S));

    private:
        T _value;

        friend constexpr std::strong_ordering operator<=>(const fixed<S, T>& lhs, const fixed<S, T>& rhs) { return lhs._value <=> rhs._value; }

        friend constexpr fixed<S, T> operator+(const fixed<S, T>& x) { return x; }

        friend constexpr fixed<S, T> operator-(const fixed<S, T>& x) { return fixed<S, T>(-x._value, RawValue::Value); }

        friend constexpr fixed<S, T> operator+(const fixed<S, T>& lhs, const fixed<S, T>& rhs) { return fixed<S, T>(lhs._value + rhs._value, RawValue::Value); }

        friend constexpr fixed<S, T> operator-(const fixed<S, T>& lhs, const fixed<S, T>& rhs) { return fixed<S, T>(lhs._value - rhs._value, RawValue::Value); }

        friend constexpr fixed<S, T> operator*(const fixed<S, T>& lhs, const fixed<S, T>& rhs)
            requires NextSizable<T>
        {
            return fixed<S, T>(
                static_cast<T>((static_cast<next_size_t<T>>(lhs._value) * static_cast<next_size_t<T>>(rhs._value)) / static_cast<next_size_t<T>>(S)),
                RawValue::Value);
        }

        friend constexpr fixed<S, T> operator/(const fixed<S, T>& lhs, const fixed<S, T>& rhs)
            requires NextSizable<T>
        {
            const T result =
                static_cast<T>((static_cast<next_size_t<T>>(lhs._value) * static_cast<next_size_t<T>>(S)) / static_cast<next_size_t<T>>(rhs._value));
            return fixed<S, T>(result, RawValue::Value);
        }

        friend constexpr fixed<S, T> operator<<(const fixed<S, T>& x, size_t shift)
            requires(std::has_single_bit(S))
        {
            return fixed<S, T>(x._value << shift, RawValue::Value);
        }

        friend constexpr fixed<S, T> operator>>(const fixed<S, T>& x, size_t shift)
            requires(std::has_single_bit(S))
        {
            return fixed<S, T>(x._value >> shift, RawValue::Value);
        }

        template <typename CharT, typename Traits>
        friend std::basic_ostream<CharT, Traits>& operator<<(std::basic_ostream<CharT, Traits>& os, const fixed<S, T>& x) {
            return os << x.to_string();
        }

        template <typename CharT, typename Traits>
        friend std::basic_istream<CharT, Traits>& operator>>(std::basic_istream<CharT, Traits>& is, fixed<S, T>& x) {
            std::string inputStr;
            is >> inputStr;
            size_t pos = 0;
            const float f = std::stof(inputStr, &pos);
            if (pos == 0) {
                is.setstate(std::ios::failbit);
            }

            x = fixed<S, T>(f);

            return is;
        }

        template <uintmax_t Q, typename R>
            requires std::integral<R>
        friend struct fixed;
    };

}  // namespace bldc

namespace std {
    template <std::uintmax_t S, typename T>
        requires integral<T>
    constexpr bldc::fixed<S, T> abs(const bldc::fixed<S, T>& x) {
        return bldc::fixed<S, T>(std::abs(x.rawValue()), bldc::RawValue::Value);
    }
}  // namespace std

// IMPLEMENTATION

namespace bldc {
    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr fixed<S, T>::fixed(float x) {
        _value = static_cast<T>(
            std::clamp(static_cast<float>(S) * x, static_cast<float>(std::numeric_limits<T>::min()), static_cast<float>(std::numeric_limits<T>::max())));
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr fixed<S, T>::fixed(double x) {
        _value = static_cast<T>(
            std::clamp(static_cast<double>(S) * x, static_cast<double>(std::numeric_limits<T>::min()), static_cast<double>(std::numeric_limits<T>::max())));
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr fixed<S, T>::fixed(long double x) {
        _value = static_cast<T>(std::clamp(static_cast<long double>(S) * x, static_cast<long double>(std::numeric_limits<T>::min()),
                                           static_cast<long double>(std::numeric_limits<T>::max())));
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    template <typename U>
    constexpr fixed<S, T>::fixed(const U& x)
        requires std::integral<U>
    {
        _value =
            static_cast<T>(std::clamp(static_cast<U>(S * x), static_cast<U>(std::numeric_limits<T>::min()), static_cast<U>(std::numeric_limits<T>::max())));
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr fixed<S, T>::fixed(T value, RawValue raw) {
        _value = value;
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    template <std::uintmax_t Q, typename R>
        requires std::is_convertible_v<R, T>
    constexpr fixed<S, T>::fixed(const fixed<Q, R>& other) {
        const intmax_t inter = static_cast<intmax_t>(other._value) * static_cast<intmax_t>(S);
        _value = static_cast<T>(inter / static_cast<intmax_t>(Q));
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr fixed<S, T>::operator float() const {
        return static_cast<float>(_value) / static_cast<float>(S);
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr fixed<S, T>::operator double() const {
        return static_cast<double>(_value) / static_cast<double>(S);
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr fixed<S, T>::operator long double() const {
        return static_cast<long double>(_value) / static_cast<long double>(S);
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    template <typename U>
    constexpr fixed<S, T>::operator U() const
        requires std::integral<U>
    {
        return static_cast<U>(_value) / static_cast<U>(S);
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    std::string fixed<S, T>::to_string() const {
        const float f = static_cast<float>(*this);
        const int8_t digits = static_cast<int8_t>(log10f(S));
        std::string result;
        std::stringstream stream;
        stream << std::setprecision(digits) << f;
        result = stream.str();
        return result;
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr const fixed<S, T>& fixed<S, T>::operator+=(const fixed<S, T>& other) {
        _value += other._value;
        return *this;
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr const fixed<S, T>& fixed<S, T>::operator-=(const fixed<S, T>& other) {
        _value -= other._value;
        return *this;
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr const fixed<S, T>& fixed<S, T>::operator*=(const fixed<S, T>& other)
        requires NextSizable<T>
    {
        _value = static_cast<T>((static_cast<next_size_t<T>>(_value) * other._value) / static_cast<next_size_t<T>>(S));
        return *this;
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr const fixed<S, T>& fixed<S, T>::operator/=(const fixed<S, T>& other)
        requires NextSizable<T>
    {
        _value = static_cast<T>((static_cast<next_size_t<T>>(_value) * static_cast<next_size_t<T>>(S)) / static_cast<next_size_t<T>>(other._value));
        return *this;
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr const fixed<S, T>& fixed<S, T>::operator<<=(size_t shift)
        requires(std::has_single_bit(S))
    {
        _value <<= shift;
        return *this;
    }

    template <std::uintmax_t S, typename T>
        requires std::integral<T>
    constexpr const fixed<S, T>& fixed<S, T>::operator>>=(size_t shift)
        requires(std::has_single_bit(S))
    {
        _value >>= shift;
        return *this;
    }
}  // namespace bldc

