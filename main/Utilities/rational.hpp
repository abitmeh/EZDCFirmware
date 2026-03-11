#pragma once

#include "Utilities/next_size.hpp"

#include <algorithm>
#include <cassert>
#include <charconv>
#include <iostream>
#include <numeric>
#include <optional>
#include <string>
#include <type_traits>

namespace bldc {
    // All fractions are always stored in reduced form
    // For example, the rational 2/4 will never be stored, instead, if 2/4 is asked for, 1/2 will be stored.
    template <typename T>
        requires std::integral<T>
    class rational {
    public:
        constexpr rational() = default;
        constexpr rational(T numerator, T denominator = T(1));
        template <typename U>
        explicit constexpr rational(const rational<U>& other)
            requires std::is_convertible_v<U, T>;

        std::string toString() const;
        explicit constexpr operator T() const;
        explicit constexpr operator float() const
            requires std::is_convertible_v<T, float>;
        explicit constexpr operator double() const
            requires std::is_convertible_v<T, double>;
        explicit constexpr operator uint16_t() const
            requires std::is_convertible_v<T, uint16_t>;
        explicit constexpr operator uint32_t() const
            requires std::is_convertible_v<T, uint32_t>;

        template <std::size_t I>
        friend constexpr T& get(rational<T>& x) {
            if constexpr (I == 0) {
                return x._numerator;
            } else {
                return x._denominator;
            }
        }

        template <std::size_t I>
        friend constexpr const T& get(const rational<T>& x) {
            if constexpr (I == 0) {
                return x._numerator;
            } else {
                return x._denominator;
            }
        }

        template <std::size_t I>
        friend constexpr T&& get(rational<T>&& x) {
            if constexpr (I == 0) {
                return std::move(x._numerator);
            } else {
                return std::move(x._denominator);
            }
        }

        template <std::size_t I>
        friend constexpr const T&& get(const rational<T>&& x) {
            if constexpr (I == 0) {
                return std::move(x._numerator);
            } else {
                return std::move(x._denominator);
            }
        }

        constexpr T& numerator() { return _numerator; }

        constexpr const T& numerator() const { return _numerator; }

        constexpr T& denominator() { return _denominator; }

        constexpr const T& denominator() const { return _denominator; }

        constexpr rational<T> inverse() const { return rational<T>(_denominator, _numerator); }

        constexpr void invert() { std::swap(_numerator, _denominator); }

        constexpr rational<T>& operator+=(const rational<T>& other)
            requires NextSizable<T>;
        constexpr rational<T>& operator+=(const T& other)
            requires NextSizable<T>;
        constexpr rational<T>& operator-=(const rational<T>& other)
            requires NextSizable<T>;
        constexpr rational<T>& operator-=(const T& other)
            requires NextSizable<T>;
        constexpr rational<T>& operator*=(const rational<T>& other)
            requires NextSizable<T>;
        constexpr rational<T>& operator*=(const T& other)
            requires NextSizable<T>;
        constexpr rational<T>& operator/=(const rational<T>& other)
            requires NextSizable<T>;
        constexpr rational<T>& operator/=(const T& other)
            requires NextSizable<T>;

        friend std::ostream& operator<<(std::ostream& os, const rational<T>& r) { return os << r.toString(); }

        friend std::istream& operator>>(std::istream& is, rational<T>& r) {
            std::string inputStr;
            is >> inputStr;

            std::optional<rational<T>> parsed = rational<T>::_parse(inputStr);
            if (parsed.has_value()) {
                r = *parsed;
            } else {
                is.setstate(std::ios::failbit);
            }

            return is;
        }

    private:
        T _numerator{0};
        T _denominator{1};

        static std::optional<rational<T>> _parse(const std::string& str);

        constexpr static rational<T> _reduce(const rational<T>& r);
    };

    template <typename T>
    constexpr rational<T> operator+(const rational<T>& lhs, const rational<T>& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator+(const rational<T>& lhs, const T& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator+(const T& lhs, const rational<T>& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator-(const rational<T>& lhs, const rational<T>& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator-(const rational<T>& lhs, const T& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator-(const T& lhs, const rational<T>& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator*(const rational<T>& lhs, const rational<T>& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator*(const rational<T>& lhs, const T& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator*(const T& lhs, const rational<T>& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator/(const rational<T>& lhs, const rational<T>& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator/(const rational<T>& lhs, const T& rhs)
        requires NextSizable<T>;
    template <typename T>
    constexpr rational<T> operator/(const T& lhs, const rational<T>& rhs)
        requires NextSizable<T>;

    template <typename T>
    constexpr rational<T> operator+(const rational<T>& r);
    template <typename T>
    constexpr rational<T> operator-(const rational<T>& r);

    template <typename T>
    constexpr std::strong_ordering operator<=>(const rational<T>& lhs, const rational<T>& rhs);
}  // namespace bldc

namespace std {
    template <typename T>
    bldc::rational<T> abs(const bldc::rational<T>& r);

    template <typename T1, typename T2>
        requires(std::integral<T2> && std::is_unsigned_v<T2>)
    bldc::rational<std::common_type_t<T1, T2>> pow(const bldc::rational<T1>& x, const T2& y);
}  // namespace std

// IMPLEMENTATION

namespace bldc {
    template <typename T>
        requires std::integral<T>
    constexpr rational<T>::rational(T numerator, T denominator) : _numerator(numerator), _denominator(denominator) {
        assert(_denominator != T(0));
        *this = rational<T>::_reduce(*this);
    }

    template <typename T>
        requires std::integral<T>
    template <typename U>
    constexpr rational<T>::rational(const rational<U>& other)
        requires std::is_convertible_v<U, T>
        : _numerator(other.numerator()), _denominator(other.denominator()) {}

    template <typename T>
        requires std::integral<T>
    constexpr rational<T> rational<T>::_reduce(const rational<T>& r) {
        rational<T> result = r;

        const T gcd = std::gcd(r._numerator, r._denominator);
        result._numerator /= gcd;
        result._denominator /= gcd;

        if (result._numerator == 0) {
            result._denominator = 1;
            return result;
        }

        if (result._denominator < 0) {
            result._numerator = -result._numerator;
            result._denominator = -result._denominator;
        }

        return result;
    }

    template <typename T>
        requires std::integral<T>
    std::string rational<T>::toString() const {
        if (_denominator == 1) {
            return std::to_string(_numerator);
        }
        return std::to_string(_numerator) + "/" + std::to_string(_denominator);
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>::operator T() const {
        return _numerator / _denominator;
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>::operator float() const
        requires std::is_convertible_v<T, float>
    {
        return static_cast<float>(_numerator) / static_cast<float>(_denominator);
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>::operator double() const
        requires std::is_convertible_v<T, double>
    {
        return static_cast<double>(_numerator) / static_cast<double>(_denominator);
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>::operator uint16_t() const
        requires std::is_convertible_v<T, uint16_t>
    {
        return static_cast<uint16_t>(_numerator) / static_cast<uint16_t>(_denominator);
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>::operator uint32_t() const
        requires std::is_convertible_v<T, uint32_t>
    {
        return static_cast<uint32_t>(_numerator) / static_cast<uint32_t>(_denominator);
    }

    template <std::size_t I, typename T>
        requires std::integral<T>
    constexpr T&& get(rational<T>&& x) {
        if constexpr (I == 0) {
            return std::move(x.numerator());
        } else {
            return std::move(x.denominator());
        }
    }

    template <std::size_t I, typename T>
        requires std::integral<T>
    constexpr const T&& get(const rational<T>&& x) {
        if constexpr (I == 0) {
            return std::move(x.numerator());
        } else {
            return std::move(x.denominator());
        }
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>& rational<T>::operator+=(const rational<T>& other)
        requires NextSizable<T>
    {
        const next_size_t<T> numerator = static_cast<next_size_t<T>>(_numerator);
        const next_size_t<T> otherNumerator = static_cast<next_size_t<T>>(other._numerator);
        const next_size_t<T> denominator = static_cast<next_size_t<T>>(_denominator);
        const next_size_t<T> otherDenominator = static_cast<next_size_t<T>>(other._denominator);

        const next_size_t<T> newDenominator = std::lcm(denominator, otherDenominator);
        const next_size_t<T> newNumerator = numerator * (newDenominator / denominator) + otherNumerator * newDenominator / otherDenominator;

        _numerator = static_cast<T>(newNumerator);
        _denominator = static_cast<T>(newDenominator);

        *this = rational<T>::_reduce(*this);

        return *this;
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>& rational<T>::operator+=(const T& other)
        requires NextSizable<T>
    {
        _numerator = _numerator + _denominator * other;

        return *this;
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>& rational<T>::operator-=(const rational<T>& other)
        requires NextSizable<T>
    {
        *this += -other;
        return *this;
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>& rational<T>::operator-=(const T& other)
        requires NextSizable<T>
    {
        *this += -other;
        return *this;
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>& rational<T>::operator*=(const rational<T>& other)
        requires NextSizable<T>
    {
        const next_size_t<T> numerator = static_cast<next_size_t<T>>(_numerator);
        const next_size_t<T> otherNumerator = static_cast<next_size_t<T>>(other._numerator);
        const next_size_t<T> denominator = static_cast<next_size_t<T>>(_denominator);
        const next_size_t<T> otherDenominator = static_cast<next_size_t<T>>(other._denominator);

        const next_size_t<T> numeratorProduct = numerator * otherNumerator;
        const next_size_t<T> denominatorProduct = denominator * otherDenominator;

        const rational<next_size_t<T>> result(numeratorProduct, denominatorProduct);

        *this = static_cast<rational<T>>(result);

        return *this;
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>& rational<T>::operator*=(const T& other)
        requires NextSizable<T>
    {
        const T gcd = std::gcd(other, _denominator);
        const T newOther = other / gcd;
        _denominator /= gcd;
        _numerator *= newOther;

        // now _reduce(), except we already did the gcd bit more efficiently.
        if (_numerator == 0) {
            _denominator = 1;
            return *this;
        }

        if (_denominator < 0) {
            _numerator = -_numerator;
            _denominator = -_denominator;
        }

        return *this;
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>& rational<T>::operator/=(const rational<T>& other)
        requires NextSizable<T>
    {
        *this *= other.inverse();
        return *this;
    }

    template <typename T>
        requires std::integral<T>
    constexpr rational<T>& rational<T>::operator/=(const T& other)
        requires NextSizable<T>
    {
        *this *= rational<T>(T(1), other);
        return *this;
    }

    template <typename T>
        requires std::integral<T>
    std::optional<rational<T>> rational<T>::_parse(const std::string& str) {
        enum class ParsingState {
            CheckingSign,
            CheckingInteger,
            CheckingFractional,
            CheckingDenominator
        };

        std::string spaceRemoved = str;
        spaceRemoved.erase(std::remove(spaceRemoved.begin(), spaceRemoved.end(), ' '), spaceRemoved.end());

        ParsingState state = ParsingState::CheckingSign;

        bool positive = true;
        size_t fractionalLength = 0;
        std::string numeratorString = "";
        std::string parsed = "";
        T numerator{0};
        T denominator{1};

        for (char c : spaceRemoved) {
            switch (state) {
                case ParsingState::CheckingSign:
                    if (c == '-') {
                        positive = false;
                        state = ParsingState::CheckingInteger;
                    } else if (c == '+') {
                        state = ParsingState::CheckingInteger;
                    } else if (isdigit(c)) {
                        parsed += c;
                        state = ParsingState::CheckingInteger;
                    } else if (c == '.') {
                        parsed = "0";
                        state = ParsingState::CheckingFractional;
                    } else {
                        return std::nullopt;
                    }
                    break;
                case ParsingState::CheckingInteger:
                    if (isdigit(c)) {
                        parsed += c;
                    } else if (c == '.') {
                        state = ParsingState::CheckingFractional;
                    } else if (c == '/') {
                        auto [ptr, err] = std::from_chars(parsed.data(), parsed.data() + parsed.size(), numerator);
                        if (err != std::errc()) {
                            return std::nullopt;
                        }
                        parsed = "";
                        state = ParsingState::CheckingDenominator;
                    } else {
                        return std::nullopt;
                    }
                    break;
                case ParsingState::CheckingFractional:
                    if (!isdigit(c)) {
                        return std::nullopt;
                    }
                    parsed += c;
                    fractionalLength++;
                    break;
                case ParsingState::CheckingDenominator:
                    if (parsed.empty() && c == '+') {
                        break;
                    }
                    if (parsed.empty() && c == '-') {
                        positive = !positive;
                        break;
                    }
                    if (!isdigit(c)) {
                        return std::nullopt;
                    }
                    parsed += c;
                    break;
            }
        }

        switch (state) {
            case ParsingState::CheckingSign:
                return std::nullopt;
            case ParsingState::CheckingInteger: {
                const auto [ptr, err] = std::from_chars(parsed.data(), parsed.data() + parsed.size(), numerator);
                if (err != std::errc()) {
                    return std::nullopt;
                }
                denominator = 1;
                break;
            }
            case ParsingState::CheckingFractional: {
                auto [ptr, err] = std::from_chars(parsed.data(), parsed.data() + parsed.size(), numerator);
                if (err != std::errc()) {
                    return std::nullopt;
                }
                denominator = 1;
                for (size_t i = 0; i < fractionalLength; i++) {
                    denominator *= 10;
                }
                break;
            }
            case ParsingState::CheckingDenominator: {
                const auto [ptr, err] = std::from_chars(parsed.data(), parsed.data() + parsed.size(), denominator);
                if (err != std::errc()) {
                    return std::nullopt;
                }

                break;
            }
        }

        if (!positive) {
            numerator = -numerator;
        }

        return rational<T>(numerator, denominator);
    }

    template <typename T>
    constexpr rational<T> operator+(const rational<T>& lhs, const rational<T>& rhs)
        requires NextSizable<T>
    {
        const next_size_t<T> lhsNumerator = static_cast<next_size_t<T>>(lhs.numerator());
        const next_size_t<T> rhsNumerator = static_cast<next_size_t<T>>(rhs.numerator());
        const next_size_t<T> lhsDenominator = static_cast<next_size_t<T>>(lhs.denominator());
        const next_size_t<T> rhsDenominator = static_cast<next_size_t<T>>(rhs.denominator());

        const next_size_t<T> newDenominator = std::lcm(lhsDenominator, rhsDenominator);
        const next_size_t<T> newNumerator = lhsNumerator * (newDenominator / lhsDenominator) + rhsNumerator * (newDenominator / rhsDenominator);

        return rational<T>(static_cast<T>(newNumerator), static_cast<T>(newDenominator));
    }

    template <typename T>
    constexpr rational<T> operator+(const rational<T>& lhs, const T& rhs)
        requires NextSizable<T>
    {
        return rational<T>(lhs.numerator() + lhs.denominator() * rhs, lhs.denominator());
    }

    template <typename T>
    constexpr rational<T> operator+(const T& lhs, const rational<T>& rhs)
        requires NextSizable<T>
    {
        return rational<T>(rhs.numerator() + rhs.denominator() * lhs, rhs.denominator());
    }

    template <typename T>
    constexpr rational<T> operator-(const rational<T>& lhs, const rational<T>& rhs)
        requires NextSizable<T>
    {
        return lhs + (-rhs);
    }

    template <typename T>
    constexpr rational<T> operator-(const rational<T>& lhs, const T& rhs)
        requires NextSizable<T>
    {
        return lhs + (-rhs);
    }

    template <typename T>
    constexpr rational<T> operator-(const T& lhs, const rational<T>& rhs)
        requires NextSizable<T>
    {
        return lhs + (-rhs);
    }

    template <typename T>
    constexpr rational<T> operator*(const rational<T>& lhs, const rational<T>& rhs)
        requires NextSizable<T>
    {
        const next_size_t<T> lhsNumerator = static_cast<next_size_t<T>>(lhs.numerator());
        const next_size_t<T> lhsDenominator = static_cast<next_size_t<T>>(lhs.denominator());
        const next_size_t<T> rhsNumerator = static_cast<next_size_t<T>>(rhs.numerator());
        const next_size_t<T> rhsDenominator = static_cast<next_size_t<T>>(rhs.denominator());

        const next_size_t<T> numeratorProduct = lhsNumerator * rhsNumerator;
        const next_size_t<T> denominatorProduct = lhsDenominator * rhsDenominator;
        const next_size_t<T> gcd = std::gcd(numeratorProduct, denominatorProduct);

        const T newNumerator(numeratorProduct / gcd);
        const T newDenominator(denominatorProduct / gcd);

        return rational<T>(newNumerator, newDenominator);
    }

    template <typename T>
    constexpr rational<T> operator*(const rational<T>& lhs, const T& rhs)
        requires NextSizable<T>
    {
        const T gcd = std::gcd(rhs, lhs.denominator());
        const T newRhs = rhs / gcd;
        const T denominator = lhs.denominator() / gcd;
        const T numerator = lhs.numerator() * newRhs;

        // now _reduce(), except we already did the gcd bit more efficiently.
        if (numerator == 0) {
            return rational<T>(0, 1);
        }

        return denominator < 0 ? rational<T>(-numerator, -denominator) : rational<T>(numerator, denominator);
    }

    template <typename T>
    constexpr rational<T> operator*(const T& lhs, const rational<T>& rhs)
        requires NextSizable<T>
    {
        const T gcd = std::gcd(rhs.denominator(), lhs);
        const T newLhs = lhs / gcd;
        const T denominator = rhs.denominator() / gcd;
        const T numerator = rhs.numerator() * newLhs;

        // now _reduce(), except we already did the gcd bit more efficiently.
        if (numerator == 0) {
            return rational<T>(0, 1);
        }

        return denominator < 0 ? rational<T>(-numerator, -denominator) : rational<T>(numerator, denominator);
    }

    template <typename T>
    constexpr rational<T> operator/(const rational<T>& lhs, const rational<T>& rhs)
        requires NextSizable<T>
    {
        return lhs * rhs.inverse();
    }

    template <typename T>
    constexpr rational<T> operator/(const rational<T>& lhs, const T& rhs)
        requires NextSizable<T>
    {
        return lhs * rational<T>(1, rhs);
    }

    template <typename T>
    constexpr rational<T> operator/(const T& lhs, const rational<T>& rhs)
        requires NextSizable<T>
    {
        return lhs * rhs.inverse();
    }

    template <typename T>
    constexpr rational<T> operator+(const rational<T>& r) {
        return r;
    }

    template <typename T>
    constexpr rational<T> operator-(const rational<T>& r) {
        return rational<T>(-r.numerator(), r.denominator());
    }

    template <typename T>
    constexpr std::strong_ordering operator<=>(const rational<T>& lhs, const rational<T>& rhs) {
        if (lhs.numerator() == rhs.numerator() && lhs.denominator() == rhs.denominator()) {
            return std::strong_ordering::equal;
        }

        const next_size_t<T> lhsMultiplied = static_cast<next_size_t<T>>(lhs.numerator()) * static_cast<next_size_t<T>>(rhs.denominator());
        const next_size_t<T> rhsMultiplied = static_cast<next_size_t<T>>(rhs.numerator()) * static_cast<next_size_t<T>>(lhs.denominator());

        return lhsMultiplied < rhsMultiplied ? std::strong_ordering::less : std::strong_ordering::greater;
    }
}  // namespace bldc

namespace std {
    template <typename T>
    bldc::rational<T> abs(const bldc::rational<T>& r) {
        return bldc::rational<T>(std::abs(r.numerator()), std::abs(r.denominator()));
    }

    template <typename T1, typename T2>
    bldc::rational<std::common_type_t<T1, T2>> pow(const bldc::rational<T1>& x, const T2& y) {
        using T3 = std::common_type_t<T1, T2>;
        if (y == 0) {
            return T3(1);
        }

        const T3 numerator = static_cast<T3>(x.numerator());
        const T3 denominator = static_cast<T3>(x.denominator());
        T3 resultNumerator = numerator;
        T3 resultDenominator = denominator;
        for (T2 i = T2(1); i < y; ++i) {
            resultNumerator *= numerator;
            resultDenominator *= denominator;
        }

        return bldc::rational<T3>(resultNumerator, resultDenominator);
    }

    template <typename T>
    struct tuple_size<bldc::rational<T>> {
        static constexpr size_t value = 2;
    };

    template <size_t I, typename T>
    struct tuple_element<I, bldc::rational<T>> {
        using type = T;
    };
}  // namespace std

