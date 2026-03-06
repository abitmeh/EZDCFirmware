#pragma once

#include <array>
#include <ranges>

namespace bldc {
    template <typename T, typename S>
    concept VectorSpace = requires (T a, T b, S s) {
        { a + b } -> std::convertible_to<T>;
        { a - b } -> std::convertible_to<T>;
        { a * s } -> std::convertible_to<T>;
        { s * a } -> std::convertible_to<T>;
        { a / s } -> std::convertible_to<T>;
    };

    template <typename T, typename S>
    concept InterpolableOver = VectorSpace<T, S> && requires(T a, T b) {
        { a / b } -> std::convertible_to<S>;
    };

    template <typename T, typename S = float> requires VectorSpace<T, S>
    T lerp(const T& a, const T& b, const S& t) {
        return T(a + t * (b - a));
    }

    template <typename T, typename S = float> requires InterpolableOver<T, S> 
    S invLerp(const T& a, const T& b, const T& v) {
        return S((v - a) / (b - a));
    }

    template <typename Y, size_t N, typename X = float> requires (InterpolableOver<X, float> && VectorSpace<Y, float> && std::totally_ordered<X>)
    class PiecewiseLinearFunction {
    public:
        struct Point {
            X x;
            Y y;
        };

        constexpr PiecewiseLinearFunction(const std::array<Point, N>& points);
        constexpr PiecewiseLinearFunction(const Point (&points)[N]);

        PiecewiseLinearFunction(const PiecewiseLinearFunction& other) = default;
        PiecewiseLinearFunction(PiecewiseLinearFunction&& other) = default;

        PiecewiseLinearFunction& operator=(const PiecewiseLinearFunction& other) = default;
        PiecewiseLinearFunction& operator=(PiecewiseLinearFunction&& other) = default;

        Y operator()(const X& x) const;

        constexpr X startX() const { return _points.front().x; }
        constexpr X endX() const { return _points.back().x; }
    private:

        std::array<Point, N> _points;
    };

    // IMPLEMENTATION

    template <typename Y, size_t N, typename X> requires (InterpolableOver<X, float> && VectorSpace<Y, float> && std::totally_ordered<X>)
    constexpr PiecewiseLinearFunction<Y, N, X>::PiecewiseLinearFunction(const std::array<bldc::PiecewiseLinearFunction<Y, N, X>::Point, N>& points) : _points(points) {
        for (const auto& [a, b] : std::ranges::views::adjacent<2>(_points)) {
            assert(a.x <= b.x);
        }
    }

    template <typename Y, size_t N, typename X> requires (InterpolableOver<X, float> && VectorSpace<Y, float> && std::totally_ordered<X>)
    constexpr PiecewiseLinearFunction<Y, N, X>::PiecewiseLinearFunction(const bldc::PiecewiseLinearFunction<Y, N, X>::Point (&points)[N]) : _points(std::to_array(points)) {
        for (const auto& [a, b] : std::ranges::views::adjacent<2>(_points)) {
            assert(a.x <= b.x);
        }
    }

    template <typename Y, size_t N, typename X> requires (InterpolableOver<X, float> && VectorSpace<Y, float> && std::totally_ordered<X>)
    Y PiecewiseLinearFunction<Y, N, X>::operator()(const X& x) const {
        if (x <= _points[0].x)  {
            return _points[0].y;
        } if (x >= _points.back().x) { 
            return _points.back().y;
        }
        
        for (const auto& [a, b] : std::ranges::views::adjacent<2>(_points)) {
            if (x >= a.x && x <= b.x) {
                const float frac = (x - a.x) / (b.x - a.x);
                return a.y + frac * (b.y - a.y);
            }
        }

        return _points.back().y;
    }
}
