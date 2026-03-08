#pragma once

#include "Utilities/LinearPiecewiseLinearFunction.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace bldc {
    static constexpr bldc::PiecewiseLinearFunction<float, 4> kDragDutyCycleCurve{{
        {0.0f, 0.55f},
        {750.0f, 0.65f},
        {1'250.0f, 0.68f},
        {11'000.0f, 0.70f},
    }};

    static constexpr bldc::PiecewiseLinearFunction<float, 5> kDragRpmCurve{{
        {0.0f, 800.0f},
        {500.0f, 1600.0f},
        {2'000.0f, 1900.0f},
        {3'000.0f, 2000.0f},
        {11'000.0f, 2000.0f},
    }};
}  // namespace bldc
