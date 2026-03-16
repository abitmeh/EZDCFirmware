#pragma once

#include "Utilities/LinearPiecewiseLinearFunction.hpp"

#include <chrono>

using namespace std::chrono_literals;

namespace bldc {
    static constexpr bldc::PiecewiseLinearFunction<float, 4> kDragDutyCycleCurve{{
        {0.0f, 0.75f},
        {750.0f, 0.80f},
        {1'250.0f, 0.85f},
        {2'200.0f, 0.85f},
    }};

    static constexpr bldc::PiecewiseLinearFunction<float, 3> kDragRpmCurve{{
        {0.0f, 200.0f},
        {500.0f, 500.0f},
        {2'200.0f, 700.0f},
    }};
}  // namespace bldc
