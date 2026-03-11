#pragma once

#include <cstdint>

namespace bldc {
    template <typename T>
    struct next_size;
    template <typename T>
    using next_size_t = typename next_size<T>::type;

    template <typename T>
    struct tag {
        using type = T;
    };

    template <>
    struct next_size<uint8_t> : tag<uint16_t> {};

    template <>
    struct next_size<uint16_t> : tag<uint32_t> {};

    template <>
    struct next_size<uint32_t> : tag<uint64_t> {};

    template <>
    struct next_size<int8_t> : tag<int16_t> {};

    template <>
    struct next_size<int16_t> : tag<int32_t> {};

    template <>
    struct next_size<int32_t> : tag<int64_t> {};

    template <typename T>
    concept NextSizable = requires(next_size_t<T> a) { a; };
}  // namespace bldc
