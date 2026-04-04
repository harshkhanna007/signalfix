// =============================================================================
// SignalFix AI — Common Utilities
// File   : include/signalfix/common/numeric_limits.hpp
// =============================================================================

#pragma once

#include <cstdint>

namespace signalfix::numeric_utils {

// Safe multiply-divide operation to reduce overflow risk.
// Computes (a * b) / c safely using quotient and remainder splitting.
constexpr uint64_t mul_div_u64(uint64_t a, uint64_t b, uint64_t c) noexcept
{
    const uint64_t q = a / c;
    const uint64_t r = a % c;

    return (q * b) + ((r * b) / c);
}

// Clamp value within bounds.
template <typename T>
constexpr T clamp(T value, T min_val, T max_val) noexcept
{
    return (value < min_val) ? min_val :
           (value > max_val) ? max_val :
           value;
}

} // namespace signalfix::numeric_utils