// =============================================================================
// SignalFix AI — Common Utilities
// File   : include/signalfix/common/time_utils.hpp
// =============================================================================

#pragma once

#include <cstdint>

namespace signalfix::time_utils {

// Safe absolute difference between two timestamps.
constexpr uint64_t abs_diff(uint64_t a, uint64_t b) noexcept
{
    return (a > b) ? (a - b) : (b - a);
}

// Safe delta computation assuming monotonic timestamps.
// If timestamps move backwards, returns 0 to prevent corruption.
constexpr uint64_t safe_delta(uint64_t current, uint64_t previous) noexcept
{
    return (current >= previous) ? (current - previous) : 0u;
}

// Clamp delta to a maximum value to avoid pathological values.
constexpr uint64_t clamp_delta(uint64_t delta, uint64_t max_delta) noexcept
{
    return (delta > max_delta) ? max_delta : delta;
}

} // namespace signalfix::time_utils