import sys

with open(r"c:\signalfix-ai\src\stages\s7.cpp", "r") as f:
    lines = f.readlines()

start_idx = 312 # 0-indexed, so line 313
end_idx = 465

original = "".join(lines[start_idx:end_idx])

new_content = """    auto& fs = channel_state.failure_state;
    const FailureMode suggested = envelope.failure_hint;
    const float suggested_conf  = envelope.failure_confidence;

    // Constant Thresholds
    constexpr uint8_t kEnterThreshold = 3u;
    constexpr uint8_t kExitThreshold  = 5u;
    constexpr float   kAlpha          = 0.3f; // Smoothing factor
    
    // Safety: Pre-compute failure conditions
    const bool is_hard_invalid = has_flag(envelope.status, SampleStatus::HARD_INVALID);
    const bool is_infinite = !std::isfinite(envelope.calibrated_value);

    // CASE A: Hard Failure Override (Safety Critical, highest priority)
    // NaN, Inf, HARD_INVALID, or suggested INVALID immediately latch mode with zero hysteresis.
    if (suggested == FailureMode::INVALID || is_infinite || is_hard_invalid)
    {
        fs.latched_hint       = FailureMode::INVALID;
        fs.latched_confidence = 1.0f;
        if (fs.duration_frames < 0xFFFFu) fs.duration_frames++;
        fs.enter_counter      = 0u;
        fs.exit_counter       = 0u;
        fs.last_failure_ts_us = envelope.arrival_time_us;
    }
    // CASE B: Suggested failure (suggested != NONE)
    else if (suggested != FailureMode::NONE)
    {
        if (suggested == fs.latched_hint)
        {
            // (1) Same as current failure
            if (fs.duration_frames < 0xFFFFu) fs.duration_frames++;
            fs.exit_counter = 0u;
            fs.enter_counter = 0u;
            fs.latched_confidence = (0.7f * fs.latched_confidence) + (kAlpha * suggested_conf);
            fs.last_failure_ts_us = envelope.arrival_time_us;
        }
        else
        {
            // (2) Different failure
            // Severity: NONE < DRIFT < GAP < STALE < SPIKE < INVALID
            if (static_cast<uint8_t>(suggested) > static_cast<uint8_t>(fs.latched_hint))
            {
                // Switch immediately for higher severity
                fs.latched_hint       = suggested;
                fs.duration_frames    = 1u;
                fs.enter_counter      = 0u;
                fs.exit_counter       = 0u;
                fs.latched_confidence = suggested_conf;
                fs.last_failure_ts_us = envelope.arrival_time_us;
            }
            else
            {
                // Wait for hysteresis if lower severity
                fs.enter_counter++;
                if (fs.enter_counter >= kEnterThreshold)
                {
                    fs.latched_hint       = suggested;
                    fs.duration_frames    = 1u;
                    fs.enter_counter      = 0u;
                    fs.exit_counter       = 0u;
                    fs.latched_confidence = suggested_conf;
                    fs.last_failure_ts_us = envelope.arrival_time_us;
                }
            }
        }
    }
    // CASE C: No failure (suggested == NONE)
    else
    {
        if (fs.latched_hint != FailureMode::NONE)
        {
            fs.exit_counter++;
            if (fs.exit_counter >= kExitThreshold)
            {
                fs.latched_hint       = FailureMode::NONE;
                fs.duration_frames    = 0u;
                fs.enter_counter      = 0u;
                fs.exit_counter       = 0u;
                fs.latched_confidence = 0.0f;
            }
            else
            {
                if (fs.duration_frames < 0xFFFFu) fs.duration_frames++;
                // Decay confidence slightly during recovery window
                fs.latched_confidence = fs.latched_confidence * 0.95f;
            }
        }
        else
        {
            fs.duration_frames    = 0u;
            fs.enter_counter      = 0u;
            fs.exit_counter       = 0u;
            fs.latched_confidence = 0.0f;
        }
    }

    // Safety: Clamp confidence to physical [0, 1] range
    if (fs.latched_confidence > 1.0f) fs.latched_confidence = 1.0f;
    if (fs.latched_confidence < 0.0f) fs.latched_confidence = 0.0f;

    // Sync persistent state back to the envelope for downstream consumption
    envelope.failure_hint          = fs.latched_hint;
    envelope.failure_confidence    = fs.latched_confidence;
    envelope.failure_duration      = static_cast<uint32_t>(fs.duration_frames);
    envelope.last_failure_timestamp_us = fs.last_failure_ts_us;
"""

new_file_content = "".join(lines[:start_idx]) + new_content + "".join(lines[end_idx:])
with open(r"c:\signalfix-ai\src\stages\s7.cpp", "w") as f:
    f.write(new_file_content)

print("Done.")
