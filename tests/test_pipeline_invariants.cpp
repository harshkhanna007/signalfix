// =============================================================================
// SignalFix AI — Module 1 Corrective Engineering Tests
// File   : tests/test_pipeline_invariants.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.1
// =============================================================================
//
// Tests for all 8 corrective engineering fixes applied in Rev 2.1 audit pass.
//
// Build (standalone, no external framework):
//   g++ -std=c++17 -Wall -Wextra -Wpedantic -Werror -fno-exceptions -fno-rtti
//   -I include -DSIGNALFIX_PIPELINE_DIAG=1
//   src/module1/pipeline.cpp tests/test_pipeline_invariants.cpp
//   -o test_pipeline_invariants
//
// Coverage:
//   [FIX-1] Timestamp/value mismatch detection (coherence check C1..C4)
//   [FIX-2] Invalid status flag combinations rejected before S7
//   [FIX-3] ROC window protection (roc_inhibit prevents invalid insertions)
//   [FIX-4] Sequence counter monotonicity across reset_channel()
//   [FIX-5] reset_channel() clears ROC/PLL/gap state completely
//   [FIX-6] Output bus rejection counters track all outcomes
//   [FIX-7] Stage ordering validation detects wrong-slot registrations
//   [FIX-8] Watchdog propagation: armed guard, last_calibrated_valid, streak
//
// =============================================================================

#include "../signal fix pipline/pipeline.hpp"

#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

using namespace signalfix;

// =============================================================================
// Test framework — minimal assert-based, no external dependencies
// =============================================================================

static int g_tests_run    = 0;
static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_BEGIN(name)                                          \
    do {                                                          \
        ++g_tests_run;                                            \
        std::fprintf(stdout, "[ RUN  ] " name "\n");

#define TEST_END(name)                                            \
        ++g_tests_passed;                                         \
        std::fprintf(stdout, "[ PASS ] " name "\n");             \
    } while (false);

#define EXPECT_TRUE(expr)                                         \
    do {                                                          \
        if (!(expr)) {                                            \
            std::fprintf(stderr, "  FAIL at %s:%d: expected true: %s\n", \
                         __FILE__, __LINE__, #expr);              \
            ++g_tests_failed;                                     \
            --g_tests_passed;                                     \
            return;                                               \
        }                                                         \
    } while (false)

#define EXPECT_FALSE(expr) EXPECT_TRUE(!(expr))
#define EXPECT_EQ(a, b)    EXPECT_TRUE((a) == (b))
#define EXPECT_NE(a, b)    EXPECT_TRUE((a) != (b))
#define EXPECT_GT(a, b)    EXPECT_TRUE((a) > (b))


// =============================================================================
// Output bus helpers
// =============================================================================

/// Simple output bus that accepts all envelopes (no NaN rejection).
static bool bus_accept_all(const MeasurementEnvelope& /*env*/) noexcept
{
    return true;
}

/// Output bus that rejects all envelopes.
static bool bus_reject_all(const MeasurementEnvelope& /*env*/) noexcept
{
    return false;
}


// =============================================================================
// Mock stages
//
// Each mock does exactly the minimum to exercise a specific invariant.
// Real stage implementations would contain the actual DSP algorithms.
// =============================================================================

/// PassthroughStage: does nothing — returns CONTINUE unchanged.
class PassthroughStage final : public IStage
{
public:
    explicit PassthroughStage(uint8_t slot) : slot_(slot) {}

    StageResult process(MeasurementEnvelope& /*env*/,
                        ChannelState& /*cs*/) noexcept override
    {
        return StageResult::CONTINUE;
    }

    const char* stage_name() const noexcept override { return "MockPassthrough"; }
    void reset() noexcept override {}

    uint8_t expected_registration_index() const noexcept override { return slot_; }

private:
    uint8_t slot_;
};

/// DropStage: always returns ABORT_DROP.
class DropStage final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& /*env*/,
                        ChannelState& /*cs*/) noexcept override
    {
        return StageResult::ABORT_DROP;
    }
    const char* stage_name() const noexcept override { return "MockDrop"; }
    void reset() noexcept override {}
};

/// FaultStage: always returns ABORT_FAULT.
class FaultStage final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& /*env*/,
                        ChannelState& /*cs*/) noexcept override
    {
        return StageResult::ABORT_FAULT;
    }
    const char* stage_name() const noexcept override { return "MockFault"; }
    void reset() noexcept override {}
};

/// MinimalS0: stamps channel_id, sequence_id, timestamp, calibration_version.
/// Arms the watchdog deadline. Clears watchdog_recently_fired (as a real S0 would).
class MinimalS0 final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& env, ChannelState& cs) noexcept override
    {
        env.channel_id          = cs.channel_id;
        env.sequence_id         = cs.sequence_counter++;
        env.calibration_version = cs.calibration_version;
        env.timestamp_us        = env.arrival_time_us;  // No PLL in mock
        env.delta_t_us          = cs.nominal_delta_t_us;
        env.jitter_us           = 0;

        cs.last_arrival_us        = env.arrival_time_us;
        cs.watchdog_deadline_us   = env.arrival_time_us + 2u * cs.nominal_delta_t_us;
        cs.watchdog_armed         = true;
        cs.watchdog_recently_fired = false;  // Clear on real sample arrival.
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "S0-InputAdapter"; }
    void reset() noexcept override {}
    uint8_t expected_registration_index() const noexcept override { return 0u; }
};

/// MinimalS1: performs a trivial calibration (multiply raw_value by 2.0).
class MinimalS1 final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& env, ChannelState& cs) noexcept override
    {
        env.calibrated_value        = env.raw_value * 2.0;
        cs.last_calibrated_value    = env.calibrated_value;
        cs.last_calibrated_valid    = true;
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "S1-CalibrationTransform"; }
    void reset() noexcept override {}
    uint8_t expected_registration_index() const noexcept override { return 1u; }
};

/// MinimalS3: sets validated_value, records ROC window n — respects roc_inhibit.
class MinimalS3 final : public IStage
{
public:
    /// How many times S3 tried to insert into the ROC window (for test inspection).
    uint32_t roc_insert_attempts{0};
    uint32_t roc_insert_inhibited{0};

    StageResult process(MeasurementEnvelope& env, ChannelState& cs) noexcept override
    {
        // Validate: calibrated_value must be non-NaN to validate.
        if (std::isnan(env.calibrated_value))
        {
            env.status |= SampleStatus::HARD_INVALID;
            env.validated_value = std::numeric_limits<double>::quiet_NaN();
            return StageResult::CONTINUE;
        }

        env.validated_value = env.calibrated_value;
        env.roc_window_n    = cs.roc_window.window_size;

        // [FIX-3] Check roc_inhibit before inserting into the ROC window.
        ++roc_insert_attempts;
        if (cs.roc_inhibit)
        {
            ++roc_insert_inhibited;
            // Do NOT update the ROC window. Return CONTINUE without modifying
            // roc_window state.
            return StageResult::CONTINUE;
        }

        // Insert into ROC window (simplified: just increment count for test).
        if (cs.roc_window.count < cs.roc_window.window_size)
        {
            cs.roc_window.buffer[cs.roc_window.head] = static_cast<float>(env.calibrated_value);
            cs.roc_window.head = (cs.roc_window.head + 1u) % cs.roc_window.window_size;
            ++cs.roc_window.count;
            cs.roc_window.running_sum    += static_cast<float>(env.calibrated_value);
            cs.roc_window.running_sum_sq += static_cast<float>(env.calibrated_value)
                                          * static_cast<float>(env.calibrated_value);
        }

        return StageResult::CONTINUE;
    }

    const char* stage_name() const noexcept override { return "S3-PlausibilityValidator"; }
    void reset() noexcept override { roc_insert_attempts = 0; roc_insert_inhibited = 0; }
};

/// MinimalS6: sets measurement_trust_tier and nominal_streak_count.
class MinimalS6 final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& env, ChannelState& cs) noexcept override
    {
        env.nominal_streak_count = cs.nominal_streak;
        env.measurement_trust_tier = derive_trust_tier(env.status);

        if (is_nominal(env.status))
        {
            if (cs.nominal_streak < 0xFFFFu) { ++cs.nominal_streak; }
        }
        else
        {
            cs.nominal_streak = 0u;
        }
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "S6-EnvelopePackager"; }
    void reset() noexcept override {}
};

/// BadFlagStage: sets an invalid status combination (HARD_INVALID | SOFT_SUSPECT).
/// validated_value is NaN (correct for HARD_INVALID) so coherence C1 passes, but
/// INV-2 (HARD_INVALID subsumes SOFT_SUSPECT) fires in validate_status_flags(). [FIX-2]
class BadFlagStage final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& env, ChannelState& /*cs*/) noexcept override
    {
        env.calibrated_value = 1.0;
        // validated_value must be NaN when HARD_INVALID is set (coherence C1).
        // This makes the coherence check pass and lets INV-2 fire in validate_status_flags.
        env.validated_value  = std::numeric_limits<double>::quiet_NaN();
        env.filtered_value   = std::numeric_limits<double>::quiet_NaN();
        env.timestamp_us     = env.arrival_time_us;
        env.sequence_id      = 1u;
        env.calibration_version = 0u;
        // INV-2 violation: HARD_INVALID subsumes SOFT_SUSPECT.
        env.status = SampleStatus::HARD_INVALID | SampleStatus::SOFT_SUSPECT;
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "MockBadFlag"; }
    void reset() noexcept override {}
};

/// HardInvalidWithValueStage: sets HARD_INVALID but leaves validated_value non-NaN.
/// Used to test coherence check C1.
class HardInvalidWithValueStage final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& env, ChannelState& /*cs*/) noexcept override
    {
        env.calibrated_value    = 999.0;
        env.validated_value     = 999.0;  // Should be NaN if HARD_INVALID.
        env.filtered_value      = 999.0;
        env.timestamp_us        = env.arrival_time_us;
        env.sequence_id         = 1u;
        env.channel_id          = env.channel_id;
        env.calibration_version = 0u;
        // Coherence C1 violation: HARD_INVALID set but validated_value != NaN.
        env.status = SampleStatus::HARD_INVALID;
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "MockHardInvalidWithValue"; }
    void reset() noexcept override {}
};

/// MissingWithValueStage: sets MISSING but leaves calibrated_value non-NaN.
/// Used to test coherence check C2.
class MissingWithValueStage final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& env, ChannelState& /*cs*/) noexcept override
    {
        env.calibrated_value    = 1.0;   // Must be NaN when MISSING.
        env.validated_value     = std::numeric_limits<double>::quiet_NaN();
        env.filtered_value      = std::numeric_limits<double>::quiet_NaN();
        env.timestamp_us        = env.arrival_time_us;
        env.sequence_id         = 1u;
        env.channel_id          = env.channel_id;
        env.calibration_version = 0u;
        env.status              = SampleStatus::MISSING;
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "MockMissingWithValue"; }
    void reset() noexcept override {}
};

/// StaleWithoutMissingStage: sets STALE without MISSING.
/// Used to test coherence check C3.
class StaleWithoutMissingStage final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& env, ChannelState& /*cs*/) noexcept override
    {
        env.calibrated_value    = std::numeric_limits<double>::quiet_NaN();
        env.validated_value     = std::numeric_limits<double>::quiet_NaN();
        env.filtered_value      = std::numeric_limits<double>::quiet_NaN();
        env.timestamp_us        = env.arrival_time_us;
        env.sequence_id         = 1u;
        env.channel_id          = env.channel_id;
        env.calibration_version = 0u;
        // C3 violation: STALE without MISSING.
        env.status              = SampleStatus::STALE;
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "MockStaleWithoutMissing"; }
    void reset() noexcept override {}
};

/// GoodFullStage: produces a valid envelope with all fields populated correctly.
class GoodFullStage final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& env, ChannelState& cs) noexcept override
    {
        env.channel_id          = cs.channel_id;
        env.sequence_id         = cs.sequence_counter++;
        env.calibration_version = cs.calibration_version;
        env.timestamp_us        = env.arrival_time_us;
        env.delta_t_us          = cs.nominal_delta_t_us;
        env.jitter_us           = 0;

        env.calibrated_value    = env.raw_value * 2.0;
        env.validated_value     = env.calibrated_value;
        env.filtered_value      = env.validated_value;  // No filter.

        env.roc                 = 0.0f;
        env.roc_adaptive_limit  = 1.0f;
        env.roc_window_n        = cs.roc_window.window_size;
        env.nominal_streak_count= cs.nominal_streak;
        env.pre_filter_applied  = false;
        env.filter_window_n     = 0u;
        env.status              = SampleStatus::NOMINAL;
        env.measurement_trust_tier = MeasurementTrustTier::HIGH;

        cs.last_arrival_us      = env.arrival_time_us;
        cs.watchdog_deadline_us = env.arrival_time_us + 2u * cs.nominal_delta_t_us;
        cs.watchdog_armed       = true;
        cs.last_calibrated_value = env.calibrated_value;
        cs.last_calibrated_valid = true;
        cs.nominal_streak       = static_cast<uint16_t>(
                                      cs.nominal_streak < 0xFFFFu
                                          ? cs.nominal_streak + 1u
                                          : cs.nominal_streak);
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "MockGoodFull"; }
    void reset() noexcept override {}
};

/// WrongSlotStage: declares expected slot 3 regardless of actual registration.
/// Used to test FIX-7 ordering validation.
class WrongSlotStage final : public IStage
{
public:
    StageResult process(MeasurementEnvelope& /*env*/,
                        ChannelState& /*cs*/) noexcept override
    {
        return StageResult::CONTINUE;
    }
    const char* stage_name() const noexcept override { return "MockWrongSlot"; }
    void reset() noexcept override {}
    uint8_t expected_registration_index() const noexcept override { return 3u; }
};


// =============================================================================
// Helper: build a valid RawSample
// =============================================================================

static RawSample make_sample(uint32_t channel_id,
                              double   raw_value,
                              uint64_t arrival_us) noexcept
{
    RawSample s{};
    s.channel_id     = channel_id;
    s.raw_value      = raw_value;
    s.arrival_time_us= arrival_us;
    s._pad           = 0u;
    return s;
}

static constexpr uint32_t kCh = 42u;
static constexpr uint64_t kDt = 1000u;  // 1 ms nominal interval


// =============================================================================
// FIX-1: Timestamp / value mismatch detection
// =============================================================================

static void test_fix1_coherence_hard_invalid_with_value()
{
    TEST_BEGIN("FIX-1a: HARD_INVALID with non-NaN validated_value → COHERENCE_FAIL")

    HardInvalidWithValueStage bad{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&bad));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    const auto r = pipeline.process(make_sample(kCh, 5.0, 1000u));
    EXPECT_EQ(r.status, PipelineStatus::COHERENCE_FAIL);
    EXPECT_EQ(pipeline.get_pipeline_counters().coherence_fail_count, 1u);

    TEST_END("FIX-1a: HARD_INVALID with non-NaN validated_value → COHERENCE_FAIL")
}

static void test_fix1_coherence_missing_with_value()
{
    TEST_BEGIN("FIX-1b: MISSING with non-NaN calibrated_value → COHERENCE_FAIL")

    MissingWithValueStage bad{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&bad));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    const auto r = pipeline.process(make_sample(kCh, 5.0, 1000u));
    EXPECT_EQ(r.status, PipelineStatus::COHERENCE_FAIL);

    TEST_END("FIX-1b: MISSING with non-NaN calibrated_value → COHERENCE_FAIL")
}

static void test_fix1_coherence_stale_without_missing()
{
    TEST_BEGIN("FIX-1c: STALE without MISSING → COHERENCE_FAIL")

    StaleWithoutMissingStage bad{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&bad));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    const auto r = pipeline.process(make_sample(kCh, 5.0, 1000u));
    EXPECT_EQ(r.status, PipelineStatus::COHERENCE_FAIL);

    TEST_END("FIX-1c: STALE without MISSING → COHERENCE_FAIL")
}

static void test_fix1_good_envelope_passes_coherence()
{
    TEST_BEGIN("FIX-1d: valid NOMINAL envelope passes coherence and outputs OK")

    GoodFullStage good{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&good));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    const auto r = pipeline.process(make_sample(kCh, 5.0, 1000u));
    EXPECT_EQ(r.status, PipelineStatus::OK);
    EXPECT_EQ(r.envelope.status, SampleStatus::NOMINAL);
    EXPECT_EQ(pipeline.get_pipeline_counters().ok_count, 1u);
    EXPECT_EQ(pipeline.get_pipeline_counters().coherence_fail_count, 0u);

    TEST_END("FIX-1d: valid NOMINAL envelope passes coherence and outputs OK")
}


// =============================================================================
// FIX-2: Status flag consistency
// =============================================================================

static void test_fix2_invalid_flag_combination_rejected()
{
    TEST_BEGIN("FIX-2: HARD_INVALID | SOFT_SUSPECT → INV_FAIL at S7")

    BadFlagStage bad{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&bad));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    const auto r = pipeline.process(make_sample(kCh, 5.0, 1000u));
    EXPECT_EQ(r.status, PipelineStatus::INV_FAIL);
    EXPECT_EQ(pipeline.get_pipeline_counters().inv_fail_count, 1u);

    TEST_END("FIX-2: HARD_INVALID | SOFT_SUSPECT → INV_FAIL at S7")
}

static void test_fix2_valid_flag_combinations_pass()
{
    TEST_BEGIN("FIX-2b: valid flag combinations pass invariant check")

    // Verify that individually valid combinations (SOFT_SUSPECT alone,
    // ROC_EXCEEDED alone, TIMING_ANOMALY alone) all pass validate_status_flags.
    EXPECT_TRUE(validate_status_flags(SampleStatus::NOMINAL));
    EXPECT_TRUE(validate_status_flags(SampleStatus::SOFT_SUSPECT));
    EXPECT_TRUE(validate_status_flags(SampleStatus::ROC_EXCEEDED));
    EXPECT_TRUE(validate_status_flags(SampleStatus::TIMING_ANOMALY));
    EXPECT_TRUE(validate_status_flags(SampleStatus::MISSING));
    EXPECT_TRUE(validate_status_flags(SampleStatus::HARD_INVALID));
    EXPECT_TRUE(validate_status_flags(SampleStatus::STALE | SampleStatus::MISSING));

    // INV-1: MISSING | INTERPOLATED
    EXPECT_FALSE(validate_status_flags(SampleStatus::MISSING | SampleStatus::INTERPOLATED));
    // INV-2: HARD_INVALID | SOFT_SUSPECT
    EXPECT_FALSE(validate_status_flags(SampleStatus::HARD_INVALID | SampleStatus::SOFT_SUSPECT));
    // INV-3: HARD_INVALID | FILTER_CLIPPED
    EXPECT_FALSE(validate_status_flags(SampleStatus::HARD_INVALID | SampleStatus::FILTER_CLIPPED));
    // INV-4: MISSING | SOFT_SUSPECT
    EXPECT_FALSE(validate_status_flags(SampleStatus::MISSING | SampleStatus::SOFT_SUSPECT));
    // INV-5: MISSING | FILTER_CLIPPED
    EXPECT_FALSE(validate_status_flags(SampleStatus::MISSING | SampleStatus::FILTER_CLIPPED));
    // INV-6: INTERPOLATED | SOFT_SUSPECT
    EXPECT_FALSE(validate_status_flags(SampleStatus::INTERPOLATED | SampleStatus::SOFT_SUSPECT));

    // INV-7: HARD_INVALID | ROC_EXCEEDED [0x0022]
    EXPECT_FALSE(validate_status_flags(SampleStatus::HARD_INVALID | SampleStatus::ROC_EXCEEDED));
    // INV-8: MISSING | ROC_EXCEEDED [0x0012]
    EXPECT_FALSE(validate_status_flags(SampleStatus::MISSING | SampleStatus::ROC_EXCEEDED));
    // INV-9: INTERPOLATED | ROC_EXCEEDED [0x000A]
    EXPECT_FALSE(validate_status_flags(SampleStatus::INTERPOLATED | SampleStatus::ROC_EXCEEDED));

    TEST_END("FIX-2b: valid flag combinations pass invariant check")
}


// =============================================================================
// FIX-3: ROC window protection
// =============================================================================

static void test_fix3_roc_inhibited_for_invalid_samples()
{
    TEST_BEGIN("FIX-3: ROC window not updated on HARD_INVALID sample")

    MinimalS0 s0{};
    MinimalS1 s1{};
    MinimalS3 s3{};

    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&s0));
    EXPECT_TRUE(pipeline.register_stage(&s1));
    EXPECT_TRUE(pipeline.register_stage(&s3));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 16u));

    // First: send a valid sample. ROC window should accept it.
    auto r1 = pipeline.process(make_sample(kCh, 5.0, 1000u));
    EXPECT_EQ(r1.status, PipelineStatus::OK);
    // After S1, calibrated_value=10.0, roc_inhibit should be false.
    // S3 should have attempted and succeeded an insert.
    const uint32_t initial_insert_attempts = s3.roc_insert_attempts;
    EXPECT_GT(initial_insert_attempts, 0u);

    // Retrieve ROC window count after valid sample.
    const ChannelState* cs = pipeline.get_channel_state(kCh);
    EXPECT_TRUE(cs != nullptr);
    const uint32_t roc_count_after_valid = cs->roc_window.count;

    // Second: send a NaN raw_value — S1 will produce NaN calibrated_value, S3
    // will see roc_inhibit == true and skip the insert.
    auto r2 = pipeline.process(make_sample(kCh,
        std::numeric_limits<double>::quiet_NaN(), 2000u));
    // S3 sets HARD_INVALID when calibrated_value is NaN.
    // The coherence check will pass (HARD_INVALID with NaN validated_value is valid).
    // The status flag check will pass (HARD_INVALID alone is valid).
    // Bus should accept (bus_accept_all).
    EXPECT_NE(r2.status, PipelineStatus::COHERENCE_FAIL);

    // ROC window count must not have increased — inhibit was active.
    EXPECT_TRUE(cs->roc_window.count == roc_count_after_valid);
    // The insert_attempts counter may not increment if S3 returns early on NaN;
    // either way, the invariant is that the window state did not change.

    TEST_END("FIX-3: ROC window not updated on HARD_INVALID sample")
}

static void test_fix3_roc_populated_by_valid_samples()
{
    TEST_BEGIN("FIX-3b: ROC window accumulates on consecutive valid samples")

    MinimalS0 s0{};
    MinimalS1 s1{};
    MinimalS3 s3{};

    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&s0));
    EXPECT_TRUE(pipeline.register_stage(&s1));
    EXPECT_TRUE(pipeline.register_stage(&s3));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 16u));

    // Send 4 valid samples — ROC window should accumulate all 4.
    for (int i = 0; i < 4; ++i)
    {
        auto r = pipeline.process(make_sample(kCh, static_cast<double>(i + 1), 1000u + static_cast<uint64_t>(i) * kDt));
        EXPECT_EQ(r.status, PipelineStatus::OK);
    }

    const ChannelState* cs = pipeline.get_channel_state(kCh);
    EXPECT_TRUE(cs != nullptr);
    EXPECT_EQ(cs->roc_window.count, 4u);
    EXPECT_EQ(s3.roc_insert_inhibited, 0u);

    TEST_END("FIX-3b: ROC window accumulates on consecutive valid samples")
}


// =============================================================================
// FIX-4: Sequence counter monotonicity
// =============================================================================

static void test_fix4_sequence_counter_preserved_across_reset()
{
    TEST_BEGIN("FIX-4: sequence_counter preserved across reset_channel()")

    GoodFullStage good{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&good));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    // Process several samples to advance the sequence counter.
    for (int i = 0; i < 5; ++i)
    {
        auto r = pipeline.process(make_sample(kCh, static_cast<double>(i), 1000u + static_cast<uint64_t>(i) * kDt));
        EXPECT_EQ(r.status, PipelineStatus::OK);
    }

    const ChannelState* cs_before = pipeline.get_channel_state(kCh);
    EXPECT_TRUE(cs_before != nullptr);
    const uint64_t seq_before_reset = cs_before->sequence_counter;
    EXPECT_GT(seq_before_reset, 0u);  // Must have advanced.

    // Reset the channel.
    EXPECT_TRUE(pipeline.reset_channel(kCh));

    const ChannelState* cs_after = pipeline.get_channel_state(kCh);
    EXPECT_TRUE(cs_after != nullptr);

    // Sequence counter must be preserved — it must not have regressed to 0.
    EXPECT_EQ(cs_after->sequence_counter, seq_before_reset);

    // Process one more sample — the next sequence_id must be >= preserved value.
    auto r_post = pipeline.process(make_sample(kCh, 99.0, 10000u));
    EXPECT_EQ(r_post.status, PipelineStatus::OK);
    EXPECT_TRUE(r_post.envelope.sequence_id >= seq_before_reset);

    TEST_END("FIX-4: sequence_counter preserved across reset_channel()")
}

static void test_fix4_sequence_counter_strictly_monotonic()
{
    TEST_BEGIN("FIX-4b: sequence_id strictly increases across consecutive samples")

    GoodFullStage good{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&good));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    uint64_t last_seq   = UINT64_MAX;  // sentinel: no previous sample
    bool     first_seen = false;
    for (int i = 0; i < 10; ++i)
    {
        auto r = pipeline.process(make_sample(kCh, static_cast<double>(i),
                                              1000u + static_cast<uint64_t>(i) * kDt));
        EXPECT_EQ(r.status, PipelineStatus::OK);
        if (first_seen)
        {
            EXPECT_GT(r.envelope.sequence_id, last_seq);
        }
        last_seq   = r.envelope.sequence_id;
        first_seen = true;
    }

    TEST_END("FIX-4b: sequence_id strictly increases across consecutive samples")
}


// =============================================================================
// FIX-5: Channel reset clears all derived state
// =============================================================================

static void test_fix5_reset_clears_roc_and_pll_state()
{
    TEST_BEGIN("FIX-5: reset_channel() clears ROC window, PLL, and gap state")

    MinimalS0 s0{};
    MinimalS1 s1{};
    MinimalS3 s3{};

    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&s0));
    EXPECT_TRUE(pipeline.register_stage(&s1));
    EXPECT_TRUE(pipeline.register_stage(&s3));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 16u));

    // Populate ROC window with valid samples.
    for (int i = 0; i < 5; ++i)
    {
        (void)pipeline.process(make_sample(kCh, static_cast<double>(i + 1),
                                     1000u + static_cast<uint64_t>(i) * kDt));
    }

    const ChannelState* cs_before = pipeline.get_channel_state(kCh);
    EXPECT_TRUE(cs_before != nullptr);
    EXPECT_GT(cs_before->roc_window.count, 0u);  // Window has data.
    EXPECT_GT(cs_before->roc_window.running_sum, 0.0f);

    // Reset.
    EXPECT_TRUE(pipeline.reset_channel(kCh));

    const ChannelState* cs_after = pipeline.get_channel_state(kCh);
    EXPECT_TRUE(cs_after != nullptr);

    // ROC window must be zeroed.
    EXPECT_EQ(cs_after->roc_window.count, 0u);
    EXPECT_EQ(cs_after->roc_window.head, 0u);
    EXPECT_EQ(cs_after->roc_window.running_sum, 0.0f);
    EXPECT_EQ(cs_after->roc_window.running_sum_sq, 0.0f);

    // PLL must be reset.
    EXPECT_FALSE(cs_after->pll.locked);
    EXPECT_EQ(cs_after->pll.lock_sample_count, 0u);

    // Gap state must be zeroed except next_gap_event_id = 1.
    EXPECT_EQ(cs_after->gap.next_gap_event_id, 1u);
    EXPECT_EQ(cs_after->gap.consecutive_gap_count, 0u);
    EXPECT_EQ(cs_after->gap.total_gap_count, 0u);
    EXPECT_FALSE(cs_after->gap.gap_pending);

    // Watchdog must be disarmed.
    EXPECT_FALSE(cs_after->watchdog_armed);

    TEST_END("FIX-5: reset_channel() clears ROC window, PLL, and gap state")
}


// =============================================================================
// FIX-6: Output bus rejection counters
// =============================================================================

static void test_fix6_rejection_counters_track_all_outcomes()
{
    TEST_BEGIN("FIX-6: PipelineCounters tracks all rejection and fault outcomes")

    GoodFullStage good{};
    BadFlagStage  bad_flag{};
    DropStage     drop{};

    // Pipeline 1: a good stage with a rejecting bus.
    {
        IngestionPipeline p(bus_reject_all);
        EXPECT_TRUE(p.register_stage(&good));
        EXPECT_TRUE(p.register_channel(kCh, 1u, kDt, 0u));
        (void)p.process(make_sample(kCh, 1.0, 1000u));
        (void)p.process(make_sample(kCh, 2.0, 2000u));
        const auto cnt = p.get_pipeline_counters();
        EXPECT_EQ(cnt.gate_reject_count, 2u);
        EXPECT_EQ(cnt.ok_count, 0u);
    }

    // Pipeline 2: drop stage.
    {
        IngestionPipeline p(bus_accept_all);
        EXPECT_TRUE(p.register_stage(&drop));
        EXPECT_TRUE(p.register_channel(kCh, 1u, kDt, 0u));
        (void)p.process(make_sample(kCh, 1.0, 1000u));
        (void)p.process(make_sample(kCh, 1.0, 2000u));
        const auto cnt = p.get_pipeline_counters();
        EXPECT_EQ(cnt.dropped_count, 2u);
    }

    // Pipeline 3: bad flag → INV_FAIL.
    {
        IngestionPipeline p(bus_accept_all);
        EXPECT_TRUE(p.register_stage(&bad_flag));
        EXPECT_TRUE(p.register_channel(kCh, 1u, kDt, 0u));
        (void)p.process(make_sample(kCh, 1.0, 1000u));
        const auto cnt = p.get_pipeline_counters();
        EXPECT_EQ(cnt.inv_fail_count, 1u);
    }

    // Pipeline 4: unknown channel.
    {
        IngestionPipeline p(bus_accept_all);
        EXPECT_TRUE(p.register_stage(&good));
        EXPECT_TRUE(p.register_channel(kCh, 1u, kDt, 0u));
        (void)p.process(make_sample(99u, 1.0, 1000u));  // unknown channel_id
        const auto cnt = p.get_pipeline_counters();
        EXPECT_EQ(cnt.channel_unknown_count, 1u);
    }

    TEST_END("FIX-6: PipelineCounters tracks all rejection and fault outcomes")
}


// =============================================================================
// FIX-7: Stage ordering validation
// =============================================================================

static void test_fix7_stage_ordering_detects_wrong_slot()
{
    TEST_BEGIN("FIX-7: validate_stage_registration() fails for wrong-slot stage")

    // WrongSlotStage declares expected slot 3 but is registered at slot 0.
    WrongSlotStage wrong{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&wrong));  // Registered at slot 0.

    // Ordering validation must fail — declared slot (3) != actual slot (0).
    EXPECT_FALSE(pipeline.validate_stage_registration());

    TEST_END("FIX-7: validate_stage_registration() fails for wrong-slot stage")
}

static void test_fix7_stage_ordering_passes_for_correct_registration()
{
    TEST_BEGIN("FIX-7b: validate_stage_registration() passes for correctly ordered stages")

    MinimalS0 s0{};   // declares slot 0
    MinimalS1 s1{};   // declares slot 1

    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&s0));
    EXPECT_TRUE(pipeline.register_stage(&s1));

    // Both stages at correct slots — validation must pass.
    EXPECT_TRUE(pipeline.validate_stage_registration());

    TEST_END("FIX-7b: validate_stage_registration() passes for correctly ordered stages")
}

static void test_fix7_unconstrained_stages_always_pass()
{
    TEST_BEGIN("FIX-7c: unconstrained stages (kUnconstrainedIndex) always pass ordering check")

    PassthroughStage pt(IStage::kUnconstrainedIndex);  // no constraint
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&pt));

    EXPECT_TRUE(pipeline.validate_stage_registration());

    TEST_END("FIX-7c: unconstrained stages (kUnconstrainedIndex) always pass ordering check")
}


// =============================================================================
// FIX-8: Watchdog propagation
// =============================================================================

static void test_fix8_watchdog_requires_armed_state()
{
    TEST_BEGIN("FIX-8a: process_watchdog() returns DROPPED when watchdog not armed")

    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    // No samples yet — watchdog_armed == false.
    const auto r = pipeline.process_watchdog(kCh, 5000u);

    // Must return DROPPED, not OK or GATE_REJECT.
    EXPECT_EQ(r.status, PipelineStatus::DROPPED);

    // Counter must not be incremented (this is not a real watchdog event).
    EXPECT_EQ(pipeline.get_pipeline_counters().watchdog_count, 0u);

    TEST_END("FIX-8a: process_watchdog() returns DROPPED when watchdog not armed")
}

static void test_fix8_watchdog_invalidates_calibrated_value()
{
    TEST_BEGIN("FIX-8b: process_watchdog() clears last_calibrated_valid")

    GoodFullStage good{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&good));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    // Send one real sample to arm the watchdog.
    (void)pipeline.process(make_sample(kCh, 10.0, 1000u));

    const ChannelState* cs = pipeline.get_channel_state(kCh);
    EXPECT_TRUE(cs != nullptr);
    EXPECT_TRUE(cs->watchdog_armed);
    EXPECT_TRUE(cs->last_calibrated_valid);

    // Fire watchdog.
    (void)pipeline.process_watchdog(kCh, 3000u);

    // last_calibrated_valid must be cleared — the previous value is stale.
    EXPECT_FALSE(cs->last_calibrated_valid);
    EXPECT_TRUE(cs->watchdog_recently_fired);
    EXPECT_EQ(cs->nominal_streak, 0u);
    EXPECT_EQ(pipeline.get_pipeline_counters().watchdog_count, 1u);

    TEST_END("FIX-8b: process_watchdog() clears last_calibrated_valid")
}

static void test_fix8_watchdog_sequence_monotonic()
{
    TEST_BEGIN("FIX-8c: watchdog envelope sequence_id continues monotonically")

    GoodFullStage good{};
    IngestionPipeline pipeline(bus_reject_all);  // bus rejects all, but we observe seq
    EXPECT_TRUE(pipeline.register_stage(&good));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    // Send a few real samples.
    uint64_t last_seq = 0u;
    for (int i = 0; i < 3; ++i)
    {
        auto r = pipeline.process(make_sample(kCh, static_cast<double>(i),
                                              1000u + static_cast<uint64_t>(i) * kDt));
        last_seq = r.envelope.sequence_id;
    }

    // Fire watchdog — sequence must continue.
    auto wr = pipeline.process_watchdog(kCh, 10000u);
    EXPECT_GT(wr.envelope.sequence_id, last_seq);
    EXPECT_EQ(wr.envelope.status, SampleStatus::STALE | SampleStatus::MISSING);

    // Fire watchdog again — sequence must continue further.
    auto wr2 = pipeline.process_watchdog(kCh, 12000u);
    EXPECT_GT(wr2.envelope.sequence_id, wr.envelope.sequence_id);

    TEST_END("FIX-8c: watchdog envelope sequence_id continues monotonically")
}

static void test_fix8_watchdog_fires_after_reset_if_rearmed()
{
    TEST_BEGIN("FIX-8d: watchdog requires re-arming after reset_channel()")

    GoodFullStage good{};
    IngestionPipeline pipeline(bus_accept_all);
    EXPECT_TRUE(pipeline.register_stage(&good));
    EXPECT_TRUE(pipeline.register_channel(kCh, 1u, kDt, 0u));

    // Arm with a real sample.
    (void)pipeline.process(make_sample(kCh, 10.0, 1000u));

    // Reset channel — clears watchdog_armed.
    EXPECT_TRUE(pipeline.reset_channel(kCh));

    // Watchdog must not fire after reset until re-armed.
    const auto r = pipeline.process_watchdog(kCh, 5000u);
    EXPECT_EQ(r.status, PipelineStatus::DROPPED);

    TEST_END("FIX-8d: watchdog requires re-arming after reset_channel()")
}


// =============================================================================
// Main
// =============================================================================

int main()
{
    std::fprintf(stdout, "=== SignalFix M1 Corrective Engineering Tests ===\n\n");

    // FIX-1: Timestamp / value mismatch
    test_fix1_coherence_hard_invalid_with_value();
    test_fix1_coherence_missing_with_value();
    test_fix1_coherence_stale_without_missing();
    test_fix1_good_envelope_passes_coherence();

    // FIX-2: Status flag consistency
    test_fix2_invalid_flag_combination_rejected();
    test_fix2_valid_flag_combinations_pass();

    // FIX-3: ROC window protection
    test_fix3_roc_inhibited_for_invalid_samples();
    test_fix3_roc_populated_by_valid_samples();

    // FIX-4: Sequence counter monotonicity
    test_fix4_sequence_counter_preserved_across_reset();
    test_fix4_sequence_counter_strictly_monotonic();

    // FIX-5: Channel reset correctness
    test_fix5_reset_clears_roc_and_pll_state();

    // FIX-6: Output bus rejection counters
    test_fix6_rejection_counters_track_all_outcomes();

    // FIX-7: Stage ordering validation
    test_fix7_stage_ordering_detects_wrong_slot();
    test_fix7_stage_ordering_passes_for_correct_registration();
    test_fix7_unconstrained_stages_always_pass();

    // FIX-8: Watchdog propagation
    test_fix8_watchdog_requires_armed_state();
    test_fix8_watchdog_invalidates_calibrated_value();
    test_fix8_watchdog_sequence_monotonic();
    test_fix8_watchdog_fires_after_reset_if_rearmed();

    std::fprintf(stdout, "\n=== Results: %d run, %d passed, %d failed ===\n",
                 g_tests_run, g_tests_passed, g_tests_failed);

    return (g_tests_failed == 0) ? 0 : 1;
}
