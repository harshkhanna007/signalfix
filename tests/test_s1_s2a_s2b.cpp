// =============================================================================
// SignalFix AI — S1 / S2a / S2b integration test harness
// =============================================================================
//
// Compile:
//   g++ -std=c++17 -O2 -fno-exceptions -fno-rtti -DNDEBUG \
//       -I include \
//       -o test_s1_s2 \
//       tests/test_s1_s2a_s2b.cpp \
//       src/module1/stages/stage_s0_input_adapter.cpp \
//       src/module1/stages/stage_s1_calibration.cpp \
//       src/module1/stages/stage_s2a_gap_detection.cpp \
//       src/module1/stages/stage_s2b_clock_alignment.cpp
//
// =============================================================================

#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/stages/stage_s1_calibration.hpp"
#include "signalfix/module1/stages/stage_s2a_gap_detection.hpp"
#include "signalfix/module1/stages/stage_s2b_clock_alignment.hpp"

#include <cstdio>
#include <cmath>
#include <cstdint>
#include <limits>

// =============================================================================
// Minimal test framework
// =============================================================================

namespace {

static int g_pass = 0;
static int g_fail = 0;

#define CHECK(cond, msg)                                           \
    do {                                                            \
        if (cond) { std::fprintf(stdout, "  [PASS] %s\n", msg);   \
                    ++g_pass; }                                    \
        else      { std::fprintf(stdout, "  [FAIL] %s  (line %d)\n", \
                                  msg, __LINE__);                   \
                    ++g_fail; }                                    \
    } while (false)

#define TEST(name) std::fprintf(stdout, "\n--- %s ---\n", name)

// ── Helpers ──────────────────────────────────────────────────────────────────

signalfix::ChannelState make_cs(
    uint32_t channel_id         = 0xAA00'0001u,
    uint64_t nominal_delta_t_us = 10'000u)
{
    signalfix::ChannelState cs{};
    signalfix::init_channel_state(cs, channel_id, 1u, nominal_delta_t_us, 30u);
    return cs;
}

signalfix::MeasurementEnvelope make_env(
    uint32_t channel_id     = 0xAA00'0001u,
    uint64_t arrival_us     = 1'000'000u,
    double   raw_val        = 2048.0,
    uint64_t delta_t_us     = 10'000u)
{
    signalfix::MeasurementEnvelope env = signalfix::make_nominal_envelope();
    env.channel_id      = channel_id;
    env.arrival_time_us = arrival_us;
    env.raw_value       = raw_val;
    env.delta_t_us      = delta_t_us;
    return env;
}

} // anonymous namespace


// =============================================================================
// S1 Calibration tests
// =============================================================================

static void test_s1_nominal()
{
    TEST("S1-01 — Nominal calibration (gain=2.0, offset=100.0)");

    signalfix::StageS1Calibration s1(2.0, 100.0);
    auto cs = make_cs();

    auto env = make_env(0xAA00'0001u, 1'000'000u, 500.0);
    const auto result = s1.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE, "returns CONTINUE");
    CHECK(env.calibrated_value == 500.0 * 2.0 + 100.0,
          "calibrated_value == raw*gain + offset == 1100.0");
    CHECK(!signalfix::has_flag(env.status, signalfix::SampleStatus::HARD_INVALID),
          "no HARD_INVALID on nominal sample");
    CHECK(cs.last_calibrated_valid == true,
          "cs.last_calibrated_valid set to true");
    CHECK(cs.last_calibrated_value == 1100.0,
          "cs.last_calibrated_value == 1100.0");
}

static void test_s1_identity()
{
    TEST("S1-02 — Identity transform (default gain=1.0, offset=0.0)");

    signalfix::StageS1Calibration s1;  // default coefficients
    auto cs = make_cs();
    auto env = make_env(0xAA00'0001u, 1'000'000u, 42.5);
    s1.process(env, cs);

    CHECK(env.calibrated_value == 42.5, "identity: calibrated_value == raw_value");
    CHECK(cs.last_calibrated_value == 42.5, "history updated");
}

static void test_s1_nan_raw()
{
    TEST("S1-03 — NaN raw_value → HARD_INVALID, calibrated stays NaN");

    signalfix::StageS1Calibration s1;
    auto cs = make_cs();
    // Pre-populate history with a valid value
    cs.last_calibrated_value = 99.0;
    cs.last_calibrated_valid  = true;

    auto env = make_env();
    env.raw_value = std::numeric_limits<double>::quiet_NaN();
    const auto result = s1.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE, "returns CONTINUE even on NaN");
    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::HARD_INVALID),
          "HARD_INVALID set on NaN raw_value");
    CHECK(std::isnan(env.calibrated_value),
          "calibrated_value remains NaN");
    CHECK(cs.last_calibrated_valid == false,
          "cs.last_calibrated_valid invalidated (FM-S1-04)");
}

static void test_s1_inf_raw()
{
    TEST("S1-04 — Inf raw_value → HARD_INVALID");

    signalfix::StageS1Calibration s1;
    auto cs = make_cs();
    auto env = make_env();
    env.raw_value = std::numeric_limits<double>::infinity();
    s1.process(env, cs);

    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::HARD_INVALID),
          "HARD_INVALID set on Inf raw_value");
    CHECK(cs.last_calibrated_valid == false,
          "cs.last_calibrated_valid invalidated");
}

static void test_s1_overflow_calibration()
{
    TEST("S1-05 — Calibration output overflow → HARD_INVALID");

    // gain = DBL_MAX, raw = DBL_MAX → product overflows to Inf
    signalfix::StageS1Calibration s1(std::numeric_limits<double>::max(), 0.0);
    auto cs = make_cs();
    auto env = make_env(0xAA00'0001u, 1'000'000u, std::numeric_limits<double>::max());
    s1.process(env, cs);

    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::HARD_INVALID),
          "HARD_INVALID when calibration overflows to Inf");
    CHECK(std::isnan(env.calibrated_value),
          "calibrated_value written as NaN on overflow (S7-gate compatible)");
    CHECK(cs.last_calibrated_valid == false,
          "cs.last_calibrated_valid invalidated on overflow");
}

static void test_s1_existing_hard_invalid()
{
    TEST("S1-06 — Pre-existing HARD_INVALID skips calibration");

    signalfix::StageS1Calibration s1(2.0, 0.0);
    auto cs = make_cs();
    cs.last_calibrated_value = 55.0;
    cs.last_calibrated_valid  = true;

    auto env = make_env(0xAA00'0001u, 1'000'000u, 100.0);
    env.status |= signalfix::SampleStatus::HARD_INVALID;

    s1.process(env, cs);

    CHECK(std::isnan(env.calibrated_value),
          "calibrated_value not written when HARD_INVALID already set");
    CHECK(cs.last_calibrated_valid == false,
          "cs.last_calibrated_valid cleared on pre-existing HARD_INVALID (FM-S1-04)");
}

static void test_s1_preserves_flags()
{
    TEST("S1-07 — Existing status flags preserved on nominal path");

    signalfix::StageS1Calibration s1;
    auto cs = make_cs();
    auto env = make_env(0xAA00'0001u, 1'000'000u, 1.0);
    env.status |= signalfix::SampleStatus::TIMING_ANOMALY;

    s1.process(env, cs);

    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "TIMING_ANOMALY flag preserved by S1 on nominal path");
    CHECK(!signalfix::has_flag(env.status, signalfix::SampleStatus::HARD_INVALID),
          "HARD_INVALID not added spuriously");
}

static void test_s1_100_samples()
{
    TEST("S1-08 — 100 sequential samples, history tracks last valid");

    signalfix::StageS1Calibration s1(1.0, 10.0);  // offset = +10
    auto cs = make_cs();

    bool ok = true;
    for (int i = 0; i < 100; ++i)
    {
        auto env = make_env(0xAA00'0001u, 1'000'000u + static_cast<uint64_t>(i) * 10'000u,
                            static_cast<double>(i));
        s1.process(env, cs);
        const double expected = static_cast<double>(i) + 10.0;
        if (env.calibrated_value != expected) { ok = false; break; }
        if (cs.last_calibrated_value != expected) { ok = false; break; }
    }
    CHECK(ok, "calibrated_value and history correct for 100 sequential samples");
}


// =============================================================================
// S2a Gap Detection tests
// =============================================================================

static void test_s2a_no_gap()
{
    TEST("S2a-01 — Nominal delta_t → no gap detected");

    signalfix::StageS2aGapDetection s2a;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    auto env = make_env(0xAA00'0001u, 1'000'000u, 0.0, 10'000u);
    const auto result = s2a.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE, "returns CONTINUE");
    CHECK(env.gap_event_id == 0u,     "gap_event_id == 0 (no gap)");
    CHECK(cs.gap.gap_pending == false, "gap_pending == false");
    CHECK(cs.gap.consecutive_gap_count == 0u, "consecutive_gap_count reset to 0");
}

static void test_s2a_gap_detected()
{
    TEST("S2a-02 — delta_t > 2.5×nominal → gap detected");

    signalfix::StageS2aGapDetection s2a;
    auto cs = make_cs(0xAA00'0001u, 10'000u);
    // init sets next_gap_event_id = 1

    // delta = 26,000 > 25,000 (2.5×10,000)
    auto env = make_env(0xAA00'0001u, 1'000'000u, 0.0, 26'000u);
    const auto result = s2a.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE, "returns CONTINUE");
    CHECK(env.gap_event_id == 1u,     "gap_event_id == 1 (first gap)");
    CHECK(cs.gap.gap_pending == true,  "gap_pending == true");
    CHECK(cs.gap.consecutive_gap_count == 1u, "consecutive_gap_count == 1");
    CHECK(cs.gap.total_gap_count == 1u,       "total_gap_count == 1");
    CHECK(cs.gap.next_gap_event_id == 2u,     "next_gap_event_id advanced to 2");
}

static void test_s2a_boundary_exact()
{
    TEST("S2a-03 — delta_t == 2.5×nominal (exactly at threshold) → NO gap");

    signalfix::StageS2aGapDetection s2a;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // 2.5×10,000 = 25,000 exactly.  The condition is strictly > threshold.
    auto env = make_env(0xAA00'0001u, 1'000'000u, 0.0, 25'000u);
    s2a.process(env, cs);

    CHECK(env.gap_event_id == 0u,      "gap_event_id == 0 at exact threshold");
    CHECK(cs.gap.gap_pending == false,  "gap_pending == false at exact threshold");
}

static void test_s2a_consecutive_gaps()
{
    TEST("S2a-04 — Consecutive gaps increment counters correctly");

    signalfix::StageS2aGapDetection s2a;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    for (int i = 0; i < 5; ++i)
    {
        auto env = make_env(0xAA00'0001u, 1'000'000u, 0.0, 30'000u);
        s2a.process(env, cs);
    }

    CHECK(cs.gap.consecutive_gap_count == 5u, "consecutive_gap_count == 5");
    CHECK(cs.gap.total_gap_count == 5u,       "total_gap_count == 5");
    CHECK(cs.gap.next_gap_event_id == 6u,     "next_gap_event_id == 6");
}

static void test_s2a_gap_then_recovery()
{
    TEST("S2a-05 — Gap followed by recovery clears consecutive counter");

    signalfix::StageS2aGapDetection s2a;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    { auto env = make_env(0xAA00'0001u, 1'000'000u, 0.0, 30'000u);
      s2a.process(env, cs); }

    CHECK(cs.gap.consecutive_gap_count == 1u, "gap detected");

    { auto env = make_env(0xAA00'0001u, 1'010'000u, 0.0, 10'000u);
      s2a.process(env, cs); }

    CHECK(cs.gap.consecutive_gap_count == 0u,  "consecutive_gap_count reset on recovery");
    CHECK(cs.gap.gap_pending == false,          "gap_pending cleared on recovery");
    CHECK(cs.gap.total_gap_count == 1u,         "total_gap_count unchanged (1)");
}

static void test_s2a_threshold_computation()
{
    TEST("S2a-06 — Gap threshold: 2.5×nominal via integer arithmetic");

    // Verify compute_gap_threshold for several nominally spaced values.
    CHECK(signalfix::StageS2aGapDetection::compute_gap_threshold(10'000u) == 25'000u,
          "threshold(10000) == 25000");
    CHECK(signalfix::StageS2aGapDetection::compute_gap_threshold(4u) == 10u,
          "threshold(4) == 10  (4×5/2)");
    CHECK(signalfix::StageS2aGapDetection::compute_gap_threshold(1u) == 2u,
          "threshold(1) == 2  (1×5/2 = 2, integer)");

    // Overflow case: very large nominal — should not produce a threshold < nominal.
    const uint64_t huge_nominal = UINT64_MAX / 3u;
    const uint64_t thresh       = signalfix::StageS2aGapDetection::compute_gap_threshold(
        huge_nominal);
    CHECK(thresh >= huge_nominal,
          "overflow-safe threshold is >= nominal for huge nominal values");
}


// =============================================================================
// S2b Clock Alignment tests
// =============================================================================

static void test_s2b_bootstrap()
{
    TEST("S2b-01 — First sample bootstraps to arrival_time_us");

    signalfix::StageS2bClockAlignment s2b;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Simulate S0 output: arrival = 5,000,000 μs (system uptime)
    auto env = make_env(0xAA00'0001u, 5'000'000u, 0.0, 10'000u);
    env.arrival_time_us = 5'000'000u;
    cs.gap.gap_pending  = false;

    const auto result = s2b.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE, "returns CONTINUE");
    CHECK(env.timestamp_us == 5'000'000u,
          "first-sample timestamp == arrival_time_us (bootstrap)");
    CHECK(env.jitter_us == 0,
          "jitter == 0 on bootstrap");
    CHECK(cs.last_timestamp_us == 5'000'000u,
          "cs.last_timestamp_us updated on bootstrap");
    CHECK(cs.pll.lock_sample_count == 1u,
          "lock_sample_count == 1 after bootstrap");
    CHECK(!signalfix::has_flag(env.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "no TIMING_ANOMALY on bootstrap");
}

static void test_s2b_nominal_convergence()
{
    TEST("S2b-02 — Nominal samples advance timestamp monotonically");

    signalfix::StageS2bClockAlignment s2b;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    const uint64_t kNominal  = 10'000u;
    const uint64_t kBase     = 5'000'000u;

    uint64_t prev_ts = 0u;
    bool monotonic = true;

    for (int i = 0; i < 20; ++i)
    {
        auto env = make_env(0xAA00'0001u,
                            kBase + static_cast<uint64_t>(i) * kNominal,
                            0.0, kNominal);
        env.arrival_time_us = kBase + static_cast<uint64_t>(i) * kNominal;
        cs.gap.gap_pending = false;

        s2b.process(env, cs);

        if (i > 0 && env.timestamp_us <= prev_ts)
        {
            monotonic = false;
            break;
        }
        prev_ts = env.timestamp_us;
    }
    CHECK(monotonic, "timestamp_us monotonically increases over 20 nominal samples");
}

static void test_s2b_gap_suppresses_pll()
{
    TEST("S2b-03 — Gap path holds PLL state, advances nominally, sets TIMING_ANOMALY");

    signalfix::StageS2bClockAlignment s2b;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Bootstrap first
    { auto env = make_env(0xAA00'0001u, 5'000'000u, 0.0, 10'000u);
      env.arrival_time_us = 5'000'000u; cs.gap.gap_pending = false;
      s2b.process(env, cs); }

    const double saved_phase  = cs.pll.phase_error_us;
    const double saved_freq   = cs.pll.freq_correction_ppm;
    const uint64_t saved_ts   = cs.last_timestamp_us;

    // Gap sample: gap_pending = true
    auto gap_env = make_env(0xAA00'0001u, 5'500'000u, 0.0, 500'000u);
    gap_env.arrival_time_us = 5'500'000u;
    cs.gap.gap_pending = true;

    s2b.process(gap_env, cs);

    CHECK(signalfix::has_flag(gap_env.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "TIMING_ANOMALY set on gap sample");
    CHECK(cs.pll.phase_error_us     == saved_phase,
          "phase_error_us unchanged on gap (PLL held)");
    CHECK(cs.pll.freq_correction_ppm == saved_freq,
          "freq_correction_ppm unchanged on gap (PLL held)");
    // timestamp should advance by nominal from saved_ts
    CHECK(gap_env.timestamp_us == saved_ts + 10'000u,
          "gap-path timestamp = last_ts + nominal");
}

static void test_s2b_pll_locks()
{
    TEST("S2b-04 — PLL locks after kLockThreshold samples");

    signalfix::StageS2bClockAlignment s2b;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    const uint32_t N = signalfix::StageS2bClockAlignment::kLockThreshold;
    const uint64_t kBase = 1'000'000u;

    for (uint32_t i = 0; i < N; ++i)
    {
        auto env = make_env(0xAA00'0001u,
                            kBase + static_cast<uint64_t>(i) * 10'000u,
                            0.0, 10'000u);
        env.arrival_time_us = kBase + static_cast<uint64_t>(i) * 10'000u;
        cs.gap.gap_pending = false;
        s2b.process(env, cs);
    }

    CHECK(cs.pll.locked == true,
          "PLL locked after kLockThreshold samples");
    CHECK(cs.pll.lock_sample_count >= N,
          "lock_sample_count >= kLockThreshold");
}

static void test_s2b_monotonicity_enforced()
{
    TEST("S2b-05 — Monotonicity enforced: timestamp never regresses");

    signalfix::StageS2bClockAlignment s2b;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Manually set up a state that would regress.
    // Bootstrap at t=5,000,000.
    { auto env = make_env(0xAA00'0001u, 5'000'000u, 0.0, 10'000u);
      env.arrival_time_us = 5'000'000u; cs.gap.gap_pending = false;
      s2b.process(env, cs); }

    const uint64_t last_ts = cs.last_timestamp_us;

    // Send a sample that arrives much earlier than expected.
    // t_estimated = last_ts + 10000.  arrival = last_ts - 50000 (early).
    // Phase error = large negative → correction could go negative.
    auto env = make_env(0xAA00'0001u, last_ts - 50'000u, 0.0, 10'000u);
    if (last_ts >= 50'000u)
    {
        env.arrival_time_us = last_ts - 50'000u;
    }
    else
    {
        env.arrival_time_us = 1u;
    }
    cs.gap.gap_pending = false;
    s2b.process(env, cs);

    CHECK(env.timestamp_us >= last_ts,
          "timestamp_us never regresses below previous value");
}

static void test_s2b_jitter_clamped()
{
    TEST("S2b-06 — jitter_us clamped to int32_t range");

    signalfix::StageS2bClockAlignment s2b;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Bootstrap.
    { auto env = make_env(0xAA00'0001u, 1'000'000u, 0.0, 10'000u);
      env.arrival_time_us = 1'000'000u; cs.gap.gap_pending = false;
      s2b.process(env, cs); }

    // Next sample: arrival much much later (huge positive difference).
    // After bootstrap last_ts = 1,000,000.
    // t_estimated = 1,010,000.
    // arrival = 1,000,000 + 3 billion → huge positive jitter.
    const uint64_t huge_arrival = 1'000'000u + 3'000'000'000u;
    auto env = make_env(0xAA00'0001u, huge_arrival, 0.0, 10'000u);
    env.arrival_time_us = huge_arrival;
    cs.gap.gap_pending  = false;
    s2b.process(env, cs);

    // jitter_us is int32_t — must not have UB.
    // We just verify it equals INT32_MAX (saturated) or some large value.
    CHECK(env.jitter_us == INT32_MAX,
          "jitter_us saturated at INT32_MAX for extreme positive jitter");
}

static void test_s2b_freq_correction_clamped()
{
    TEST("S2b-07 — freq_correction_ppm clamped to prevent divergence");

    signalfix::StageS2bClockAlignment s2b;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Bootstrap + reach locked state.
    const uint32_t N = signalfix::StageS2bClockAlignment::kLockThreshold;
    const uint64_t kBase = 1'000'000u;
    for (uint32_t i = 0; i < N; ++i)
    {
        auto env = make_env(0xAA00'0001u, kBase + static_cast<uint64_t>(i) * 10'000u,
                            0.0, 10'000u);
        env.arrival_time_us = kBase + static_cast<uint64_t>(i) * 10'000u;
        cs.gap.gap_pending  = false;
        s2b.process(env, cs);
    }

    // Inject 1000 samples with a large consistent phase error.
    // Each iteration: phi_err is large; K_f × phi_err accumulates.
    // Without clamping, freq_correction would diverge to Inf.
    for (int i = 0; i < 1000; ++i)
    {
        const uint64_t arrival = kBase + static_cast<uint64_t>(N + i) * 10'000u
                                 + 5'000u;  // +5000 μs late every sample
        auto env = make_env(0xAA00'0001u, arrival, 0.0, 10'000u);
        env.arrival_time_us = arrival;
        cs.gap.gap_pending  = false;
        s2b.process(env, cs);
    }

    const bool corr_finite =
        !std::isnan(cs.pll.freq_correction_ppm) &&
        !std::isinf(cs.pll.freq_correction_ppm);

    CHECK(corr_finite, "freq_correction_ppm remains finite after 1000 biased samples");
    CHECK(cs.last_timestamp_us > 0u, "last_timestamp_us is valid (not corrupted)");
}


// =============================================================================
// main
// =============================================================================

int main()
{
    std::fprintf(stdout,
        "============================================================\n"
        "  SignalFix AI — S1 / S2a / S2b — Integration Test Harness\n"
        "  Spec: SFX-M1-TDS-001 Rev 2.1\n"
        "============================================================\n");

    // S1
    test_s1_nominal();
    test_s1_identity();
    test_s1_nan_raw();
    test_s1_inf_raw();
    test_s1_overflow_calibration();
    test_s1_existing_hard_invalid();
    test_s1_preserves_flags();
    test_s1_100_samples();

    // S2a
    test_s2a_no_gap();
    test_s2a_gap_detected();
    test_s2a_boundary_exact();
    test_s2a_consecutive_gaps();
    test_s2a_gap_then_recovery();
    test_s2a_threshold_computation();

    // S2b
    test_s2b_bootstrap();
    test_s2b_nominal_convergence();
    test_s2b_gap_suppresses_pll();
    test_s2b_pll_locks();
    test_s2b_monotonicity_enforced();
    test_s2b_jitter_clamped();
    test_s2b_freq_correction_clamped();

    std::fprintf(stdout,
        "\n============================================================\n"
        "  TEST SUMMARY\n"
        "  Pass: %d    Fail: %d\n"
        "  Result: %s\n"
        "============================================================\n",
        g_pass, g_fail,
        (g_fail == 0) ? "PASS" : "FAIL");

    return (g_fail == 0) ? 0 : 1;
}
