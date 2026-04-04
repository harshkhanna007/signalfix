// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : tests/test_stage_s0.cpp
// Stage  : S0 — Input Adapter
// =============================================================================
//
// Deterministic test harness for StageS0InputAdapter.
//
// Tests:
//   T01  — Sequence ordering: 200 sequential samples, verify monotonicity.
//   T02  — delta_t correctness: uniform sample cadence, verify delta_t == nominal.
//   T03  — First-sample delta_t: delta_t set to nominal on first call.
//   T04  — Signed timestamp arithmetic: arrival regression → TIMING_ANOMALY.
//   T05  — Watchdog arm: after first sample watchdog_armed == true.
//   T06  — Watchdog deadline: deadline = last_arrival + 2×nominal.
//   T07  — Watchdog timeout injection via build_stale_envelope():
//             sequence advances, STALE|MISSING set, value fields NaN,
//             nominal_streak reset.
//   T08  — Channel mismatch: ABORT_FAULT returned, sequence_counter unchanged.
//   T09  — calibration_version propagation.
//   T10  — Nominal streak NOT modified by S0::process() (owned by S6).
//   T11  — TIMING_ANOMALY with nominal fallback: delta_t never 0.
//   T12  — Watchdog deadline saturation guard (near UINT64_MAX).
//   T13  — Sequence gap after watchdog: M2-visible sequence id gap.
//   T14  — Consecutive watchdog injections: sequence_counter increments,
//             deadline advances, nominal_streak stays 0.
//   T15  — Multiple channels remain isolated (independent ChannelStates).
//
// Compile:
//   g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic \
//       -fno-exceptions -fno-rtti \
//       -I include \
//       -o test_s0 \
//       tests/test_stage_s0.cpp \
//       src/module1/stages/stage_s0_input_adapter.cpp
//
// Expected output: all lines "PASS".
//
// =============================================================================

#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/stages/stage_s0_input_adapter.hpp"

#include <cstdio>
#include <cstdint>
#include <cmath>
#include <limits>
#include <cstring>

// =============================================================================
// Test framework — minimal, no heap, no exceptions
// =============================================================================

namespace {

static int g_pass_count = 0;
static int g_fail_count = 0;

#define CHECK(cond, msg)                                          \
    do {                                                           \
        if (cond) {                                               \
            std::fprintf(stdout, "  [PASS] %s\n", msg);          \
            ++g_pass_count;                                       \
        } else {                                                   \
            std::fprintf(stdout, "  [FAIL] %s  (line %d)\n",     \
                         msg, __LINE__);                          \
            ++g_fail_count;                                       \
        }                                                          \
    } while (false)

#define TEST(name) std::fprintf(stdout, "\n--- %s ---\n", name)

// ── Helpers ──────────────────────────────────────────────────────────────────

signalfix::ChannelState make_cs(
    uint32_t channel_id         = 0xAA00'0001u,
    uint64_t nominal_delta_t_us = 10'000u,   // 100 Hz
    uint32_t cal_version        = 7u,
    uint16_t roc_window         = 30u)
{
    signalfix::ChannelState cs{};
    signalfix::init_channel_state(cs,
                                   channel_id,
                                   cal_version,
                                   nominal_delta_t_us,
                                   roc_window);
    return cs;
}

signalfix::MeasurementEnvelope make_env(
    uint32_t channel_id     = 0xAA00'0001u,
    uint64_t arrival_us     = 1'000'000u,
    double   raw_val        = 2048.0)
{
    signalfix::MeasurementEnvelope env = signalfix::make_nominal_envelope();
    env.channel_id      = channel_id;
    env.arrival_time_us = arrival_us;
    env.raw_value       = raw_val;
    return env;
}

bool is_nan_double(double v) noexcept { return std::isnan(v); }
bool is_nan_float(float v)   noexcept { return std::isnan(v); }

} // anonymous namespace


// =============================================================================
// Tests
// =============================================================================

// ---------------------------------------------------------------------------
// T01 — Sequence ordering over 200 sequential samples
// ---------------------------------------------------------------------------
static void test_t01_sequence_ordering()
{
    TEST("T01 — Sequence ordering (200 samples)");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs();

    static constexpr int     kN         = 200;
    static constexpr uint64_t kNominal  = 10'000u;
    static constexpr uint64_t kBaseTime = 1'000'000u;

    uint64_t prev_seq = UINT64_MAX;
    bool     order_ok = true;

    for (int i = 0; i < kN; ++i)
    {
        auto env = make_env(0xAA00'0001u, kBaseTime + static_cast<uint64_t>(i) * kNominal);
        const auto result = s0.process(env, cs);

        if (result != signalfix::StageResult::CONTINUE) { order_ok = false; break; }

        if (i == 0)
        {
            // First sample; just record
            prev_seq = env.sequence_id;
        }
        else
        {
            if (env.sequence_id != prev_seq + 1u) { order_ok = false; break; }
            prev_seq = env.sequence_id;
        }
    }

    CHECK(order_ok,      "sequence_id increments by 1 each sample for 200 samples");
    CHECK(prev_seq == 199u, "final sequence_id == 199 (0-based, 200 samples)");
    CHECK(cs.sequence_counter == 200u, "cs.sequence_counter == 200 after 200 samples");
}


// ---------------------------------------------------------------------------
// T02 — delta_t correctness at uniform cadence
// ---------------------------------------------------------------------------
static void test_t02_delta_t()
{
    TEST("T02 — delta_t correctness at uniform cadence");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    static constexpr uint64_t kNominal  = 10'000u;
    static constexpr uint64_t kBaseTime = 5'000'000u;
    static constexpr int      kN        = 100;

    bool delta_ok = true;

    for (int i = 0; i < kN; ++i)
    {
        auto env = make_env(0xAA00'0001u, kBaseTime + static_cast<uint64_t>(i) * kNominal);
        s0.process(env, cs);

        if (i == 0)
        {
            // First sample: delta = nominal
            if (env.delta_t_us != kNominal) { delta_ok = false; break; }
        }
        else
        {
            if (env.delta_t_us != kNominal) { delta_ok = false; break; }
            if (signalfix::has_flag(env.status, signalfix::SampleStatus::TIMING_ANOMALY))
            { delta_ok = false; break; }
        }
    }

    CHECK(delta_ok, "delta_t_us == nominal for every sample at uniform cadence");
}


// ---------------------------------------------------------------------------
// T03 — First-sample delta_t = nominal
// ---------------------------------------------------------------------------
static void test_t03_first_sample()
{
    TEST("T03 — First-sample delta_t defaults to nominal");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // arrival_time_us is large (system uptime), NOT a small delta
    auto env = make_env(0xAA00'0001u, 999'000'000u);
    const auto result = s0.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE, "process returns CONTINUE");
    CHECK(env.delta_t_us == 10'000u, "delta_t_us == nominal on first sample");
    CHECK(!signalfix::has_flag(env.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "no TIMING_ANOMALY on first sample");
    CHECK(cs.watchdog_armed == true, "watchdog armed after first sample");
}


// ---------------------------------------------------------------------------
// T04 — Timestamp regression → TIMING_ANOMALY, delta_t == nominal
// ---------------------------------------------------------------------------
static void test_t04_timestamp_regression()
{
    TEST("T04 — Timestamp regression → TIMING_ANOMALY, delta fallback to nominal");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Establish last_arrival_us
    {
        auto env = make_env(0xAA00'0001u, 1'000'000u);
        s0.process(env, cs);
    }

    // Now send a sample with an EARLIER timestamp (regression)
    auto env = make_env(0xAA00'0001u, 999'990u);  // 10 μs BEFORE prior arrival
    const auto result = s0.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE,
          "process returns CONTINUE on regression (not a fault)");
    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "TIMING_ANOMALY set on timestamp regression");
    CHECK(env.delta_t_us == 10'000u,
          "delta_t_us falls back to nominal on regression");
    CHECK(env.delta_t_us != 0u,
          "delta_t_us is never zero (no downstream division-by-zero)");
}


// ---------------------------------------------------------------------------
// T05 — Watchdog arm state
// ---------------------------------------------------------------------------
static void test_t05_watchdog_arm()
{
    TEST("T05 — Watchdog arm state");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    CHECK(cs.watchdog_armed == false, "watchdog not armed before first sample");

    auto env = make_env(0xAA00'0001u, 1'000'000u);
    s0.process(env, cs);

    CHECK(cs.watchdog_armed == true, "watchdog armed after first sample");
}


// ---------------------------------------------------------------------------
// T06 — Watchdog deadline = last_arrival + 2×nominal
// ---------------------------------------------------------------------------
static void test_t06_watchdog_deadline()
{
    TEST("T06 — Watchdog deadline arithmetic");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    const uint64_t arrival = 5'000'000u;
    auto env = make_env(0xAA00'0001u, arrival);
    s0.process(env, cs);

    const uint64_t expected_deadline = arrival + 2u * 10'000u;
    CHECK(cs.watchdog_deadline_us == expected_deadline,
          "watchdog_deadline_us == arrival_time + 2×nominal_delta_t");

    // Second sample — deadline must update to the latest arrival
    const uint64_t arrival2 = 5'010'000u;
    auto env2 = make_env(0xAA00'0001u, arrival2);
    s0.process(env2, cs);
    const uint64_t expected2 = arrival2 + 2u * 10'000u;
    CHECK(cs.watchdog_deadline_us == expected2,
          "watchdog_deadline_us updated to latest arrival after each sample");
}


// ---------------------------------------------------------------------------
// T07 — build_stale_envelope(): STALE|MISSING, NaN values, seq advance, streak reset
// ---------------------------------------------------------------------------
static void test_t07_stale_envelope()
{
    TEST("T07 — build_stale_envelope() produces correct STALE|MISSING envelope");

    auto cs = make_cs(0xAA00'0001u, 10'000u);
    cs.nominal_streak    = 42u;   // simulate a healthy streak before stall
    cs.sequence_counter  = 77u;   // known starting point
    cs.calibration_version = 9u;

    auto env = signalfix::make_nominal_envelope();
    const uint64_t now = 9'000'000u;

    signalfix::StageS0InputAdapter::build_stale_envelope(env, cs, now);

    CHECK(env.sequence_id == 77u,
          "STALE envelope carries pre-increment sequence_id 77");
    CHECK(cs.sequence_counter == 78u,
          "cs.sequence_counter incremented to 78 after stale injection");

    CHECK(env.channel_id == 0xAA00'0001u,
          "channel_id correct in stale envelope");

    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::STALE),
          "STALE flag set");
    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::MISSING),
          "MISSING flag set (always paired with STALE)");

    CHECK(env.measurement_trust_tier == signalfix::MeasurementTrustTier::REJECTED,
          "trust tier REJECTED on STALE envelope");

    CHECK(is_nan_double(env.raw_value),        "raw_value is NaN (no measurement data)");
    CHECK(is_nan_double(env.calibrated_value), "calibrated_value is NaN");
    CHECK(is_nan_double(env.validated_value),  "validated_value is NaN");
    CHECK(is_nan_double(env.filtered_value),   "filtered_value is NaN");
    CHECK(is_nan_float(env.roc),               "roc is NaN");
    CHECK(is_nan_float(env.roc_adaptive_limit), "roc_adaptive_limit is NaN");

    CHECK(env.nominal_streak_count == 42u,
          "nominal_streak_count carries pre-stall streak (42) for M2 diagnostics");
    CHECK(cs.nominal_streak == 0u,
          "cs.nominal_streak reset to 0 after stale injection");

    CHECK(env.delta_t_us == 2u * 10'000u,
          "delta_t_us == 2 × nominal on watchdog envelope");

    const uint64_t expected_deadline = now + 2u * 10'000u;
    CHECK(cs.watchdog_deadline_us == expected_deadline,
          "watchdog deadline advanced after stale injection");

    CHECK(env.calibration_version == 9u,
          "calibration_version copied from ChannelState");

    CHECK(env.roc_window_n == 0u,
          "roc_window_n == 0 (no ROC on synthetic sample)");

    // Validate status flags pass INV checks
    CHECK(signalfix::validate_status_flags(env.status),
          "STALE|MISSING combination passes validate_status_flags()");
}


// ---------------------------------------------------------------------------
// T08 — Channel mismatch → ABORT_FAULT, no side effects
// ---------------------------------------------------------------------------
static void test_t08_channel_mismatch()
{
    TEST("T08 — Channel mismatch → ABORT_FAULT");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Envelope claims a different channel_id
    auto env = make_env(0xBB00'9999u, 1'000'000u);

    const uint64_t seq_before = cs.sequence_counter;

    const auto result = s0.process(env, cs);

    CHECK(result == signalfix::StageResult::ABORT_FAULT,
          "ABORT_FAULT returned on channel_id mismatch");
    CHECK(cs.sequence_counter == seq_before,
          "sequence_counter NOT incremented on fault (no seq id consumed)");
    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::HARD_INVALID),
          "HARD_INVALID set in envelope on mismatch fault");
}


// ---------------------------------------------------------------------------
// T09 — calibration_version propagation
// ---------------------------------------------------------------------------
static void test_t09_cal_version()
{
    TEST("T09 — calibration_version propagated into envelope");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u, /*cal_version=*/42u);

    auto env = make_env(0xAA00'0001u, 1'000'000u);
    s0.process(env, cs);

    CHECK(env.calibration_version == 42u,
          "calibration_version == 42 (from ChannelState)");
}


// ---------------------------------------------------------------------------
// T10 — S0::process() does NOT modify cs.nominal_streak (owned by S6)
// ---------------------------------------------------------------------------
static void test_t10_streak_not_modified()
{
    TEST("T10 — S0::process() does not modify cs.nominal_streak");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs();
    cs.nominal_streak = 55u;

    for (int i = 0; i < 10; ++i)
    {
        auto env = make_env(0xAA00'0001u, 1'000'000u + static_cast<uint64_t>(i) * 10'000u);
        s0.process(env, cs);
    }

    CHECK(cs.nominal_streak == 55u,
          "cs.nominal_streak unchanged after 10 process() calls (S6 owns it)");
}


// ---------------------------------------------------------------------------
// T11 — TIMING_ANOMALY ensures delta_t_us != 0
// ---------------------------------------------------------------------------
static void test_t11_timing_anomaly_non_zero_delta()
{
    TEST("T11 — delta_t_us != 0 even when TIMING_ANOMALY is set");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 5'000u);

    // Establish baseline
    { auto env = make_env(0xAA00'0001u, 1'000'000u); s0.process(env, cs); }

    // Send identical timestamp (zero delta)
    auto env = make_env(0xAA00'0001u, 1'000'000u);
    s0.process(env, cs);

    CHECK(env.delta_t_us != 0u,
          "delta_t_us is non-zero even on zero-delta regression");
    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "TIMING_ANOMALY set for zero-delta sample");
}


// ---------------------------------------------------------------------------
// T12 — Watchdog deadline saturation near UINT64_MAX
// ---------------------------------------------------------------------------
static void test_t12_deadline_saturation()
{
    TEST("T12 — Watchdog deadline saturates at UINT64_MAX");

    signalfix::StageS0InputAdapter s0;
    // nominal_delta_t_us large enough that arrival + 2×nominal overflows
    // Use a very large nominal; with arrival near UINT64_MAX this wraps.
    auto cs = make_cs(0xAA00'0001u, 0x7FFF'FFFF'FFFF'FFFFu);

    // arrival near UINT64_MAX
    const uint64_t huge_arrival = UINT64_MAX - 1u;
    auto env = make_env(0xAA00'0001u, huge_arrival);
    const auto result = s0.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE,
          "process returns CONTINUE even with near-overflow arrival time");
    CHECK(cs.watchdog_deadline_us == UINT64_MAX,
          "watchdog_deadline_us saturated to UINT64_MAX (no overflow)");
}


// ---------------------------------------------------------------------------
// T13 — Sequence gap visible to M2 after watchdog stale injection
// ---------------------------------------------------------------------------
static void test_t13_sequence_gap_after_watchdog()
{
    TEST("T13 — Sequence gap visible after watchdog injection");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Process 10 normal samples
    uint64_t last_normal_seq = 0u;
    for (int i = 0; i < 10; ++i)
    {
        auto env = make_env(0xAA00'0001u, 1'000'000u + static_cast<uint64_t>(i) * 10'000u);
        s0.process(env, cs);
        last_normal_seq = env.sequence_id;
    }
    CHECK(last_normal_seq == 9u, "last normal sample has seq_id == 9");

    // Inject 3 stale envelopes (sensor gone for 3 watchdog periods)
    for (int i = 0; i < 3; ++i)
    {
        auto stale_env = signalfix::make_nominal_envelope();
        signalfix::StageS0InputAdapter::build_stale_envelope(
            stale_env, cs, 1'200'000u + static_cast<uint64_t>(i) * 20'000u);
    }
    CHECK(cs.sequence_counter == 13u,
          "sequence_counter == 13 after 10 normal + 3 stale envelopes");

    // Resume normal samples
    auto env = make_env(0xAA00'0001u, 1'300'000u);
    s0.process(env, cs);
    CHECK(env.sequence_id == 13u,
          "first resumed sample has seq_id 13 (gap of 3 visible to M2)");
}


// ---------------------------------------------------------------------------
// T14 — Consecutive watchdog injections: seq advances, streak stays 0
// ---------------------------------------------------------------------------
static void test_t14_consecutive_watchdog()
{
    TEST("T14 — Consecutive watchdog injections");

    auto cs = make_cs(0xAA00'0001u, 10'000u);
    cs.sequence_counter = 100u;
    cs.nominal_streak   = 99u;

    uint64_t prev_deadline = 0u;
    bool     seqs_ok       = true;
    bool     streak_ok     = true;

    for (int i = 0; i < 5; ++i)
    {
        auto env = signalfix::make_nominal_envelope();
        const uint64_t t = 2'000'000u + static_cast<uint64_t>(i) * 20'000u;
        signalfix::StageS0InputAdapter::build_stale_envelope(env, cs, t);

        if (env.sequence_id != static_cast<uint64_t>(100 + i)) seqs_ok = false;
        if (cs.nominal_streak != 0u)                             streak_ok = false;
        if (cs.watchdog_deadline_us <= prev_deadline)            seqs_ok = false;

        prev_deadline = cs.watchdog_deadline_us;
    }

    CHECK(seqs_ok,   "sequence_id increments on every consecutive watchdog injection");
    CHECK(streak_ok, "nominal_streak remains 0 throughout consecutive injections");
    CHECK(cs.sequence_counter == 105u,
          "sequence_counter == 105 after 5 consecutive stale injections");
}


// ---------------------------------------------------------------------------
// T15 — Two independent channels remain isolated
// ---------------------------------------------------------------------------
static void test_t15_channel_isolation()
{
    TEST("T15 — Channel state isolation between two channels");

    signalfix::StageS0InputAdapter s0;
    auto cs_a = make_cs(0xAA00'0001u, 10'000u, 1u);
    auto cs_b = make_cs(0xBB00'0002u, 5'000u,  2u);

    // Pump channel A for 50 samples
    for (int i = 0; i < 50; ++i)
    {
        auto env = make_env(0xAA00'0001u, 1'000'000u + static_cast<uint64_t>(i) * 10'000u);
        s0.process(env, cs_a);
    }

    // Pump channel B for 30 samples
    for (int i = 0; i < 30; ++i)
    {
        auto env = make_env(0xBB00'0002u, 2'000'000u + static_cast<uint64_t>(i) * 5'000u);
        s0.process(env, cs_b);
    }

    CHECK(cs_a.sequence_counter == 50u,
          "channel A sequence_counter == 50 (unaffected by channel B processing)");
    CHECK(cs_b.sequence_counter == 30u,
          "channel B sequence_counter == 30 (independent from channel A)");
    CHECK(cs_a.calibration_version == 1u,
          "channel A calibration_version unchanged");
    CHECK(cs_b.calibration_version == 2u,
          "channel B calibration_version unchanged");
    CHECK(cs_a.nominal_delta_t_us == 10'000u,
          "channel A nominal_delta_t_us unchanged");
    CHECK(cs_b.nominal_delta_t_us == 5'000u,
          "channel B nominal_delta_t_us unchanged");
}


// ---------------------------------------------------------------------------
// T16 — FIX-2/5: delta_t > 10×nominal → clamped to nominal + TIMING_ANOMALY
// ---------------------------------------------------------------------------
static void test_t16_delta_clamp()
{
    TEST("T16 — Extreme delta_t clamped to nominal (FIX-2/FIX-5)");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Establish baseline arrival at t=1,000,000
    { auto env = make_env(0xAA00'0001u, 1'000'000u); s0.process(env, cs); }

    // Arrive 200× nominal later — far beyond the 10× threshold.
    // This simulates a sensor that restarts after a long absence but
    // somehow bypassed the watchdog.
    const uint64_t extreme_arrival = 1'000'000u + 200u * 10'000u;
    auto env = make_env(0xAA00'0001u, extreme_arrival);
    const auto result = s0.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE,
          "process returns CONTINUE on extreme delta (not a fault)");
    CHECK(signalfix::has_flag(env.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "TIMING_ANOMALY set when delta > 10×nominal");
    CHECK(env.delta_t_us == 10'000u,
          "delta_t_us clamped to nominal (not the raw 200×nominal interval)");
    CHECK(env.delta_t_us != 0u,
          "delta_t_us is never zero after clamp");

    // Boundary: exactly 10×nominal should NOT trigger the clamp.
    { auto env2 = make_env(0xAA00'0001u, extreme_arrival + 200u * 10'000u);
      s0.process(env2, cs); }  // advance last_arrival

    const uint64_t boundary_arrival = cs.last_arrival_us + 10u * 10'000u;
    auto env_boundary = make_env(0xAA00'0001u, boundary_arrival);
    s0.process(env_boundary, cs);

    CHECK(!signalfix::has_flag(env_boundary.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "delta == 10×nominal is NOT clamped (boundary is inclusive on valid side)");
    CHECK(env_boundary.delta_t_us == 10u * 10'000u,
          "delta_t_us == 10×nominal passes through unmodified at boundary");

    // One step over: 10×nominal + 1 μs triggers the clamp.
    const uint64_t over_arrival = cs.last_arrival_us + 10u * 10'000u + 1u;
    auto env_over = make_env(0xAA00'0001u, over_arrival);
    s0.process(env_over, cs);

    CHECK(signalfix::has_flag(env_over.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "delta == 10×nominal+1 triggers TIMING_ANOMALY (just over boundary)");
    CHECK(env_over.delta_t_us == 10'000u,
          "delta_t_us clamped to nominal at 10×nominal+1");
}


// ---------------------------------------------------------------------------
// T17 — FIX-4: watchdog deadline saturation when 2×nominal overflows
// ---------------------------------------------------------------------------
static void test_t17_timeout_multiplication_saturation()
{
    TEST("T17 — Watchdog timeout multiplication saturates (FIX-4)");

    signalfix::StageS0InputAdapter s0;
    // nominal near UINT64_MAX/2 so that 2×nominal overflows uint64_t.
    const uint64_t huge_nominal = (UINT64_MAX / 2u) + 1u;
    auto cs = make_cs(0xAA00'0001u, huge_nominal);

    const uint64_t arrival = 1'000'000u;
    auto env = make_env(0xAA00'0001u, arrival);
    const auto result = s0.process(env, cs);

    CHECK(result == signalfix::StageResult::CONTINUE,
          "process returns CONTINUE with huge nominal_delta_t_us");
    // 2×nominal overflows; timeout must saturate to UINT64_MAX.
    // arrival (1,000,000) < UINT64_MAX - UINT64_MAX = 0 is false,
    // so deadline = UINT64_MAX.
    CHECK(cs.watchdog_deadline_us == UINT64_MAX,
          "watchdog_deadline_us saturated to UINT64_MAX when 2×nominal overflows");

    // Also verify build_stale_envelope applies the same saturation.
    auto stale_env = signalfix::make_nominal_envelope();
    signalfix::StageS0InputAdapter::build_stale_envelope(stale_env, cs, 2'000'000u);
    CHECK(cs.watchdog_deadline_us == UINT64_MAX,
          "build_stale_envelope also saturates deadline when 2×nominal overflows");
    // delta_t_us in the stale envelope is also capped.
    CHECK(stale_env.delta_t_us == UINT64_MAX,
          "stale envelope delta_t_us saturated to UINT64_MAX when 2×nominal overflows");
}


// ---------------------------------------------------------------------------
// T18 — FIX-1: pure unsigned regression detection matches signed behavior
// ---------------------------------------------------------------------------
static void test_t18_unsigned_regression_detection()
{
    TEST("T18 — Pure unsigned regression detection (FIX-1)");

    signalfix::StageS0InputAdapter s0;
    auto cs = make_cs(0xAA00'0001u, 10'000u);

    // Establish last_arrival at a large value that would have been sign-bit-set
    // if misinterpreted as int64_t (> 2^63).
    const uint64_t large_base = 0x8000'0000'0000'0000ULL;  // 2^63, the int64 sign bit
    { auto env = make_env(0xAA00'0001u, large_base); s0.process(env, cs); }

    // A sample arriving before large_base — regression.
    auto env_reg = make_env(0xAA00'0001u, large_base - 1u);
    s0.process(env_reg, cs);
    CHECK(signalfix::has_flag(env_reg.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "TIMING_ANOMALY set when arrival < last_arrival at 2^63 boundary");
    CHECK(env_reg.delta_t_us == 10'000u,
          "delta_t_us falls back to nominal on regression at 2^63 boundary");

    // A sample arriving after last_arrival_us by exactly nominal — no anomaly.
    // After the regression, cs.last_arrival_us was updated to (large_base - 1).
    // Depart from there so the delta is exactly nominal.
    const uint64_t norm_arrival = (large_base - 1u) + 10'000u;
    auto env_norm = make_env(0xAA00'0001u, norm_arrival);
    s0.process(env_norm, cs);
    CHECK(!signalfix::has_flag(env_norm.status, signalfix::SampleStatus::TIMING_ANOMALY),
          "no TIMING_ANOMALY for normal arrival at 2^63 boundary");
    CHECK(env_norm.delta_t_us == 10'000u,
          "delta_t_us correct for normal arrival at 2^63 boundary");
}


// ---------------------------------------------------------------------------
// Summary
// ---------------------------------------------------------------------------
static void print_summary()
{
    std::fprintf(stdout,
        "\n============================================================\n"
        "  TEST SUMMARY\n"
        "  Pass: %d    Fail: %d\n"
        "  Result: %s\n"
        "============================================================\n",
        g_pass_count, g_fail_count,
        (g_fail_count == 0) ? "PASS" : "FAIL");
}


// =============================================================================
// main
// =============================================================================

int main()
{
    std::fprintf(stdout,
        "============================================================\n"
        "  SignalFix AI — Stage S0 Input Adapter — Test Harness\n"
        "  Spec: SFX-M1-TDS-001 Rev 2.1\n"
        "============================================================\n");

    test_t01_sequence_ordering();
    test_t02_delta_t();
    test_t03_first_sample();
    test_t04_timestamp_regression();
    test_t05_watchdog_arm();
    test_t06_watchdog_deadline();
    test_t07_stale_envelope();
    test_t08_channel_mismatch();
    test_t09_cal_version();
    test_t10_streak_not_modified();
    test_t11_timing_anomaly_non_zero_delta();
    test_t12_deadline_saturation();
    test_t13_sequence_gap_after_watchdog();
    test_t14_consecutive_watchdog();
    test_t15_channel_isolation();
    test_t16_delta_clamp();
    test_t17_timeout_multiplication_saturation();
    test_t18_unsigned_regression_detection();

    print_summary();

    return (g_fail_count == 0) ? 0 : 1;
}
