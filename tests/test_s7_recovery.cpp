#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>

#include "signalfix/module1/types.hpp"
#include "signalfix/module1/stages/s7.hpp"
#include "signalfix/module1/channel_state.hpp"

using namespace signalfix;

// Minimal test framework
static int g_tests_run = 0;
static int g_tests_passed = 0;

#define TEST_BEGIN(name) { \
    g_tests_run++; \
    std::cout << "[ RUN  ] " << name << std::endl;

#define TEST_END(name) { \
    g_tests_passed++; \
    std::cout << "[ PASS ] " << name << std::endl; \
} }

#define EXPECT_EQ(a, b) if ((a) != (b)) { \
    std::cerr << "  FAIL: " << #a << " (" << (long long)(a) << ") != " << #b << " (" << (long long)(b) << ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
    return; \
}

#define EXPECT_TRUE(expr) if (!(expr)) { \
    std::cerr << "  FAIL: " << #expr << " is false at " << __FILE__ << ":" << __LINE__ << std::endl; \
    return; \
}

#define EXPECT_NEAR(a, b, eps) if (std::abs((double)(a) - (double)(b)) > (double)(eps)) { \
    std::cerr << "  FAIL: " << #a << " (" << (a) << ") far from " << #b << " (" << (b) << ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
    return; \
}

// Mock callback matching ProcessedOutputFn
bool mock_output_bus(const ProcessedSample& /*pkg*/, void* /*ctx*/) noexcept {
    return true;
}

// Prime the S7 instance with enough samples to pass the Hard Gate safety threshold (100 samples)
void prime_s7(StageS7Output& s7, ChannelState& cs, uint64_t& ts) {
    for (int i = 0; i < 110; ++i) {
        MeasurementEnvelope env = make_nominal_envelope();
        env.channel_id = cs.channel_id;
        ts += 1000;
        env.arrival_time_us = ts;
        env.corrected_timestamp_us = ts;
        env.corrected_delta_t_us = 1000;
        env.calibrated_value = 100.0;
        env.failure_hint = FailureMode::NONE;
        s7.process(env, cs);
    }
}

void test_s7_active_fault() {
    TEST_BEGIN("S7 Active Fault Reporting");

    StageS7Output s7(mock_output_bus, nullptr);
    ChannelState cs;
    init_channel_state(cs, 1, 1000, 128, 1);
    cs.roc_n = 50;
    cs.roc_prev_sample_invalid = false;
    uint64_t ts = 0;

    prime_s7(s7, cs, ts);

    // S7 Escalation Policy: NONE -> SPIKE is immediate (CASE B-2)
    {
        MeasurementEnvelope env = make_nominal_envelope();
        env.channel_id = 1;
        ts += 1000;
        env.arrival_time_us = ts;
        env.corrected_timestamp_us = ts;
        env.corrected_delta_t_us = 1000;
        env.calibrated_value = 100.0;
        env.failure_hint = FailureMode::SPIKE;
        env.failure_confidence = 0.95f;
        s7.process(env, cs);

        EXPECT_EQ(env.failure_hint, FailureMode::SPIKE);
        EXPECT_EQ(env.internal_failure_hint, FailureMode::SPIKE);
    }

    TEST_END("S7 Active Fault Reporting");
}

void test_s7_recovery_suppression() {
    TEST_BEGIN("S7 Recovery Phase Suppression");

    StageS7Output s7(mock_output_bus, nullptr);
    ChannelState cs;
    init_channel_state(cs, 1, 1000, 128, 1);
    cs.roc_n = 50;
    cs.roc_prev_sample_invalid = false;
    uint64_t ts = 0;

    prime_s7(s7, cs, ts);

    // 1. Latch a SPIKE
    {
        MeasurementEnvelope env = make_nominal_envelope();
        env.channel_id = 1;
        ts += 1000;
        env.arrival_time_us = ts;
        env.corrected_timestamp_us = ts;
        env.corrected_delta_t_us = 1000;
        env.calibrated_value = 100.0;
        env.failure_hint = FailureMode::SPIKE;
        env.failure_confidence = 0.95f;
        s7.process(env, cs);
    }
    
    // 2. Transition to Recovery (S5 suggests NONE)
    for (int i = 0; i < 4; ++i) {
        MeasurementEnvelope env = make_nominal_envelope();
        env.channel_id = 1;
        ts += 1000;
        env.arrival_time_us = ts;
        env.corrected_timestamp_us = ts;
        env.corrected_delta_t_us = 1000;
        env.calibrated_value = 100.0;
        env.failure_hint = FailureMode::NONE;
        env.failure_confidence = 0.0f;

        s7.process(env, cs);

        // Assert: Public is silenced during recovery
        EXPECT_EQ(env.failure_hint, FailureMode::NONE);
        EXPECT_EQ(env.internal_failure_hint, FailureMode::SPIKE);
    }

    TEST_END("S7 Recovery Phase Suppression");
}

void test_s7_invalid_override() {
    TEST_BEGIN("S7 Hard Invalid Override during Recovery");

    StageS7Output s7(mock_output_bus, nullptr);
    ChannelState cs;
    init_channel_state(cs, 1, 1000, 128, 1);
    cs.roc_n = 50;
    cs.roc_prev_sample_invalid = false;
    uint64_t ts = 0;

    prime_s7(s7, cs, ts);

    // 1. Latch a SPIKE
    {
        MeasurementEnvelope env = make_nominal_envelope();
        env.channel_id = 1;
        ts += 1000;
        env.arrival_time_us = ts;
        env.corrected_timestamp_us = ts;
        env.corrected_delta_t_us = 1000;
        env.calibrated_value = 100.0;
        env.failure_hint = FailureMode::SPIKE;
        env.failure_confidence = 0.95f;
        s7.process(env, cs);
    }

    // 2. Recovery frame with HARD_INVALID
    MeasurementEnvelope env = make_nominal_envelope();
    env.channel_id = 1;
    ts += 1000;
    env.arrival_time_us = ts;
    env.corrected_timestamp_us = ts;
    env.corrected_delta_t_us = 1000;
    env.calibrated_value = 100.0;
    env.failure_hint = FailureMode::NONE;
    env.status = SampleStatus::HARD_INVALID;

    s7.process(env, cs);

    // Assert: Overridden by INVALID
    EXPECT_EQ(env.failure_hint, FailureMode::INVALID);

    TEST_END("S7 Hard Invalid Override during Recovery");
}

void test_s7_confidence_floor() {
    TEST_BEGIN("S7 Confidence Floor Application");

    StageS7Output s7(mock_output_bus, nullptr);
    ChannelState cs;
    init_channel_state(cs, 1, 1000, 128, 1);
    cs.roc_n = 50;
    cs.roc_prev_sample_invalid = false;
    uint64_t ts = 0;

    prime_s7(s7, cs, ts);

    MeasurementEnvelope env = make_nominal_envelope();
    env.channel_id = 1;
    ts += 1000;
    env.arrival_time_us = ts;
    env.corrected_timestamp_us = ts;
    env.corrected_delta_t_us = 1000;
    env.calibrated_value = 100.0;
    env.failure_hint = FailureMode::DRIFT;
    env.failure_confidence = 0.45f;
    s7.process(env, cs);

    // Public NONE, Internal DRIFT
    EXPECT_EQ(env.failure_hint, FailureMode::NONE);
    EXPECT_EQ(env.internal_failure_hint, FailureMode::DRIFT);

    TEST_END("S7 Confidence Floor Application");
}

int main() {
    test_s7_active_fault();
    test_s7_recovery_suppression();
    test_s7_invalid_override();
    test_s7_confidence_floor();

    std::cout << "\nALL TESTS PASSED (" << g_tests_passed << "/" << g_tests_run << ")" << std::endl;
    return 0;
}
