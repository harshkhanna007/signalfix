// =============================================================================
// SignalFix AI — Stage 5 Validation Harness
// =============================================================================

#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/roc_detection_config.hpp"
#include "signalfix/module1/stages/stage_s5_rate_of_change.hpp"
#include <iostream>
#include <cassert>
#include <vector>
#include <cmath>

using namespace signalfix;

void assert_test(bool condition, const char* name) {
    if (!condition) {
        std::cerr << "[FAIL] " << name << std::endl;
        std::exit(1);
    }
    std::cout << "[PASS] " << name << std::endl;
}

void print_env(const MeasurementEnvelope& e) {
    std::cout << "  -> ROC: " << e.roc 
              << " | Limit: " << e.roc_adaptive_limit 
              << " | Stat: " << std::hex << static_cast<int>(e.status) << std::dec
              << " | Hint: " << static_cast<int>(e.failure_hint) << std::endl;
}

int main() {
    std::cout << "--- Starting S5 Test Harness ---\n";
    
    RocDetectionConfig config;
    config.warm_up_samples = 30u;
    config.fallback_limit = 5000.0;
    
    StageS5RateOfChange s5(config);
    ChannelState cs;
    init_channel_state(cs, 1, 1, 1000u, kRocWindowMax);

    auto make_sample = [](double val, uint64_t dt, SampleStatus status = SampleStatus::NOMINAL) {
        MeasurementEnvelope e = make_nominal_envelope();
        e.calibrated_value = val;
        e.delta_t_us = dt;
        e.status = status;
        return e;
    };

    double current_value = 100.0;

    // 1. Warmup / Steady State
    std::cout << "Test 1: Steady State (Warmup)\n";
    for (int i=0; i<40; i++) {
        MeasurementEnvelope e = make_sample(current_value, 1000u); // 1ms
        (void)s5.process(e, cs);
    }
    assert_test(cs.roc_normal_streak >= 10, "Streak accrued safely");
    assert_test(cs.roc_sigma_baseline > 0.0f, "Baseline stabilized");

    // 2. Spike Injection
    std::cout << "Test 2: Spike Injection\n";
    MeasurementEnvelope e_spike = make_sample(current_value + 50.0, 1000u); // Jump 50k unit/s
    (void)s5.process(e_spike, cs);
    print_env(e_spike);
    assert_test(has_flag(e_spike.status, SampleStatus::ROC_EXCEEDED), "Spike flagged");
    assert_test(e_spike.failure_hint == FailureMode::SPIKE, "Classified as SPIKE");

    // 3. Recovery
    std::cout << "Test 3: Recovery\n";
    current_value += 50.0; // Stay at new offset
    MeasurementEnvelope e_recov = make_sample(current_value, 1000u);
    (void)s5.process(e_recov, cs);
    print_env(e_recov);
    assert_test(!has_flag(e_recov.status, SampleStatus::ROC_EXCEEDED), "Spike recovered instantly");

    // Warmup again
    for (int i=0; i<10; i++) {
        MeasurementEnvelope e_warm = make_sample(current_value, 1000u);
        (void)s5.process(e_warm, cs);
    }

    // 4. Drift Test
    std::cout << "Test 4: Drift Test\n";
    int drift_detected = 0;
    for (int i=0; i<15; i++) {
        current_value += 1.5; // High enough to cross threshold
        MeasurementEnvelope e_drift = make_sample(current_value, 1000u);
        (void)s5.process(e_drift, cs);
        if (has_flag(e_drift.status, SampleStatus::ROC_EXCEEDED)) drift_detected++;
    }
    assert_test(drift_detected > 0, "Drift successfully latched");

    // 5. Missing Sample / Zero DT
    std::cout << "Test 5: Edge Cases\n";
    MeasurementEnvelope e_zero = make_sample(current_value, 0u);
    (void)s5.process(e_zero, cs);
    assert_test(has_flag(e_zero.status, SampleStatus::RATE_ANOMALY), "Zero DT flagged explicitly");

    std::cout << "--- All Basic Tests Passed ---\n";
    return 0;
}
