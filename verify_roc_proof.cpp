#include "signalfix/module1/types.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/roc_detection_config.hpp"
#include "signalfix/module1/stages/stage_s5_rate_of_change.hpp"
#include <cstdio>
#include <vector>

using namespace signalfix;

int main() {
    RocDetectionConfig config;
    config.fallback_limit = 10.0;
    config.warm_up_samples = 30;
    config.baseline_reset_threshold = 3; // 3 violations to reset
    
    StageS5RateOfChange s5(config);
    ChannelState cs;
    init_channel_state(cs, 1, 1, 1000, 30);

    printf("--- ZERO-AMBIGUITY ROC PROOF TRACE (ROC = 0.05) ---\n");
    printf("N   | Value     | ROC     | Limit(A) | Used     | Status\n");
    printf("----------------------------------------------------------\n");
    
    for (int i = 0; i < 40; ++i) {
        MeasurementEnvelope env = make_nominal_envelope();
        env.calibrated_value = 10.0 + (i % 2 == 0 ? 0.000025 : -0.000025);
        env.delta_t_us = 1000;
        
        (void)s5.process(env, cs);
        
        if (i < 5 || (i >= 30 && i < 35)) {
            bool roc_exceeded = has_flag(env.status, SampleStatus::ROC_EXCEEDED);
            printf("[%02d] | %8.5f | %7.4f | %8.4f | %8.4f | %s\n", 
                   i, env.calibrated_value, (double)env.roc, 
                   (double)env.roc_adaptive_limit, (double)env.roc_threshold_used,
                   roc_exceeded ? "ROC_EXCEEDED" : "NOMINAL");
        }
    }

    printf("\n--- BASELINE PROTECTION (INVALID SAMPLE JUMP) ---\n");
    // 1. Inject HARD_INVALID
    MeasurementEnvelope inv_env = make_nominal_envelope();
    inv_env.status |= SampleStatus::HARD_INVALID;
    inv_env.calibrated_value = 50.0; // Distant value
    inv_env.delta_t_us = 1000;
    (void)s5.process(inv_env, cs);
    printf("Invalid Sample (50.0): roc=%.3f status=0x%X (EXPECTED: roc=0)\n", 
           (double)inv_env.roc, (unsigned)inv_env.status);

    // 2. Next valid sample (stabilization cycle)
    MeasurementEnvelope next_env = make_nominal_envelope();
    next_env.calibrated_value = 10.0; // Back to normal
    next_env.delta_t_us = 1000;
    (void)s5.process(next_env, cs);
    printf("Next Valid Sample (10.0): roc=%.3f status=%s (EXPECTED: roc=0, NO JUMP)\n", 
           (double)next_env.roc, has_flag(next_env.status, SampleStatus::ROC_EXCEEDED) ? "EXCEEDED" : "NOMINAL");

    printf("\n--- RESET HARDENING (2-CONSECUTIVE RULE) ---\n");
    // We need 3 violations at n=3 (threshold) AND 2 consecutive.
    for (int j = 0; j < 5; ++j) {
        MeasurementEnvelope v_env = make_nominal_envelope();
        v_env.calibrated_value = 20.0 + j * 1.0; 
        v_env.delta_t_us = 1000;
        (void)s5.process(v_env, cs);
        printf("[%d] Violation: roc=%.1f used=%.1f n=%llu status=%s\n", 
               j, (double)v_env.roc, (double)v_env.roc_threshold_used, 
               (unsigned long long)cs.roc_n,
               has_flag(v_env.status, SampleStatus::ROC_EXCEEDED) ? "EXCEEDED" : "NOMINAL");
    }

    printf("\n--- COLLAPSE GUARD (0.03 FLOOR) ---\n");
    // Force a very low variance/std
    cs.roc_m2 = 0.00000001; 
    cs.roc_mean = 0.0001f;
    MeasurementEnvelope low_env = make_nominal_envelope();
    low_env.calibrated_value = 10.0;
    low_env.delta_t_us = 1000;
    (void)s5.process(low_env, cs);
    printf("Low Noise Case: limit=%.4f used=%.4f (EXPECTED: >=0.03)\n", 
           (double)low_env.roc_adaptive_limit, (double)low_env.roc_threshold_used);

    return 0;
}
