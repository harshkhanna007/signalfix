// =============================================================================
// SignalFix AI — Module 1: Signal Ingestion & Pre-processing Pipeline
// File   : src/signalfix_runtime.cpp
// Spec   : SFX-M1-TDS-001  Revision 2.8.4 (Baseline-Gated)
// =============================================================================
#include <fstream>
#include <sstream>
#include "signalfix/module1/types.hpp"
#include "signalfix/module1/stages/stage_s5_rate_of_change.hpp"
#include "signalfix/module1/stages/stage_s5_5.hpp"
#include "signalfix/module1/stages/s7.hpp"
#include "signalfix/module1/channel_state.hpp"
#include "signalfix/module1/output_types.hpp"
#include "signalfix/module1/interpretation_layer.hpp"
#include <vector>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <type_traits>

namespace {

using namespace signalfix;

// ---------------------------------------------------------------------------
// Audit Traceability — log_configuration
// ---------------------------------------------------------------------------
void log_configuration_audit(const RocDetectionConfig& cfg) noexcept
{
    std::puts("\n=== [REV 2.8.4 0.1% TIER BOOT AUDIT: ROC CONFIG] ===");
    std::printf("  %-30s %.4f\n", "fallback_limit (ANCHOR):", cfg.fallback_limit);
    std::printf("  %-30s %.1f\n", "PHYSICAL_MAX_ROC:",      cfg.PHYSICAL_MAX_ROC);
    std::printf("  %-30s %.3f\n", "k_sigma_high:",         static_cast<double>(cfg.k_sigma_high));
    std::printf("  %-30s %.3f\n", "alpha (learning rate):", static_cast<double>(cfg.alpha_learning));
    std::printf("  %-30s %.3f\n", "beta_up (rise rate):",   static_cast<double>(cfg.beta_up));
    std::printf("  %-30s %.3f\n", "beta_down (fall rate):", static_cast<double>(cfg.beta_down));
    std::printf("  %-30s %.4f\n", "beta_baseline (anchor):",static_cast<double>(cfg.beta_baseline));
    std::printf("  %-30s %.2f / %.2f\n", "sigma_governor_ratios:", cfg.sigma_min_ratio, cfg.sigma_max_ratio);
    std::printf("  %-30s %u\n",    "warm_up_samples:",      cfg.warm_up_samples);
    std::puts("==============================================\n");
}

// ---------------------------------------------------------------------------
// Pipeline Alignment Stubs
// ---------------------------------------------------------------------------
static void stage_s0_adapter(signalfix::MeasurementEnvelope& env, signalfix::ChannelState&, double raw) noexcept {
    env.raw_value = raw; env.calibrated_value = raw;
}
static void stage_s0_watchdog(signalfix::MeasurementEnvelope& env, signalfix::ChannelState&, uint64_t time) noexcept {
    env.status |= signalfix::SampleStatus::MISSING;
    env.arrival_time_us = time;
    env.raw_value = std::numeric_limits<double>::quiet_NaN();
}
static void stage_s1_calibration(signalfix::MeasurementEnvelope&) noexcept {}
static void stage_s2_clock(signalfix::MeasurementEnvelope&) noexcept {}
static void stage_s3_plausibility(signalfix::MeasurementEnvelope& env, signalfix::ChannelState&) noexcept {
    if (env.calibrated_value > 1000000.0) env.status |= signalfix::SampleStatus::HARD_INVALID;
}
static void stage_s6_packager(signalfix::MeasurementEnvelope& env, signalfix::ChannelState&) noexcept {
    env.measurement_trust_tier = signalfix::derive_trust_tier(env.status);
}

// ---------------------------------------------------------------------------
// Audit Telemetry Subsystem
// ---------------------------------------------------------------------------
const char* describe_classification(signalfix::Classification c) noexcept {
    using C = signalfix::Classification;
    switch (c) {
        // NOMINAL: stream is healthy, no anomalies detected.
        // Previously "NORMAL" — renamed to read as an active state, not an absence.
        case C::NORMAL:        return "NOMINAL      ";

        // ACTIVE_FAILURE: pipeline confirmed a real signal anomaly (ROC, drift, etc.).
        // Previously "REAL_EVENT" — renamed to be immediately actionable on sight.
        case C::REAL_EVENT:    return "ACTIVE_FAULT  ";

        // SENSOR_FAULT: a transient hardware/firmware glitch (not a signal event).
        // Previously "SENSOR_GLITCH" — renamed to match operator vocabulary.
        case C::SENSOR_GLITCH: return "SENSOR_FAULT ";

        // NO_DATA: watchdog fired, sensor is not delivering samples.
        case C::NO_DATA:       return "NO_DATA      ";

        case C::DRIFT:         return "DRIFT       ";

        default:               return "UNKNOWN      ";
    }
}
struct ProcessingAnnotation {
    signalfix::Classification classification;
    float                     roc_raw;
};
static signalfix::ChannelState make_channel_state() noexcept {
    signalfix::ChannelState ch{};
    signalfix::init_channel_state(ch, 0x0003u, 7u, 1'000u, 0u);
    return ch;
}
struct RawSample { uint64_t time_us; double raw; };

// ---------------------------------------------------------------------------
// Sensor Identity — Centralized mapping from channel_id to physical name.
// Add entries here when new channels are configured. No magic indices needed.
// ---------------------------------------------------------------------------
struct SensorChannelConfig {
    uint32_t    channel_id;
    int         dataset_column_index; // 0-based column in the sensor block
    const char* sensor_name;
};

// NASA C-MAPSS FD001: columns 0-20 in the sensor block after s1/s2/s3.
// Only channel 0x0003 is currently active in this build.
static const SensorChannelConfig SENSOR_MAP[] = {
    { 0x0001u,  0, "T2 (Fan Inlet Temp)"         },
    { 0x0002u,  1, "T24 (LPC Outlet Temp)"        },
    { 0x0003u,  2, "T30 (HPC Temperature)"        },
    { 0x0004u,  3, "T50 (LPT Outlet Temp)"        },
    { 0x0005u,  4, "P2 (Fan Inlet Pressure)"      },
    { 0x0006u,  5, "P15 (Bypass-duct Pressure)"   },
    { 0x0007u,  6, "P30 (HPC Outlet Pressure)"    },
    { 0x0008u,  7, "Nf (Fan Speed)"               },
    { 0x0009u,  8, "Nc (Core Speed)"              },
    { 0x000Au,  9, "epr (Engine Pressure Ratio)"  },
    { 0x000Bu, 10, "Ps30 (Static HPC Pressure)"   },
    { 0x000Cu, 11, "phi (Fuel-Air Ratio)"         },
    { 0x000Du, 12, "NRf (Corrected Fan Speed)"    },
    { 0x000Eu, 13, "NRc (Corrected Core Speed)"   },
    { 0x000Fu, 14, "BPR (Bypass Ratio)"           },
    { 0x0010u, 15, "farB (Burner Fuel-Air Ratio)" },
    { 0x0011u, 16, "htBleed (Bleed Enthalpy)"     },
    { 0x0012u, 17, "Nf_dmd (Fan Speed Demand)"    },
    { 0x0013u, 18, "PCNfR_dmd (PC NfR Demand)"    },
    { 0x0014u, 19, "W31 (HPT Coolant Bleed)"      },
    { 0x0015u, 20, "W32 (LPT Coolant Bleed)"      },
};
static constexpr int SENSOR_MAP_SIZE = static_cast<int>(sizeof(SENSOR_MAP) / sizeof(SENSOR_MAP[0]));

[[nodiscard]] inline const char*
get_sensor_name(uint32_t channel_id) noexcept {
    for (int i = 0; i < SENSOR_MAP_SIZE; ++i) {
        if (SENSOR_MAP[i].channel_id == channel_id) return SENSOR_MAP[i].sensor_name;
    }
    return "Unknown Sensor";
}

[[nodiscard]] inline int
get_sensor_column(uint32_t channel_id) noexcept {
    for (int i = 0; i < SENSOR_MAP_SIZE; ++i) {
        if (SENSOR_MAP[i].channel_id == channel_id) return SENSOR_MAP[i].dataset_column_index;
    }
    return 2; // safe fallback to T30
}
const char* describe_status(signalfix::SampleStatus s) noexcept {
    using S = signalfix::SampleStatus;
    if (signalfix::has_flag(s, S::STALE)) return "STALE       ";
    if (signalfix::has_flag(s, S::HARD_INVALID)) return "HARD_INVALID";
    if (signalfix::has_flag(s, S::MISSING)) return "MISSING     ";
    if (signalfix::has_flag(s, S::ROC_EXCEEDED)) return "ROC_EXCEEDED";
    if (signalfix::has_flag(s, S::DRIFT_EXCEEDED)) return "DRIFT_EXCEEDED";
    return "NOMINAL     ";
}

void log_sample_full(int n, const signalfix::MeasurementEnvelope& env, 
                    const signalfix::ChannelState& ch, const ProcessingAnnotation& ann) noexcept
{
    char raw_buf[14], cal_buf[14], rr_buf[10], rc_buf[10], mu_buf[10], sig_buf[10], lim_buf[10], base_buf[10];
    if (std::isnan(env.raw_value)) std::snprintf(raw_buf, 14, "%10s", "NaN");
    else std::snprintf(raw_buf, 14, "%10.1f", env.raw_value);
    std::snprintf(cal_buf, 14, "%+11.4f", env.calibrated_value);
    std::snprintf(rr_buf, 10, "%7.0f", (double)ann.roc_raw);
    std::snprintf(rc_buf, 10, "%7.0f", (double)env.roc);
    std::snprintf(mu_buf, 10, "%7.0f", (double)ch.roc_mean);
    std::snprintf(sig_buf, 10, "%7.1f", (double)std::sqrt(std::max(0.0, ch.roc_m2)));
    std::snprintf(base_buf, 10, "%7.1f", (double)ch.roc_sigma_baseline);
    std::snprintf(lim_buf, 10, "%7.1f", (double)env.roc_threshold_used);

    const char* f_hint = "NONE ";
    switch(env.failure_hint) {
        case FailureMode::SPIKE:   f_hint = "SPIKE"; break;
        case FailureMode::DRIFT:   f_hint = "DRIFT"; break;
        case FailureMode::STALE:   f_hint = "STALE"; break;
        case FailureMode::GAP:     f_hint = "GAP  "; break;
        case FailureMode::INVALID: f_hint = "INVAL"; break;
        default: break;
    }

    std::printf("  [%3d] rr=%s | μ=%s σ=%s streak=%-2u | T=%s Status=%-12s Hint=%-5s(%.2f) dur=%-2u | Class=%s\n",
                n, rr_buf, mu_buf, sig_buf, ch.roc_normal_streak, lim_buf,
                describe_status(env.status), f_hint, (double)env.failure_confidence,
                env.failure_duration,
                describe_classification(ann.classification));
}

// ---------------------------------------------------------------------------
// Core Stream Application
// ---------------------------------------------------------------------------

static bool noop_processed_output(const signalfix::ProcessedSample&, void*) noexcept
{
    return true; // Always succeed, no side effects
}

void run_stream_core(const char* name, const RawSample* samples, int n, signalfix::ChannelState& ch) noexcept
{
    // [Rev 2.8.4 0.1% Tier Certified Parameters]
    RocDetectionConfig s5_config;
    s5_config.k_sigma_high = 3.5f;
    s5_config.sigma_min    = 1.0e-3f;
    s5_config.epsilon_dt   = 1.0e-9;
    s5_config.warm_up_samples = 30u;
    s5_config.fallback_limit = 5000.0; 
    s5_config.alpha_learning = 0.05;
    s5_config.beta_up        = 0.005;  
    s5_config.beta_down      = 0.05;   
    s5_config.beta_baseline  = 0.001;  
    s5_config.sigma_min_ratio = 0.5f;
    s5_config.sigma_max_ratio = 3.0f;
    
    log_configuration_audit(s5_config);
    StageS5RateOfChange s5_stage(s5_config);
    StageS55DriftPersistence s55_stage;
    
    StageS7Output s7_stage(noop_processed_output, nullptr);

    // Interpretation Layer config — shared default (10-sample PERSISTENT threshold).
    static const signalfix::InterpretationConfig interp_cfg{};

    // Change-detection state for smart logging.
    signalfix::InterpretationPrintState print_state{};
    // Trend evaluation state.
    signalfix::ChannelTrendState trend_state{};

    std::printf(" STREAM: %s (Rev 2.8.4 0.1%% Tier Certified)\n", name);
    std::puts(" ─────────────────────────────────────────────────────────────────────────────────────────────────────────────");

    for (int i=0; i<n; ++i) {
        const double raw = samples[i].raw;
        const uint64_t time = samples[i].time_us;
        signalfix::MeasurementEnvelope env = signalfix::make_nominal_envelope();
        env.arrival_time_us = time; env.delta_t_us = ch.nominal_delta_t_us; 
        
        env.corrected_timestamp_us = env.arrival_time_us;
        env.corrected_delta_t_us   = ch.nominal_delta_t_us;

        if (std::isnan(raw)) {
            stage_s0_watchdog(env, ch, time);
            (void)s7_stage.process(env, ch);
            const signalfix::InterpretationReport interp_wd =
                signalfix::interpret(env, interp_cfg);
            log_sample_full(i, env, ch, {signalfix::derive_classification(env), 0.0f});
            log_interpretation_smart(i, interp_wd, print_state);
            continue;
        }

        stage_s0_adapter(env, ch, raw);
        stage_s1_calibration(env);
        stage_s2_clock(env);
        stage_s3_plausibility(env, ch);

        double dt_s = static_cast<double>(env.delta_t_us) / 1'000'000.0;
        double rr = 0.0;
        if (ch.last_calibrated_valid && dt_s > 0) {
            rr = std::abs(env.calibrated_value - ch.last_calibrated_value) / dt_s;
        }

        (void)s5_stage.process(env, ch);
        (void)s55_stage.process(env, ch);
        stage_s6_packager(env, ch);
        
        (void)s7_stage.process(env, ch);

        // ── Interpretation Layer ─────────────────────────────────────────────
        // Stateless. O(1). Pure function of the envelope S7 just populated.
        // Augments the raw telemetry below — does not replace it.
        const signalfix::InterpretationReport interp =
            signalfix::interpret(env, interp_cfg);

        // ── Trend Indicator ─────────────────────────────────────────────────
        // State-aware smoothing and inertia. Managed strictly in presentation.
        const bool system_in_drift = signalfix::has_flag(env.status, signalfix::SampleStatus::DRIFT_EXCEEDED);
        signalfix::update_trend_state(trend_state, env.drift_gsigma);
        const char* trend_label = signalfix::compute_trend_label(trend_state, system_in_drift, env.drift_gsigma);

        log_sample_full(i, env, ch, {signalfix::derive_classification(env), (float)rr});
        log_interpretation_smart(i, interp, print_state);

        // Final diagnostic block — fires on every sample where drift is active.
        // Printed after smart log to avoid interleaving.
        if (system_in_drift) {
            print_final_drift_report(env, interp, get_sensor_name(ch.channel_id), ch.drift_confidence, trend_label);
        }
    }
}

static uint32_t lcg_next(uint32_t& s) noexcept { s = s*1'664'525u+1'013'904'223u; return s; }

void run_stream_scenario(const char* name, int samples_n, double baseline, double noise, int spike_at, double spike_val) noexcept
{
    RawSample sbuf[128]; uint32_t rng = 0x1234;
    ChannelState ch = make_channel_state();
    for (int i=0; i<samples_n && i<128; ++i) {
        sbuf[i].time_us = 1000000 + i * 1000;
        double n = (static_cast<double>(lcg_next(rng)) / 4294967295.0 * 2.0 - 1.0) * noise;
        sbuf[i].raw = (i == spike_at) ? spike_val : (baseline + n);
    }
    run_stream_core(name, sbuf, samples_n, ch);
}

} // namespace

int main() 
{
    std::puts("\n=== SIGNALFIX AI MODULE 1 (REV 2.8.4 0.1% TIER CERTIFICATION) ===");
    std::ifstream file("train_FD001.txt");
std::string line;

ChannelState ch = make_channel_state();

uint64_t time = 1000000;
uint64_t dt = 1000000; // 1 second
std::vector<RawSample> samples;
int current_engine = -1;
while (std::getline(file, line)) {
std::stringstream ss(line);

int engine_id, cycle;
double s1, s2, s3;

ss >> engine_id >> cycle >> s1 >> s2 >> s3;

double sensors[21];
for (int i = 0; i < 21; i++) {
    ss >> sensors[i];
}

// Resolve sensor column from the channel config — no magic indices.
const int col = get_sensor_column(ch.channel_id);
const double value = (col >= 0 && col < 21) ? sensors[col] : sensors[2];

if (current_engine == -1)
    current_engine = engine_id;

// 🔥 ENGINE CHANGE DETECTED
if (engine_id != current_engine) {
    run_stream_core("ENGINE", samples.data(), samples.size(), ch);

    samples.clear();
    ch = make_channel_state();
    current_engine = engine_id;
}

RawSample sample;
sample.raw = value;
sample.time_us = time;
samples.push_back(sample);

time += dt;
}

// 🔥 PROCESS LAST ENGINE (CRITICAL)
if (!samples.empty()) {
    run_stream_core("ENGINE", samples.data(), samples.size(), ch);
} 
return 0;
}