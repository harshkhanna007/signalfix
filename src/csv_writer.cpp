#include "signalfix/module1/csv_writer.hpp"
#include <sstream>
#include <iomanip>

namespace signalfix {

std::string CsvWriter::format_header() {
    return "timestamp_us,"
           "raw_value,"
           "calibrated_value,"
           "status,"
           "failure_hint,"
           "failure_confidence,"
           "is_active_fault,"
           "is_recovering,"
           "samples_since_fault_onset,"
           "confidence_at_onset,"
           "recovery_confirmed";
}

static const char* failure_mode_to_string(FailureMode mode) {
    switch(mode) {
        case FailureMode::NONE: return "NONE";
        case FailureMode::DRIFT: return "DRIFT";
        case FailureMode::GAP: return "GAP";
        case FailureMode::STALE: return "STALE";
        case FailureMode::SPIKE: return "SPIKE";
        case FailureMode::INVALID: return "INVALID";
        default: return "UNKNOWN";
    }
}

static const char* status_to_string(SampleStatus status) {
    if (has_flag(status, SampleStatus::HARD_INVALID)) return "HARD_INVALID";
    if (has_flag(status, SampleStatus::MISSING)) return "MISSING";
    if (has_flag(status, SampleStatus::STALE)) return "STALE";
    if (has_flag(status, SampleStatus::ROC_EXCEEDED)) return "ROC_EXCEEDED";
    if (has_flag(status, SampleStatus::RATE_ANOMALY)) return "RATE_ANOMALY";
    if (has_flag(status, SampleStatus::TIMING_ANOMALY)) return "TIMING_ANOMALY";
    return "NOMINAL";
}

std::string CsvWriter::format_row(const MeasurementEnvelope& env) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    ss << env.timestamp_us << ","
       << env.raw_value << ","
       << env.calibrated_value << ",";
       
    ss << status_to_string(env.status) << ","
       << failure_mode_to_string(env.failure_hint) << ",";
       
    ss << std::fixed << std::setprecision(2) << env.failure_confidence << ","
       << (env.fault_state.is_active_fault ? "true" : "false") << ","
       << (env.fault_state.is_recovering ? "true" : "false") << ","
       << env.fault_state.samples_since_fault_onset << ","
       << std::fixed << std::setprecision(2) << env.fault_state.confidence_at_onset << ","
       << (env.fault_state.recovery_confirmed ? "true" : "false");
       
    return ss.str();
}

} // namespace signalfix
