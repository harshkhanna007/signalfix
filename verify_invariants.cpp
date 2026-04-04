#include <iostream>
#include <iomanip>
#include <cstdint>
#include "include/signalfix/module1/types.hpp"

using namespace signalfix;

void check_combination(const char* name, SampleStatus s, bool expected) {
    bool result = validate_status_flags(s);
    std::cout << "  - [" << name << "]: result=" << (result ? "VALID" : "INVALID")
              << " (expected " << (expected ? "VALID" : "INVALID") << ") -> "
              << (result == expected ? "PASS" : "FAIL") << "\n";
    if (result != expected) exit(1);
}

int main() {
    std::cout << "=== Standalone Invariant Verification ===\n";

    // Valid cases
    check_combination("NOMINAL", SampleStatus::NOMINAL, true);
    check_combination("SOFT_SUSPECT", SampleStatus::SOFT_SUSPECT, true);
    check_combination("ROC_EXCEEDED", SampleStatus::ROC_EXCEEDED, true);
    check_combination("MISSING", SampleStatus::MISSING, true);
    check_combination("HARD_INVALID", SampleStatus::HARD_INVALID, true);
    check_combination("STALE | MISSING", SampleStatus::STALE | SampleStatus::MISSING, true);

    // Invalid cases (The "FAILURES")
    check_combination("HARD_INVALID | FILTER_CLIPPED", SampleStatus::HARD_INVALID | SampleStatus::FILTER_CLIPPED, false);
    check_combination("MISSING | SOFT_SUSPECT", SampleStatus::MISSING | SampleStatus::SOFT_SUSPECT, false);
    check_combination("MISSING | FILTER_CLIPPED", SampleStatus::MISSING | SampleStatus::FILTER_CLIPPED, false);
    check_combination("INTERPOLATED | SOFT_SUSPECT", SampleStatus::INTERPOLATED | SampleStatus::SOFT_SUSPECT, false);

    // NEW ROC Invariants
    check_combination("HARD_INVALID | ROC_EXCEEDED", SampleStatus::HARD_INVALID | SampleStatus::ROC_EXCEEDED, false);
    check_combination("MISSING | ROC_EXCEEDED", SampleStatus::MISSING | SampleStatus::ROC_EXCEEDED, false);
    check_combination("INTERPOLATED | ROC_EXCEEDED", SampleStatus::INTERPOLATED | SampleStatus::ROC_EXCEEDED, false);

    std::cout << "\n=== ALL CHECKS PASSED ===\n";
    return 0;
}
