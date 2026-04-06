#pragma once

#include "signalfix/module1/types.hpp"
#include <string>

namespace signalfix {

class CsvWriter {
public:
    static std::string format_header();
    static std::string format_row(const MeasurementEnvelope& envelope);
};

} // namespace signalfix
