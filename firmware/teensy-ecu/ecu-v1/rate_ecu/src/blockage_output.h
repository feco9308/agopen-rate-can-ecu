#pragma once

#include <Arduino.h>

namespace blockage_output {

static constexpr uint16_t UDP_PORT = 25600;
static constexpr uint16_t PGN_RATE = 32100;

void buildPacket32100(uint8_t module_id, uint8_t row_id, uint8_t rate, uint8_t out[5]);
uint8_t estimateRateByte(uint32_t target_population, float target_rpm, float actual_rpm);

} // namespace blockage_output
