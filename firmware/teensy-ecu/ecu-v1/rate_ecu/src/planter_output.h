#pragma once

#include <Arduino.h>

#include "config.h"

class EcuState;

namespace planter_output {

static constexpr uint16_t UDP_PORT = 15555;

static constexpr uint8_t PGN_CONFIG = 0xE0;
static constexpr uint8_t PGN_POP_BY_ROW_9_16 = 0xE1;
static constexpr uint8_t PGN_POP_BY_ROW_1_8 = 0xE2;
static constexpr uint8_t PGN_DOUBLES = 0xE3;
static constexpr uint8_t PGN_SKIPS = 0xE4;
static constexpr uint8_t PGN_SUMMARY = 0xE5;
static constexpr uint8_t PGN_ROW_STATUS = 0xE6;

static constexpr uint8_t STATUS_NORMAL = 0;
static constexpr uint8_t STATUS_RED = 1;
static constexpr uint8_t STATUS_YELLOW = 2;
static constexpr uint8_t STATUS_PURPLE = 3;

void buildPacket(uint8_t pgn, const uint8_t payload[8], uint8_t out[14]);
uint32_t estimatePopulation(uint32_t target_population, float target_rpm, float actual_rpm);
uint32_t populationFromNode(const NodeRuntimeState& node_state, uint32_t target_population, float target_rpm, float actual_rpm);
uint8_t estimateRowStatus(bool section_enabled, const NodeRuntimeState& node_state, float target_rpm);

} // namespace planter_output
