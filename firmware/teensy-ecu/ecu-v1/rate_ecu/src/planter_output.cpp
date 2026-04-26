#include "planter_output.h"

#include <math.h>

namespace {

uint8_t crc(const uint8_t* data, size_t len_without_crc) {
    uint16_t sum = 0;
    for (size_t i = 2; i < len_without_crc; ++i) {
        sum = static_cast<uint16_t>(sum + data[i]);
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

} // namespace

namespace planter_output {

void buildPacket(uint8_t pgn, const uint8_t payload[8], uint8_t out[14]) {
    out[0] = 0x80;
    out[1] = 0x81;
    out[2] = 0x7B;
    out[3] = pgn;
    out[4] = 8;
    for (uint8_t i = 0; i < 8; ++i) {
        out[5 + i] = payload[i];
    }
    out[13] = crc(out, 13);
}

uint32_t estimatePopulation(uint32_t target_population, float target_rpm, float actual_rpm) {
    if (target_population == 0 || target_rpm <= 0.1f || actual_rpm <= 0.0f) {
        return 0;
    }

    const float ratio = actual_rpm / target_rpm;
    if (!isfinite(ratio) || ratio <= 0.0f) {
        return 0;
    }

    const float estimated = static_cast<float>(target_population) * ratio;
    if (!isfinite(estimated) || estimated <= 0.0f) {
        return 0;
    }

    if (estimated >= 4294967295.0f) {
        return 0xFFFFFFFFu;
    }

    return static_cast<uint32_t>(estimated + 0.5f);
}

uint32_t populationFromNode(const NodeRuntimeState& node_state,
                            uint32_t target_population,
                            float target_rpm,
                            float actual_rpm) {
    if ((node_state.seed_flags & SEED_FLAG_VALID) != 0 && node_state.population_x1k > 0) {
        return static_cast<uint32_t>(node_state.population_x1k) * 1000u;
    }

    return estimatePopulation(target_population, target_rpm, actual_rpm);
}

uint8_t estimateRowStatus(bool section_enabled, const NodeRuntimeState& node_state, float target_rpm) {
    if (!section_enabled) {
        return STATUS_NORMAL;
    }

    if (!node_state.online || node_state.error_code != 0 || node_state.fault_flags != 0) {
        return STATUS_RED;
    }

    if ((node_state.seed_flags & SEED_FLAG_BLOCKED) != 0 || node_state.blockage_pct >= 80u) {
        return STATUS_RED;
    }

    const float rpm_error = fabsf(node_state.actual_rpm - target_rpm);
    const float rpm_threshold = (target_rpm > 0.0f) ? max(50.0f, target_rpm * 0.10f) : 50.0f;
    if (node_state.warning_flags != 0 ||
        rpm_error > rpm_threshold ||
        node_state.skip_pct > 0 ||
        node_state.double_pct > 0 ||
        node_state.slowdown_pct > 0) {
        return STATUS_YELLOW;
    }

    return STATUS_NORMAL;
}

} // namespace planter_output
