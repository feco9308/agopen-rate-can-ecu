#include "blockage_output.h"

#include <math.h>

namespace {

uint8_t crc(const uint8_t* data, size_t len_without_crc) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len_without_crc; ++i) {
        sum = static_cast<uint16_t>(sum + data[i]);
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

} // namespace

namespace blockage_output {

void buildPacket32100(uint8_t module_id, uint8_t row_id, uint8_t rate, uint8_t out[5]) {
    out[0] = 0x64;
    out[1] = 0x7D;
    out[2] = static_cast<uint8_t>(((row_id & 0x0Fu) << 4) | (module_id & 0x0Fu));
    out[3] = rate;
    out[4] = crc(out, 4);
}

uint8_t estimateRateByte(uint32_t target_population, float target_rpm, float actual_rpm) {
    if (target_population == 0 || target_rpm <= 0.1f || actual_rpm <= 0.0f) {
        return 0;
    }

    const float ratio = actual_rpm / target_rpm;
    if (!isfinite(ratio) || ratio <= 0.0f) {
        return 0;
    }

    const float estimated_population = static_cast<float>(target_population) * ratio;
    if (!isfinite(estimated_population) || estimated_population <= 0.0f) {
        return 0;
    }

    const uint32_t scaled = static_cast<uint32_t>(estimated_population / 1000.0f + 0.5f);
    return static_cast<uint8_t>(min<uint32_t>(255u, scaled));
}

} // namespace blockage_output
