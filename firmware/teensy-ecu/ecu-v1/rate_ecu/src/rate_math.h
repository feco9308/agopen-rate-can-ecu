#pragma once

#include <Arduino.h>
#include <stdint.h>

namespace rate_math {

inline uint8_t countActiveSections(uint16_t section_mask, uint8_t section_count) {
    uint8_t cnt = 0;
    for (uint8_t i = 0; i < section_count; ++i) {
        if (section_mask & (static_cast<uint16_t>(1u) << i)) {
            ++cnt;
        }
    }
    return cnt;
}

// AgOpen Rate app target_upm = Units Per Minute a teljes aktív munkaszélességre.
// 1 node = 1 sor = 1 tárcsa esetben az egy node-ra jutó UPM-et az aktív
// szakaszok számával osztjuk. Mivel az UPM már perc alapú mennyiség,
// NEM osztjuk még egyszer 60-nal.
inline float upmToDiscRpm(float total_upm,
                          uint16_t holes_per_rev,
                          uint8_t active_sections) {
    if (total_upm <= 0.0f || holes_per_rev == 0 || active_sections == 0) {
        return 0.0f;
    }

    const float upm_per_section = total_upm / static_cast<float>(active_sections);
    return upm_per_section / static_cast<float>(holes_per_rev);
}

} // namespace rate_math
