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
inline float targetUpmToMotorRpm(float target_upm,
                                 float upm_scale,
                                 uint16_t holes_per_rev,
                                 uint8_t active_sections,
                                 float combined_ratio) {
    if (target_upm <= 0.0f || upm_scale <= 0.0f || holes_per_rev == 0 ||
        active_sections == 0 || combined_ratio <= 0.0f) {
        return 0.0f;
    }

    const float total_upm = target_upm * upm_scale;
    const float row_upm = total_upm / static_cast<float>(active_sections);
    const float disc_rpm = row_upm / static_cast<float>(holes_per_rev);
    return disc_rpm * combined_ratio;
}

inline float motorRpmToFeedbackUpm(float motor_rpm,
                                   float upm_scale,
                                   uint16_t holes_per_rev,
                                   uint8_t active_sections,
                                   float combined_ratio) {
    if (motor_rpm <= 0.0f || upm_scale <= 0.0f || holes_per_rev == 0 ||
        active_sections == 0 || combined_ratio <= 0.0f) {
        return 0.0f;
    }

    const float disc_rpm = motor_rpm / combined_ratio;
    const float row_upm = disc_rpm * static_cast<float>(holes_per_rev);
    const float total_upm = row_upm * static_cast<float>(active_sections);
    return total_upm / upm_scale;
}

inline float summedMotorRpmToFeedbackUpm(float summed_motor_rpm,
                                         float upm_scale,
                                         uint16_t holes_per_rev,
                                         float combined_ratio) {
    if (summed_motor_rpm <= 0.0f || upm_scale <= 0.0f || holes_per_rev == 0 ||
        combined_ratio <= 0.0f) {
        return 0.0f;
    }

    const float summed_disc_rpm = summed_motor_rpm / combined_ratio;
    const float total_upm = summed_disc_rpm * static_cast<float>(holes_per_rev);
    return total_upm / upm_scale;
}

inline float shortestPosErrorRev(uint16_t target_pos_u16, uint16_t actual_pos_u16) {
    int32_t delta = static_cast<int32_t>(target_pos_u16) - static_cast<int32_t>(actual_pos_u16);
    if (delta > 32767) {
        delta -= 65536;
    } else if (delta < -32768) {
        delta += 65536;
    }
    return static_cast<float>(delta) / 65535.0f;
}

inline float positionErrorToTrimRpm(float base_rpm,
                                    float pos_error_rev,
                                    float kp,
                                    float trim_limit_rpm) {
    if (base_rpm <= 0.0f || kp <= 0.0f || trim_limit_rpm <= 0.0f) {
        return 0.0f;
    }

    float trim_rpm = pos_error_rev * base_rpm * kp;
    if (trim_rpm > trim_limit_rpm) trim_rpm = trim_limit_rpm;
    if (trim_rpm < -trim_limit_rpm) trim_rpm = -trim_limit_rpm;
    return trim_rpm;
}

} // namespace rate_math
