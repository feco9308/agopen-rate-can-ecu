#pragma once
#include <Arduino.h>

namespace cfg {

// loop
static constexpr uint32_t MAIN_LOOP_HZ = 100;
static constexpr uint32_t GLOBAL_CONTROL_HZ = 50;

// limits
static constexpr float RPM_MIN = 0.0f;
static constexpr float RPM_MAX = 250.0f;

// sync
static constexpr float POSITION_KP = 0.5f;

} // namespace cfg

enum class SystemMode : uint8_t {
    OFF = 0,
    MANUAL = 1,
    AUTO = 2,
    CALIBRATION = 3,
    FAULT_HOLD = 4
};

enum ControlFlags : uint8_t {
    CTRL_DRIVE_ENABLE = 1 << 0,
    CTRL_SYNC_ENABLE  = 1 << 1,
    CTRL_ESTOP        = 1 << 2
};