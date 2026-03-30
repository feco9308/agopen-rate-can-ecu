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

// CAN
static constexpr uint32_t CAN_BAUDRATE = 500000;

// CAN IDs
static constexpr uint16_t CAN_ID_GLOBAL_CONTROL = 0x080;
static constexpr uint16_t CAN_ID_GLOBAL_TIMEBASE = 0x081;
static constexpr uint16_t CAN_ID_GLOBAL_ESTOP = 0x082;

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

struct GlobalControlFrame {
    uint8_t system_mode;
    uint8_t control_flags;
    int16_t base_rpm_x10;
    uint16_t sync_pos_u16;
    uint8_t sequence;
    uint8_t reserved;
};