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
static constexpr uint32_t CAN_BAUDRATE = 250000;
static constexpr uint8_t NODE_COUNT_MAX = 8;
static constexpr uint32_t NODE_TIMEOUT_MS = 250;

// CAN IDs
static constexpr uint16_t CAN_ID_GLOBAL_CONTROL   = 0x080;
static constexpr uint16_t CAN_ID_GLOBAL_TIMEBASE  = 0x081;
static constexpr uint16_t CAN_ID_GLOBAL_ESTOP     = 0x082;

static constexpr uint16_t CAN_ID_NODE_CMD_BASE    = 0x100;
static constexpr uint16_t CAN_ID_NODE_PRES_BASE   = 0x140;
static constexpr uint16_t CAN_ID_NODE_CFG_ACK_BASE= 0x160;
static constexpr uint16_t CAN_ID_NODE_STATUS_BASE = 0x180;
static constexpr uint16_t CAN_ID_NODE_DIAG_BASE   = 0x1C0;

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

struct NodeStatusFastFrame {
    uint8_t status_flags;
    uint8_t error_code;
    int16_t actual_rpm_x10;
    uint16_t actual_pos_u16;
    uint8_t alive_counter;
    int8_t sync_error_x256rev;
};

struct NodeRuntimeState {
    bool online = false;
    uint32_t last_seen_ms = 0;
    uint8_t status_flags = 0;
    uint8_t error_code = 0;
    float actual_rpm = 0.0f;
    uint16_t actual_pos = 0;
    uint8_t alive_counter = 0;
    int8_t sync_error = 0;
};