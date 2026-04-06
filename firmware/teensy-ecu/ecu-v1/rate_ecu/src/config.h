#pragma once
#include <Arduino.h>

namespace cfg {

// loop
static constexpr uint32_t MAIN_LOOP_HZ = 100;
static constexpr uint32_t GLOBAL_CONTROL_HZ = 50;
static constexpr uint32_t NODE_COMMAND_HZ = 20;

// limits
static constexpr float RPM_MIN = 0.0f;
// GLOBAL_CONTROL.base_rpm is now encoded as uint16 with 1 rpm resolution.
static constexpr float RPM_MAX = 10000.0f;

// geometry / machine defaults
static constexpr uint8_t MAX_SENSOR_CHANNELS = 4;           // compiled-in capacity
static constexpr uint8_t DEFAULT_ACTIVE_SENSOR_CHANNELS = 1;
static constexpr uint8_t PGN_SECTION_BIT_COUNT = 16;          // relay_lo + relay_hi
static constexpr uint8_t NODE_COUNT_MAX = 16;                // current CAN node range uses 0x101..0x110
static constexpr uint8_t DEFAULT_CONFIGURED_ROW_COUNT = 6;   // rows/sections configured in the machine
static constexpr uint16_t DEFAULT_HOLES_PER_REV = 26;
static constexpr float DEFAULT_DRIVE_RATIO = 1.0f;         // disc-side drivetrain multiplier
static constexpr float DEFAULT_MOTOR_RATIO = 2.0f;         // motor rpm / disc rpm after drivetrain
static constexpr float DEFAULT_UPM_SCALE = 100.0f;         // ECU-side scaling for AOG target_upm
static constexpr float DEFAULT_PLANTER_ROW_WIDTH_CM = 70.0f;
static constexpr uint32_t DEFAULT_PLANTER_TARGET_POPULATION = 60000u;
static constexpr float DEFAULT_PLANTER_DOUBLES_FACTOR = 2.0f;
static constexpr bool DEFAULT_PLANTER_METRIC = true;
static constexpr uint8_t DEFAULT_BLOCKAGE_ROWS_PER_MODULE = 16;
static constexpr uint8_t DEFAULT_BLOCKAGE_THRESHOLD = 20;
// sync
static constexpr float POSITION_KP = 0.5f;
static constexpr float TRIM_RPM_LIMIT = 200.0f;

// CAN
static constexpr uint32_t CAN_BAUDRATE = 250000;
static constexpr uint32_t NODE_TIMEOUT_MS = 250;

// CAN IDs
struct CanProfile {
    uint16_t global_control;
    uint16_t global_timebase;
    uint16_t global_estop;
    uint16_t node_cmd_base;
    uint16_t node_pres_base;
    uint16_t node_cfg_ack_base;
    uint16_t node_status_base;
    uint16_t node_diag_base;
    uint8_t status_bit_index;
};

// sensor 0 keeps the current CAN IDs
// sensor 1 defaults to a second ID bank where every message starts with 0x2..
static constexpr CanProfile CAN_PROFILES[MAX_SENSOR_CHANNELS] = {
    {0x080, 0x081, 0x082, 0x100, 0x140, 0x160, 0x180, 0x1C0, 0},
    {0x200, 0x201, 0x202, 0x210, 0x240, 0x260, 0x280, 0x2C0, 1},
    {0x300, 0x301, 0x302, 0x310, 0x340, 0x360, 0x380, 0x3C0, 1},
    {0x400, 0x401, 0x402, 0x410, 0x440, 0x460, 0x480, 0x4C0, 1},
};

constexpr uint32_t sectionMaskLimit() {
    return (PGN_SECTION_BIT_COUNT >= 32)
        ? 0xFFFFFFFFu
        : ((1u << PGN_SECTION_BIT_COUNT) - 1u);
}

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
    CTRL_ESTOP        = 1 << 2,
    CTRL_DIAG_ENABLE  = 1 << 3
};

enum class MonitorOutputMode : uint8_t {
    OFF = 0,
    PLANTER = 1,
    BLOCKAGE = 2
};

enum NodeCommandCode : uint8_t {
    NODE_CMD_NOP         = 0,
    NODE_CMD_ENABLE      = 1,
    NODE_CMD_DISABLE     = 2,
    NODE_CMD_ZERO_POS    = 3,
    NODE_CMD_CLEAR_FAULT = 4
};

enum NodeCommandFlags : uint8_t {
    NODE_FLAG_ALLOW_RUN       = 1 << 0,
    NODE_FLAG_INVERT_DIR      = 1 << 1,
    NODE_FLAG_USE_LOCAL_SENSOR= 1 << 2
};

struct GlobalControlFrame {
    uint8_t system_mode;
    uint8_t control_flags;
    uint16_t base_rpm_u16;
    uint16_t sync_pos_u16;
    uint8_t sequence;
    uint8_t reserved;
};

struct NodeCommandFrame {
    uint8_t node_command;
    uint8_t node_flags;
    int16_t trim_rpm_x10;
    int16_t pos_offset_u16;
    uint8_t sequence;
    uint8_t reserved;
};

struct NodeStatusFastFrame {
    uint8_t status_flags;
    uint8_t error_code;
    uint16_t actual_rpm_u16;
    uint16_t actual_pos_u16;
    uint8_t alive_counter;
    int8_t sync_error_x256rev;
};

struct NodeDiagFrame {
    uint16_t bus_voltage_x10;
    int16_t motor_current_x10;
    uint8_t controller_temp_c;
    uint8_t motor_temp_c;
    uint8_t fault_flags;
    uint8_t warning_flags;
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
    float bus_voltage = 0.0f;
    float motor_current = 0.0f;
    uint8_t controller_temp = 0;
    uint8_t motor_temp = 0;
    uint8_t fault_flags = 0;
    uint8_t warning_flags = 0;
};
