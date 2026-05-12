#ifndef MOTOR_NODE_CAN_H
#define MOTOR_NODE_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include "fdcan.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    MOTOR_NODE_CAN_PROFILE_S0 = 0,
    MOTOR_NODE_CAN_PROFILE_S1 = 1,
    MOTOR_NODE_CAN_PROFILE_S2 = 2,
    MOTOR_NODE_CAN_PROFILE_S3 = 3
} MotorNodeCanProfile;

typedef enum
{
    MOTOR_NODE_SYSTEM_OFF = 0,
    MOTOR_NODE_SYSTEM_MANUAL = 1,
    MOTOR_NODE_SYSTEM_AUTO = 2,
    MOTOR_NODE_SYSTEM_CALIBRATION = 3,
    MOTOR_NODE_SYSTEM_FAULT_HOLD = 4
} MotorNodeSystemMode;

enum
{
    MOTOR_NODE_CTRL_DRIVE_ENABLE = (1u << 0),
    MOTOR_NODE_CTRL_SYNC_ENABLE = (1u << 1),
    MOTOR_NODE_CTRL_ESTOP = (1u << 2),
    MOTOR_NODE_CTRL_DIAG_ENABLE = (1u << 3)
};

enum
{
    MOTOR_NODE_CMD_NOP = 0,
    MOTOR_NODE_CMD_ENABLE = 1,
    MOTOR_NODE_CMD_DISABLE = 2,
    MOTOR_NODE_CMD_ZERO_POS = 3,
    MOTOR_NODE_CMD_CLEAR_FAULT = 4
};

enum
{
    MOTOR_NODE_FLAG_ALLOW_RUN = (1u << 0),
    MOTOR_NODE_FLAG_INVERT_DIR = (1u << 1),
    MOTOR_NODE_FLAG_USE_LOCAL_SENSOR = (1u << 2)
};

typedef struct
{
    FDCAN_HandleTypeDef *hfdcan;
    uint8_t node_id;
    uint8_t section_index;
    MotorNodeCanProfile profile;
    uint32_t status_period_ms;
    uint32_t presence_period_ms;
    uint32_t diag_period_ms;
} MotorNodeCanConfig;

typedef struct
{
    uint16_t global_control_id;
    uint16_t node_cmd_id;
    uint16_t node_presence_id;
    uint16_t node_cfg_ack_id;
    uint16_t node_status_id;
    uint16_t node_diag_id;
} MotorNodeCanIds;

typedef struct
{
    uint8_t system_mode;
    uint8_t control_flags;
    uint16_t base_rpm;
    uint16_t sync_pos;
    uint8_t sequence;
} MotorNodeGlobalControl;

typedef struct
{
    uint8_t node_command;
    uint8_t node_flags;
    int16_t trim_rpm_x10;
    uint16_t section_mask;
    uint8_t sequence;
} MotorNodeCommand;

typedef struct
{
    uint8_t status_flags;
    uint8_t error_code;
    uint16_t actual_rpm;
    uint16_t actual_pos;
    uint8_t alive_counter;
    int8_t sync_error;
} MotorNodeStatusFast;

typedef struct
{
    uint16_t bus_voltage_x10;
    int16_t motor_current_x10;
    uint8_t controller_temp_c;
    uint8_t motor_temp_c;
    uint8_t fault_flags;
    uint8_t warning_flags;
} MotorNodeDiag;

typedef struct
{
    uint8_t seed_flags;
    uint8_t blockage_pct;
    uint8_t slowdown_pct;
    uint8_t skip_pct;
    uint8_t double_pct;
    uint8_t singulation_pct;
    uint16_t population_x1k;
} MotorNodePresence;

typedef struct
{
    MotorNodeCanConfig cfg;
    MotorNodeCanIds ids;
    MotorNodeGlobalControl global_control;
    MotorNodeCommand node_command;
    MotorNodeStatusFast status_fast;
    MotorNodeDiag diag;
    MotorNodePresence presence;
    uint16_t target_motor_rpm;
    bool section_active;
    bool run_allowed;
    bool configured;
    uint32_t last_status_tx_ms;
    uint32_t last_presence_tx_ms;
    uint32_t last_diag_tx_ms;
} MotorNodeCan;

void MotorNodeCan_Init(MotorNodeCan *node, const MotorNodeCanConfig *cfg);
void MotorNodeCan_OnRxMessage(MotorNodeCan *node, const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8]);
void MotorNodeCan_Process(MotorNodeCan *node, uint32_t now_ms);

void MotorNodeCan_SetMeasuredState(MotorNodeCan *node, uint16_t actual_rpm, uint16_t actual_pos, int8_t sync_error);
void MotorNodeCan_SetDiag(MotorNodeCan *node,
                          float bus_voltage_v,
                          float motor_current_a,
                          uint8_t controller_temp_c,
                          uint8_t motor_temp_c,
                          uint8_t fault_flags,
                          uint8_t warning_flags);
void MotorNodeCan_SetSeedData(MotorNodeCan *node,
                              uint8_t seed_flags,
                              uint8_t blockage_pct,
                              uint8_t slowdown_pct,
                              uint8_t skip_pct,
                              uint8_t double_pct,
                              uint8_t singulation_pct,
                              uint16_t population_x1k);

const MotorNodeCanIds *MotorNodeCan_GetIds(const MotorNodeCan *node);
uint16_t MotorNodeCan_GetTargetRpm(const MotorNodeCan *node);
bool MotorNodeCan_IsRunAllowed(const MotorNodeCan *node);
bool MotorNodeCan_IsSectionActive(const MotorNodeCan *node);

#ifdef __cplusplus
}
#endif

#endif
