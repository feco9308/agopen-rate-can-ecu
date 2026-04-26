#include "motor_node_can.h"

#include <string.h>

typedef struct
{
    uint16_t global_control;
    uint16_t node_cmd_base;
    uint16_t node_presence_base;
    uint16_t node_cfg_ack_base;
    uint16_t node_status_base;
    uint16_t node_diag_base;
} MotorNodeProfileIds;

static const MotorNodeProfileIds k_profiles[4] = {
    {0x080u, 0x100u, 0x140u, 0x160u, 0x180u, 0x1C0u},
    {0x200u, 0x210u, 0x240u, 0x260u, 0x280u, 0x2C0u},
    {0x300u, 0x310u, 0x340u, 0x360u, 0x380u, 0x3C0u},
    {0x400u, 0x410u, 0x440u, 0x460u, 0x480u, 0x4C0u}
};

static uint16_t read_u16_le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
    return (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
    data[0] = (uint8_t)(value & 0xFFu);
    data[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static uint16_t clamp_u16_from_i32(int32_t value)
{
    if (value <= 0) {
        return 0u;
    }
    if (value >= 65535) {
        return 65535u;
    }
    return (uint16_t)value;
}

static uint16_t clamp_target_rpm(uint16_t base_rpm, int16_t trim_rpm_x10)
{
    int32_t rpm = (int32_t)base_rpm + ((int32_t)trim_rpm_x10 / 10);
    return clamp_u16_from_i32(rpm);
}

static bool section_bit_is_set(uint16_t mask, uint8_t section_index)
{
    if (section_index >= 16u) {
        return false;
    }
    return (mask & (uint16_t)(1u << section_index)) != 0u;
}

static void update_run_permission(MotorNodeCan *node)
{
    const bool drive_enable = (node->global_control.control_flags & MOTOR_NODE_CTRL_DRIVE_ENABLE) != 0u;
    const bool estop = (node->global_control.control_flags & MOTOR_NODE_CTRL_ESTOP) != 0u;
    const bool cmd_enable = (node->node_command.node_command == MOTOR_NODE_CMD_ENABLE) ||
                            ((node->node_command.node_flags & MOTOR_NODE_FLAG_ALLOW_RUN) != 0u);

    node->section_active = section_bit_is_set(node->node_command.section_mask, node->cfg.section_index);
    node->target_motor_rpm = clamp_target_rpm(node->global_control.base_rpm, node->node_command.trim_rpm_x10);

    if ((!drive_enable) || estop || (!cmd_enable) || (!node->section_active)) {
        node->run_allowed = false;
        node->target_motor_rpm = 0u;
        node->status_fast.status_flags &= (uint8_t)~0x01u;
    } else {
        node->run_allowed = true;
        node->status_fast.status_flags |= 0x01u;
    }
}

static HAL_StatusTypeDef transmit_std_frame(FDCAN_HandleTypeDef *hfdcan, uint16_t can_id, const uint8_t data[8])
{
    FDCAN_TxHeaderTypeDef tx_header;

    tx_header.Identifier = can_id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0u;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, (uint8_t *)data);
}

static void send_status_fast(MotorNodeCan *node)
{
    uint8_t payload[8] = {0};

    payload[0] = node->status_fast.status_flags;
    payload[1] = node->status_fast.error_code;
    write_u16_le(&payload[2], node->status_fast.actual_rpm);
    write_u16_le(&payload[4], node->status_fast.actual_pos);
    payload[6] = node->status_fast.alive_counter++;
    payload[7] = (uint8_t)node->status_fast.sync_error;

    (void)transmit_std_frame(node->cfg.hfdcan, node->ids.node_status_id, payload);
}

static void send_presence(MotorNodeCan *node)
{
    uint8_t payload[8] = {0};

    payload[0] = node->presence.seed_flags;
    payload[1] = node->presence.blockage_pct;
    payload[2] = node->presence.slowdown_pct;
    payload[3] = node->presence.skip_pct;
    payload[4] = node->presence.double_pct;
    payload[5] = node->presence.singulation_pct;
    write_u16_le(&payload[6], node->presence.population_x1k);

    (void)transmit_std_frame(node->cfg.hfdcan, node->ids.node_presence_id, payload);
}

static void send_diag(MotorNodeCan *node)
{
    uint8_t payload[8] = {0};

    write_u16_le(&payload[0], node->diag.bus_voltage_x10);
    write_u16_le(&payload[2], (uint16_t)node->diag.motor_current_x10);
    payload[4] = node->diag.controller_temp_c;
    payload[5] = node->diag.motor_temp_c;
    payload[6] = node->diag.fault_flags;
    payload[7] = node->diag.warning_flags;

    (void)transmit_std_frame(node->cfg.hfdcan, node->ids.node_diag_id, payload);
}

void MotorNodeCan_Init(MotorNodeCan *node, const MotorNodeCanConfig *cfg)
{
    const MotorNodeProfileIds *profile_ids;

    if ((node == NULL) || (cfg == NULL) || (cfg->hfdcan == NULL)) {
        return;
    }

    memset(node, 0, sizeof(*node));
    node->cfg = *cfg;

    if (node->cfg.profile > MOTOR_NODE_CAN_PROFILE_S3) {
        node->cfg.profile = MOTOR_NODE_CAN_PROFILE_S0;
    }

    profile_ids = &k_profiles[(uint32_t)node->cfg.profile];

    node->ids.global_control_id = profile_ids->global_control;
    node->ids.node_cmd_id = (uint16_t)(profile_ids->node_cmd_base + node->cfg.node_id);
    node->ids.node_presence_id = (uint16_t)(profile_ids->node_presence_base + node->cfg.node_id);
    node->ids.node_cfg_ack_id = (uint16_t)(profile_ids->node_cfg_ack_base + node->cfg.node_id);
    node->ids.node_status_id = (uint16_t)(profile_ids->node_status_base + node->cfg.node_id);
    node->ids.node_diag_id = (uint16_t)(profile_ids->node_diag_base + node->cfg.node_id);

    node->presence.singulation_pct = 100u;
    node->configured = true;
}

void MotorNodeCan_OnRxMessage(MotorNodeCan *node, const FDCAN_RxHeaderTypeDef *header, const uint8_t data[8])
{
    if ((node == NULL) || (header == NULL) || (data == NULL) || (!node->configured)) {
        return;
    }

    if ((header->IdType != FDCAN_STANDARD_ID) || (header->DataLength != FDCAN_DLC_BYTES_8)) {
        return;
    }

    if (header->Identifier == node->ids.global_control_id) {
        node->global_control.system_mode = data[0];
        node->global_control.control_flags = data[1];
        node->global_control.base_rpm = read_u16_le(&data[2]);
        node->global_control.sync_pos = read_u16_le(&data[4]);
        node->global_control.sequence = data[6];
        update_run_permission(node);
        return;
    }

    if (header->Identifier == node->ids.node_cmd_id) {
        node->node_command.node_command = data[0];
        node->node_command.node_flags = data[1];
        node->node_command.trim_rpm_x10 = read_i16_le(&data[2]);
        node->node_command.section_mask = read_u16_le(&data[4]);
        node->node_command.sequence = data[6];

        if (node->node_command.node_command == MOTOR_NODE_CMD_ZERO_POS) {
            node->status_fast.actual_pos = 0u;
        }

        if (node->node_command.node_command == MOTOR_NODE_CMD_CLEAR_FAULT) {
            node->status_fast.error_code = 0u;
            node->diag.fault_flags = 0u;
        }

        update_run_permission(node);
    }
}

void MotorNodeCan_Process(MotorNodeCan *node, uint32_t now_ms)
{
    bool diag_enabled;

    if ((node == NULL) || (!node->configured)) {
        return;
    }

    diag_enabled = (node->global_control.control_flags & MOTOR_NODE_CTRL_DIAG_ENABLE) != 0u;

    if ((node->cfg.status_period_ms > 0u) &&
        ((now_ms - node->last_status_tx_ms) >= node->cfg.status_period_ms)) {
        node->last_status_tx_ms = now_ms;
        send_status_fast(node);
    }

    if ((node->cfg.presence_period_ms > 0u) &&
        ((now_ms - node->last_presence_tx_ms) >= node->cfg.presence_period_ms)) {
        node->last_presence_tx_ms = now_ms;
        send_presence(node);
    }

    if (diag_enabled &&
        (node->cfg.diag_period_ms > 0u) &&
        ((now_ms - node->last_diag_tx_ms) >= node->cfg.diag_period_ms)) {
        node->last_diag_tx_ms = now_ms;
        send_diag(node);
    }
}

void MotorNodeCan_SetMeasuredState(MotorNodeCan *node, uint16_t actual_rpm, uint16_t actual_pos, int8_t sync_error)
{
    if (node == NULL) {
        return;
    }

    node->status_fast.actual_rpm = actual_rpm;
    node->status_fast.actual_pos = actual_pos;
    node->status_fast.sync_error = sync_error;
}

void MotorNodeCan_SetDiag(MotorNodeCan *node,
                          float bus_voltage_v,
                          float motor_current_a,
                          uint8_t controller_temp_c,
                          uint8_t motor_temp_c,
                          uint8_t fault_flags,
                          uint8_t warning_flags)
{
    int32_t bus_v_x10;
    int32_t motor_a_x10;

    if (node == NULL) {
        return;
    }

    bus_v_x10 = (int32_t)(bus_voltage_v * 10.0f);
    motor_a_x10 = (int32_t)(motor_current_a * 10.0f);

    node->diag.bus_voltage_x10 = clamp_u16_from_i32(bus_v_x10);
    node->diag.motor_current_x10 = (int16_t)motor_a_x10;
    node->diag.controller_temp_c = controller_temp_c;
    node->diag.motor_temp_c = motor_temp_c;
    node->diag.fault_flags = fault_flags;
    node->diag.warning_flags = warning_flags;
}

void MotorNodeCan_SetSeedData(MotorNodeCan *node,
                              uint8_t seed_flags,
                              uint8_t blockage_pct,
                              uint8_t slowdown_pct,
                              uint8_t skip_pct,
                              uint8_t double_pct,
                              uint8_t singulation_pct,
                              uint16_t population_x1k)
{
    if (node == NULL) {
        return;
    }

    node->presence.seed_flags = seed_flags;
    node->presence.blockage_pct = blockage_pct;
    node->presence.slowdown_pct = slowdown_pct;
    node->presence.skip_pct = skip_pct;
    node->presence.double_pct = double_pct;
    node->presence.singulation_pct = singulation_pct;
    node->presence.population_x1k = population_x1k;
}

const MotorNodeCanIds *MotorNodeCan_GetIds(const MotorNodeCan *node)
{
    if (node == NULL) {
        return NULL;
    }

    return &node->ids;
}

uint16_t MotorNodeCan_GetTargetRpm(const MotorNodeCan *node)
{
    if (node == NULL) {
        return 0u;
    }

    return node->target_motor_rpm;
}

bool MotorNodeCan_IsRunAllowed(const MotorNodeCan *node)
{
    if (node == NULL) {
        return false;
    }

    return node->run_allowed;
}

bool MotorNodeCan_IsSectionActive(const MotorNodeCan *node)
{
    if (node == NULL) {
        return false;
    }

    return node->section_active;
}
