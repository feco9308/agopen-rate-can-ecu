#include "can_bus.h"
#include "ecu_state.h"
#include "sync_axis.h"
#include "node_manager.h"
#include "rate_math.h"
#include "runtime_config.h"
#include "service_can_protocol.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_256> Can0;

static constexpr bool CAN_TX_DEBUG = false;
static constexpr bool CAN_RX_DEBUG = false;
static constexpr bool CAN_SERVICE_DEBUG = true;
static constexpr bool CAN_TX_WARNINGS = true;

static uint16_t clampBaseRpmU16(float rpm) {
    if (rpm > cfg::RPM_MAX) rpm = cfg::RPM_MAX;
    if (rpm < 0.0f) rpm = 0.0f;
    const int32_t scaled = static_cast<int32_t>(rpm + 0.5f);
    if (scaled > 65535) return 65535;
    return static_cast<uint16_t>(scaled);
}

bool CanBus::begin() {
    Serial.print("CAN init start, baud=");
    Serial.println(cfg::CAN_BAUDRATE);

    Can0.begin();
    Can0.setBaudRate(cfg::CAN_BAUDRATE);

    Serial.println("CAN init OK");
    return true;
}

void CanBus::update(NodeManager* nodeManagers, uint8_t channelCount) {
    CAN_message_t msg;
    while (Can0.read(msg)) {
        handleFrame(msg, nodeManagers, channelCount);
    }
}

void CanBus::handleFrame(const CAN_message_t& msg, NodeManager* nodeManagers, uint8_t channelCount) {
    const uint32_t id = msg.id;

    switch (id) {
        case service_can::ID_UID_A:
            if (CAN_SERVICE_DEBUG && msg.len == 8) {
                Serial.print("[CAN SRV RX] UID_A uid32=0x");
                Serial.print(service_can::read_u32(msg.buf), HEX);
                Serial.print(" sensor=");
                Serial.print(msg.buf[4]);
                Serial.print(" node=");
                Serial.print(msg.buf[5]);
                Serial.print(" fw=");
                Serial.print(msg.buf[6]);
                Serial.print(".");
                Serial.println(msg.buf[7]);
            }
            return;

        case service_can::ID_UID_B:
            if (CAN_SERVICE_DEBUG && msg.len == 8) {
                Serial.print("[CAN SRV RX] UID_B uid32_hi=0x");
                Serial.print(service_can::read_u32(msg.buf), HEX);
                Serial.print(" hw=");
                Serial.print(msg.buf[4]);
                Serial.print(" caps=0x");
                Serial.print(msg.buf[5], HEX);
                Serial.print(" status=0x");
                Serial.println(msg.buf[6], HEX);
            }
            return;

        case service_can::ID_ACK:
            if (CAN_SERVICE_DEBUG && msg.len == 8) {
                Serial.print("[CAN SRV RX] ACK uid32=0x");
                Serial.print(service_can::read_u32(msg.buf), HEX);
                Serial.print(" cmd=0x");
                Serial.print(msg.buf[4], HEX);
                Serial.print(" result=");
                Serial.print(msg.buf[5]);
                Serial.print(" sensor=");
                Serial.print(msg.buf[6]);
                Serial.print(" node=");
                Serial.println(msg.buf[7]);
            }
            return;

        case service_can::ID_DIAG_RESP_A:
            if (CAN_SERVICE_DEBUG && msg.len == 8) {
                Serial.print("[CAN SRV RX] DIAG_A sensor=");
                Serial.print(msg.buf[0]);
                Serial.print(" node=");
                Serial.print(msg.buf[1]);
                Serial.print(" rpm=");
                Serial.print(service_can::read_i16(&msg.buf[2]) / 10.0f);
                Serial.print(" pos=");
                Serial.print(static_cast<uint16_t>(msg.buf[4] | (msg.buf[5] << 8)));
                Serial.print(" err=");
                Serial.print(msg.buf[6]);
                Serial.print(" flags=0x");
                Serial.println(msg.buf[7], HEX);
            }
            return;

        case service_can::ID_DIAG_RESP_B:
            if (CAN_SERVICE_DEBUG && msg.len == 8) {
                Serial.print("[CAN SRV RX] DIAG_B uid32=0x");
                Serial.print(service_can::read_u32(msg.buf), HEX);
                Serial.print(" syncErr=");
                Serial.print(static_cast<int8_t>(msg.buf[4]));
                Serial.print(" alive=");
                Serial.print(msg.buf[5]);
                Serial.print(" aux=");
                Serial.println(msg.buf[6]);
            }
            return;

        case service_can::ID_CFG_RESP:
            if (CAN_SERVICE_DEBUG && msg.len == 8) {
                Serial.print("[CAN SRV RX] CFG_RESP block=");
                Serial.print(msg.buf[0]);
                Serial.print(" node=");
                Serial.println(msg.buf[1]);
            }
            return;

        default:
            break;
    }

    for (uint8_t channelIndex = 0; channelIndex < channelCount && channelIndex < cfg::MAX_SENSOR_CHANNELS; ++channelIndex) {
        const cfg::CanProfile& profile = cfg::CAN_PROFILES[channelIndex];
        if (id >= (profile.node_status_base + 1u) &&
            id < (profile.node_status_base + cfg::NODE_COUNT_MAX + 1u) &&
            msg.len == 8) {

            const uint8_t nodeId = static_cast<uint8_t>(id - profile.node_status_base);

            NodeStatusFastFrame frame{};
            frame.status_flags       = msg.buf[0];
            frame.error_code         = msg.buf[1];
            frame.actual_rpm_u16     = static_cast<uint16_t>(msg.buf[2] | (msg.buf[3] << 8));
            frame.actual_pos_u16     = static_cast<uint16_t>(msg.buf[4] | (msg.buf[5] << 8));
            frame.alive_counter      = msg.buf[6];
            frame.sync_error_x256rev = static_cast<int8_t>(msg.buf[7]);

            nodeManagers[channelIndex].onStatusFrame(nodeId, frame);

            if (CAN_RX_DEBUG) {
                Serial.print("[CAN RX] ch=");
                Serial.print(channelIndex);
                Serial.print(" NODE_STATUS id=0x");
                Serial.print(id, HEX);
                Serial.print(" node=");
                Serial.print(nodeId);
                Serial.print(" rpm=");
                Serial.print(frame.actual_rpm_u16);
                Serial.print(" pos=");
                Serial.print(frame.actual_pos_u16);
                Serial.print(" alive=");
                Serial.println(frame.alive_counter);
            }

            return;
        }

        if (id >= (profile.node_diag_base + 1u) &&
            id < (profile.node_diag_base + cfg::NODE_COUNT_MAX + 1u) &&
            msg.len == 8) {

            const uint8_t nodeId = static_cast<uint8_t>(id - profile.node_diag_base);

            NodeDiagFrame frame{};
            frame.bus_voltage_x10 = static_cast<uint16_t>(msg.buf[0] | (msg.buf[1] << 8));
            frame.motor_current_x10 = static_cast<int16_t>(msg.buf[2] | (msg.buf[3] << 8));
            frame.controller_temp_c = msg.buf[4];
            frame.motor_temp_c = msg.buf[5];
            frame.fault_flags = msg.buf[6];
            frame.warning_flags = msg.buf[7];

            nodeManagers[channelIndex].onDiagFrame(nodeId, frame);

            if (CAN_RX_DEBUG) {
                Serial.print("[CAN RX] ch=");
                Serial.print(channelIndex);
                Serial.print(" NODE_DIAG id=0x");
                Serial.print(id, HEX);
                Serial.print(" node=");
                Serial.print(nodeId);
                Serial.print(" busV=");
                Serial.print(frame.bus_voltage_x10 / 10.0f);
                Serial.print(" current=");
                Serial.print(frame.motor_current_x10 / 10.0f);
                Serial.print(" ctrlC=");
                Serial.print(frame.controller_temp_c);
                Serial.print(" motorC=");
                Serial.print(frame.motor_temp_c);
                Serial.print(" warn=0x");
                Serial.print(frame.warning_flags, HEX);
                Serial.print(" fault=0x");
                Serial.println(frame.fault_flags, HEX);
            }

            return;
        }
    }
}

void CanBus::sendServiceFrame(uint32_t id, const uint8_t payload[8]) {
    CAN_message_t msg;
    msg.id = id;
    msg.len = 8;
    for (uint8_t i = 0; i < 8; ++i) {
        msg.buf[i] = payload[i];
    }

    const int ok = Can0.write(msg);
    if (CAN_SERVICE_DEBUG) {
        Serial.print("[CAN SRV TX] id=0x");
        Serial.print(id, HEX);
        Serial.print(" ok=");
        Serial.println(ok);
    }
}

void CanBus::sendGlobalControl(uint8_t channelIndex, const EcuState& ecu, const SyncAxis& syncAxis) {
    if (channelIndex >= cfg::MAX_SENSOR_CHANNELS) return;
    const cfg::CanProfile& profile = cfg::CAN_PROFILES[channelIndex];

    GlobalControlFrame frame{};
    frame.system_mode   = static_cast<uint8_t>(ecu.mode());
    frame.control_flags = ecu.flags();
    frame.base_rpm_u16  = clampBaseRpmU16(ecu.baseRpm());
    frame.sync_pos_u16  = syncAxis.posU16();
    frame.sequence      = sequence_[channelIndex]++;
    frame.reserved      = 0;

    CAN_message_t msg;
    msg.id = profile.global_control;
    msg.len = 8;
    msg.buf[0] = frame.system_mode;
    msg.buf[1] = frame.control_flags;
    msg.buf[2] = static_cast<uint8_t>(frame.base_rpm_u16 & 0xFF);
    msg.buf[3] = static_cast<uint8_t>((frame.base_rpm_u16 >> 8) & 0xFF);
    msg.buf[4] = static_cast<uint8_t>(frame.sync_pos_u16 & 0xFF);
    msg.buf[5] = static_cast<uint8_t>((frame.sync_pos_u16 >> 8) & 0xFF);
    msg.buf[6] = frame.sequence;
    msg.buf[7] = frame.reserved;

    const int ok = Can0.write(msg);

    if (CAN_TX_DEBUG) {
        Serial.print("[CAN TX] ID=0x");
        Serial.print(msg.id, HEX);
        Serial.print(" ch=");
        Serial.print(channelIndex);
        Serial.print(" ok=");
        Serial.print(ok);
        Serial.print(" mode=");
        Serial.print(frame.system_mode);
        Serial.print(" flags=");
        Serial.print(frame.control_flags);
        Serial.print(" rpm=");
        Serial.print(frame.base_rpm_u16);
        Serial.print(" sync=");
        Serial.print(frame.sync_pos_u16);
        Serial.print(" seq=");
        Serial.println(frame.sequence);
    }
}

void CanBus::sendNodeCommands(uint8_t channelIndex,
                              const EcuState& ecu,
                              const SyncAxis& syncAxis,
                              const NodeManager& nodeManager) {
    if (channelIndex >= cfg::MAX_SENSOR_CHANNELS) return;
    const cfg::CanProfile& profile = cfg::CAN_PROFILES[channelIndex];

    uint8_t maxNodeCommands = runtime_cfg::configuredRowCount();
    if (maxNodeCommands > cfg::NODE_COUNT_MAX) {
        maxNodeCommands = cfg::NODE_COUNT_MAX;
    }
    if (maxNodeCommands > cfg::PGN_SECTION_BIT_COUNT) {
        maxNodeCommands = cfg::PGN_SECTION_BIT_COUNT;
    }

    for (uint8_t nodeId = 1; nodeId <= maxNodeCommands; ++nodeId) {
        const bool sectionEnabled = ecu.sectionEnabled(nodeId);
        const bool allow_run = ecu.drive() && sectionEnabled && (ecu.baseRpm() > 0.01f);
        const NodeRuntimeState& nodeState = nodeManager.node(nodeId);
        const float posErrorRev = nodeState.online
            ? rate_math::shortestPosErrorRev(syncAxis.posU16(), nodeState.actual_pos)
            : 0.0f;
        const float trimRpm = allow_run
            ? rate_math::positionErrorToTrimRpm(
                ecu.baseRpm(), posErrorRev, runtime_cfg::positionKp(), runtime_cfg::trimRpmLimit()
            )
            : 0.0f;

        NodeCommandFrame frame{};
        frame.node_command = allow_run ? NODE_CMD_ENABLE : NODE_CMD_DISABLE;
        frame.node_flags = allow_run ? NODE_FLAG_ALLOW_RUN : 0;
        frame.trim_rpm_x10 = static_cast<int16_t>(trimRpm * 10.0f);
        frame.pos_offset_u16 = ecu.sectionMask();
        frame.sequence = node_sequence_[channelIndex]++;
        frame.reserved = 0;

        CAN_message_t msg;
        msg.id = profile.node_cmd_base + nodeId;
        msg.len = 8;
        msg.buf[0] = frame.node_command;
        msg.buf[1] = frame.node_flags;
        msg.buf[2] = static_cast<uint8_t>(frame.trim_rpm_x10 & 0xFF);
        msg.buf[3] = static_cast<uint8_t>((frame.trim_rpm_x10 >> 8) & 0xFF);
        msg.buf[4] = static_cast<uint8_t>(frame.pos_offset_u16 & 0xFF);
        msg.buf[5] = static_cast<uint8_t>((frame.pos_offset_u16 >> 8) & 0xFF);
        msg.buf[6] = frame.sequence;
        msg.buf[7] = frame.reserved;

        const int ok = Can0.write(msg);

        if (!ok && CAN_TX_WARNINGS) {
            Serial.print("[CAN TX WARN] ch=");
            Serial.print(channelIndex);
            Serial.print(" node=");
            Serial.print(nodeId);
            Serial.print(" id=0x");
            Serial.print(msg.id, HEX);
            Serial.println(" queue full / send failed");
        }

        if (CAN_TX_DEBUG) {
            Serial.print("[CAN TX NODE] ch=");
            Serial.print(channelIndex);
            Serial.print(" node=");
            Serial.print(nodeId);
            Serial.print(" ok=");
            Serial.print(ok);
            Serial.print(" baseRpm=");
            Serial.print(ecu.baseRpm(), 2);
            Serial.print(" trimRpm=");
            Serial.print(trimRpm, 2);
            Serial.print(" effectiveRpm=");
            Serial.print(ecu.baseRpm() + trimRpm, 2);
            Serial.print(" posErrRev=");
            Serial.println(posErrorRev, 4);
        }
    }
}

void CanBus::sendEstop(uint8_t channelIndex, uint8_t reason) {
    if (channelIndex >= cfg::MAX_SENSOR_CHANNELS) return;

    CAN_message_t msg;
    msg.id = cfg::CAN_PROFILES[channelIndex].global_estop;
    msg.len = 8;
    msg.buf[0] = 0xA5;
    msg.buf[1] = reason;
    msg.buf[2] = 0;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;

    const int ok = Can0.write(msg);
    Serial.print("[CAN TX] ESTOP ok=");
    Serial.print(ok);
    Serial.print(" reason=");
    Serial.println(reason);
}

void CanBus::sendServiceDiscover(uint8_t response_delay_slots, bool request_uid, bool request_cfg) {
    uint8_t payload[8]{};
    payload[0] = 1;
    payload[1] = response_delay_slots;
    payload[2] = request_uid ? 1 : 0;
    payload[3] = request_cfg ? 1 : 0;
    sendServiceFrame(service_can::ID_DISCOVER, payload);
}

void CanBus::sendServiceAssign(uint32_t uid32, uint8_t sensor_channel, uint8_t node_id, bool apply_now, bool save_now) {
    uint8_t payload[8]{};
    payload[0] = static_cast<uint8_t>(uid32 & 0xFF);
    payload[1] = static_cast<uint8_t>((uid32 >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((uid32 >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((uid32 >> 24) & 0xFF);
    payload[4] = sensor_channel;
    payload[5] = node_id;
    payload[6] = apply_now ? 1 : 0;
    payload[7] = save_now ? 1 : 0;
    sendServiceFrame(service_can::ID_ASSIGN, payload);
}

void CanBus::sendServiceSaveCfg(uint32_t uid32, uint8_t scope) {
    uint8_t payload[8]{};
    payload[0] = static_cast<uint8_t>(uid32 & 0xFF);
    payload[1] = static_cast<uint8_t>((uid32 >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((uid32 >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((uid32 >> 24) & 0xFF);
    payload[4] = 0xA5;
    payload[5] = scope;
    sendServiceFrame(service_can::ID_SAVE_CFG, payload);
}

void CanBus::sendServiceTestSpin(uint32_t uid32, int16_t rpm_x10, uint8_t duration_s, uint8_t mode) {
    uint8_t payload[8]{};
    payload[0] = static_cast<uint8_t>(uid32 & 0xFF);
    payload[1] = static_cast<uint8_t>((uid32 >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((uid32 >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((uid32 >> 24) & 0xFF);
    payload[4] = static_cast<uint8_t>(rpm_x10 & 0xFF);
    payload[5] = static_cast<uint8_t>((rpm_x10 >> 8) & 0xFF);
    payload[6] = duration_s;
    payload[7] = mode;
    sendServiceFrame(service_can::ID_TEST_SPIN, payload);
}

void CanBus::sendServiceDiagReq(uint32_t uid32, uint8_t detail_level, bool include_runtime, bool include_cfg) {
    uint8_t payload[8]{};
    payload[0] = static_cast<uint8_t>(uid32 & 0xFF);
    payload[1] = static_cast<uint8_t>((uid32 >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((uid32 >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((uid32 >> 24) & 0xFF);
    payload[4] = detail_level;
    payload[5] = include_runtime ? 1 : 0;
    payload[6] = include_cfg ? 1 : 0;
    sendServiceFrame(service_can::ID_DIAG_REQ, payload);
}

void CanBus::sendServiceReboot(uint32_t uid32) {
    uint8_t payload[8]{};
    payload[0] = static_cast<uint8_t>(uid32 & 0xFF);
    payload[1] = static_cast<uint8_t>((uid32 >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((uid32 >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((uid32 >> 24) & 0xFF);
    payload[4] = 0x5A;
    sendServiceFrame(service_can::ID_REBOOT, payload);
}

void CanBus::sendServiceIdentify(uint32_t uid32, uint8_t mode, uint8_t duration_s) {
    uint8_t payload[8]{};
    payload[0] = static_cast<uint8_t>(uid32 & 0xFF);
    payload[1] = static_cast<uint8_t>((uid32 >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((uid32 >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((uid32 >> 24) & 0xFF);
    payload[4] = mode;
    payload[5] = duration_s;
    sendServiceFrame(service_can::ID_IDENTIFY, payload);
}

void CanBus::sendServiceCfgRead(uint32_t uid32, uint8_t block_id) {
    uint8_t payload[8]{};
    payload[0] = static_cast<uint8_t>(uid32 & 0xFF);
    payload[1] = static_cast<uint8_t>((uid32 >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((uid32 >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((uid32 >> 24) & 0xFF);
    payload[4] = block_id;
    sendServiceFrame(service_can::ID_CFG_READ, payload);
}

void CanBus::sendServiceSetCanSource(uint32_t uid32,
                                     uint8_t sensor_channel,
                                     uint8_t node_id,
                                     uint8_t can_profile_index,
                                     bool save_now) {
    uint8_t payload[8]{};
    payload[0] = static_cast<uint8_t>(uid32 & 0xFF);
    payload[1] = static_cast<uint8_t>((uid32 >> 8) & 0xFF);
    payload[2] = static_cast<uint8_t>((uid32 >> 16) & 0xFF);
    payload[3] = static_cast<uint8_t>((uid32 >> 24) & 0xFF);
    payload[4] = sensor_channel;
    payload[5] = node_id;
    payload[6] = can_profile_index;
    payload[7] = save_now ? 1 : 0;
    sendServiceFrame(service_can::ID_SET_CAN_SRC, payload);
}
