#include "can_bus.h"
#include "ecu_state.h"
#include "sync_axis.h"
#include "node_manager.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

static constexpr bool CAN_TX_DEBUG = false;
static constexpr bool CAN_RX_DEBUG = false;

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
    for (uint8_t channelIndex = 0; channelIndex < channelCount && channelIndex < cfg::MAX_SENSOR_CHANNELS; ++channelIndex) {
        const cfg::CanProfile& profile = cfg::CAN_PROFILES[channelIndex];
        if (id >= (profile.node_status_base + 1u) &&
            id < (profile.node_status_base + cfg::NODE_COUNT_MAX + 1u) &&
            msg.len == 8) {

            const uint8_t nodeId = static_cast<uint8_t>(id - profile.node_status_base);

            NodeStatusFastFrame frame{};
            frame.status_flags       = msg.buf[0];
            frame.error_code         = msg.buf[1];
            frame.actual_rpm_x10     = static_cast<int16_t>(msg.buf[2] | (msg.buf[3] << 8));
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
                Serial.print(frame.actual_rpm_x10 / 10.0f);
                Serial.print(" pos=");
                Serial.print(frame.actual_pos_u16);
                Serial.print(" alive=");
                Serial.println(frame.alive_counter);
            }

            return;
        }
    }
}

void CanBus::sendGlobalControl(uint8_t channelIndex, const EcuState& ecu, const SyncAxis& syncAxis) {
    if (channelIndex >= cfg::MAX_SENSOR_CHANNELS) return;
    const cfg::CanProfile& profile = cfg::CAN_PROFILES[channelIndex];

    GlobalControlFrame frame{};
    frame.system_mode   = static_cast<uint8_t>(ecu.mode());
    frame.control_flags = ecu.flags();
    frame.base_rpm_x10  = static_cast<int16_t>(ecu.baseRpm() * 10.0f);
    frame.sync_pos_u16  = syncAxis.posU16();
    frame.sequence      = sequence_[channelIndex]++;
    frame.reserved      = 0;

    CAN_message_t msg;
    msg.id = profile.global_control;
    msg.len = 8;
    msg.buf[0] = frame.system_mode;
    msg.buf[1] = frame.control_flags;
    msg.buf[2] = static_cast<uint8_t>(frame.base_rpm_x10 & 0xFF);
    msg.buf[3] = static_cast<uint8_t>((frame.base_rpm_x10 >> 8) & 0xFF);
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
        Serial.print(" rpm_x10=");
        Serial.print(frame.base_rpm_x10);
        Serial.print(" sync=");
        Serial.print(frame.sync_pos_u16);
        Serial.print(" seq=");
        Serial.println(frame.sequence);
    }
}

void CanBus::sendNodeCommands(uint8_t channelIndex, const EcuState& ecu) {
    if (channelIndex >= cfg::MAX_SENSOR_CHANNELS) return;
    const cfg::CanProfile& profile = cfg::CAN_PROFILES[channelIndex];

    uint8_t maxNodeCommands = cfg::ACTIVE_SECTION_COUNT;
    if (maxNodeCommands > cfg::NODE_COUNT_MAX) {
        maxNodeCommands = cfg::NODE_COUNT_MAX;
    }
    if (maxNodeCommands > cfg::PGN_SECTION_BIT_COUNT) {
        maxNodeCommands = cfg::PGN_SECTION_BIT_COUNT;
    }

    for (uint8_t nodeId = 1; nodeId <= maxNodeCommands; ++nodeId) {
        const bool sectionEnabled = ecu.sectionEnabled(nodeId);
        const bool allow_run = ecu.drive() && sectionEnabled && (ecu.baseRpm() > 0.01f);

        NodeCommandFrame frame{};
        frame.node_command = allow_run ? NODE_CMD_ENABLE : NODE_CMD_DISABLE;
        frame.node_flags = allow_run ? NODE_FLAG_ALLOW_RUN : 0;
        frame.trim_rpm_x10 = 0;
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

        Can0.write(msg);
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
