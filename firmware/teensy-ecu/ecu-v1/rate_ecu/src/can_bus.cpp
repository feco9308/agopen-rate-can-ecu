#include "can_bus.h"
#include "ecu_state.h"
#include "sync_axis.h"
#include "node_manager.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

// 0 = nincs CAN TX debug
// 1 = CAN TX debug bekapcsolva
static constexpr bool CAN_TX_DEBUG = false;

bool CanBus::begin() {
    Serial.print("CAN init start, baud=");
    Serial.println(cfg::CAN_BAUDRATE);

    Can0.begin();
    Can0.setBaudRate(cfg::CAN_BAUDRATE);

    Serial.println("CAN init OK");
    return true;
}

void CanBus::update(NodeManager& nodeManager) {
    CAN_message_t msg;

    while (Can0.read(msg)) {
        handleFrame(msg, nodeManager);
    }
}

void CanBus::handleFrame(const CAN_message_t& msg, NodeManager& nodeManager) {
    const uint32_t id = msg.id;

    if (id >= cfg::CAN_ID_NODE_STATUS_BASE &&
        id < (cfg::CAN_ID_NODE_STATUS_BASE + cfg::NODE_COUNT_MAX + 1) &&
        msg.len == 8) {

        const uint8_t nodeId = static_cast<uint8_t>(id - cfg::CAN_ID_NODE_STATUS_BASE);

        NodeStatusFastFrame frame{};
        frame.status_flags       = msg.buf[0];
        frame.error_code         = msg.buf[1];
        frame.actual_rpm_x10     = static_cast<int16_t>(msg.buf[2] | (msg.buf[3] << 8));
        frame.actual_pos_u16     = static_cast<uint16_t>(msg.buf[4] | (msg.buf[5] << 8));
        frame.alive_counter      = msg.buf[6];
        frame.sync_error_x256rev = static_cast<int8_t>(msg.buf[7]);

        nodeManager.onStatusFrame(nodeId, frame);

        Serial.print("[CAN RX] NODE_STATUS id=0x");
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
}

void CanBus::sendGlobalControl(const EcuState& ecu, const SyncAxis& syncAxis) {
    GlobalControlFrame frame{};
    frame.system_mode   = static_cast<uint8_t>(ecu.mode());
    frame.control_flags = ecu.flags();
    frame.base_rpm_x10  = static_cast<int16_t>(ecu.baseRpm() * 10.0f);
    frame.sync_pos_u16  = syncAxis.posU16();
    frame.sequence      = sequence_++;
    frame.reserved      = 0;

    CAN_message_t msg;
    msg.id = cfg::CAN_ID_GLOBAL_CONTROL;
    msg.len = 8;

    msg.buf[0] = frame.system_mode;
    msg.buf[1] = frame.control_flags;
    msg.buf[2] = static_cast<uint8_t>(frame.base_rpm_x10 & 0xFF);
    msg.buf[3] = static_cast<uint8_t>((frame.base_rpm_x10 >> 8) & 0xFF);
    msg.buf[4] = static_cast<uint8_t>(frame.sync_pos_u16 & 0xFF);
    msg.buf[5] = static_cast<uint8_t>((frame.sync_pos_u16 >> 8) & 0xFF);
    msg.buf[6] = frame.sequence;
    msg.buf[7] = frame.reserved;

    int ok = Can0.write(msg);

    if (CAN_TX_DEBUG) {
        Serial.print("[CAN TX] ID=0x");
        Serial.print(msg.id, HEX);
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

void CanBus::sendEstop(uint8_t reason) {
    CAN_message_t msg;
    msg.id = cfg::CAN_ID_GLOBAL_ESTOP;
    msg.len = 8;

    msg.buf[0] = 0xA5;
    msg.buf[1] = reason;
    msg.buf[2] = 0;
    msg.buf[3] = 0;
    msg.buf[4] = 0;
    msg.buf[5] = 0;
    msg.buf[6] = 0;
    msg.buf[7] = 0;

    int ok = Can0.write(msg);

    Serial.print("[CAN TX] ESTOP ok=");
    Serial.print(ok);
    Serial.print(" reason=");
    Serial.println(reason);
}