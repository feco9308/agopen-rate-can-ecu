#include "can_bus.h"
#include "ecu_state.h"
#include "sync_axis.h"

FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can0;

bool CanBus::begin() {
    Serial.print("CAN init start, baud=");
    Serial.println(cfg::CAN_BAUDRATE);

    Can0.begin();
    Can0.setBaudRate(cfg::CAN_BAUDRATE);

    // próbából mailbox setup
    Can0.setMaxMB(16);
    Can0.enableFIFO();
    Can0.enableFIFOInterrupt();
    Can0.mailboxStatus();

    Serial.println("CAN init OK");
    return true;
}

void CanBus::update() {
    Can0.events();
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
    msg.flags.extended = 0;
    msg.buf[0] = frame.system_mode;
    msg.buf[1] = frame.control_flags;
    msg.buf[2] = static_cast<uint8_t>(frame.base_rpm_x10 & 0xFF);
    msg.buf[3] = static_cast<uint8_t>((frame.base_rpm_x10 >> 8) & 0xFF);
    msg.buf[4] = static_cast<uint8_t>(frame.sync_pos_u16 & 0xFF);
    msg.buf[5] = static_cast<uint8_t>((frame.sync_pos_u16 >> 8) & 0xFF);
    msg.buf[6] = frame.sequence;
    msg.buf[7] = frame.reserved;

    const int ok = Can0.write(msg);

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

void CanBus::sendEstop(uint8_t reason) {
    CAN_message_t msg;
    msg.id = cfg::CAN_ID_GLOBAL_ESTOP;
    msg.len = 8;
    msg.flags.extended = 0;
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