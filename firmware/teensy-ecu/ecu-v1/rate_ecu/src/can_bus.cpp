#include "can_bus.h"
#include "ecu_state.h"
#include "sync_axis.h"

// Később ide kerül a tényleges Teensy CAN library.
// Például FlexCAN_T4, ha azt választjuk.
// Most még csak váz + serial debug.

bool CanBus::begin() {
    Serial.print("CAN init start, baud=");
    Serial.println(cfg::CAN_BAUDRATE);

    // TODO:
    // tényleges CAN init
    // pl. FlexCAN_T4.begin();
    // pl. setBaudRate(cfg::CAN_BAUDRATE);

    Serial.println("CAN init OK (stub)");
    return true;
}

void CanBus::update() {
    // TODO:
    // bejövő frame-ek olvasása
    // node status / diag feldolgozása
}

void CanBus::sendGlobalControl(const EcuState& ecu, const SyncAxis& syncAxis) {
    GlobalControlFrame frame{};
    frame.system_mode = static_cast<uint8_t>(ecu.mode());
    frame.control_flags = ecu.flags();
    frame.base_rpm_x10 = static_cast<int16_t>(ecu.baseRpm() * 10.0f);
    frame.sync_pos_u16 = syncAxis.posU16();
    frame.sequence = sequence_++;
    frame.reserved = 0;

    // TODO:
    // tényleges 8 byte CAN küldés ID 0x080-on

    Serial.print("[CAN TX] ID=0x");
    Serial.print(cfg::CAN_ID_GLOBAL_CONTROL, HEX);
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
    // TODO:
    // tényleges E-STOP frame küldés

    Serial.print("[CAN TX] ID=0x");
    Serial.print(cfg::CAN_ID_GLOBAL_ESTOP, HEX);
    Serial.print(" reason=");
    Serial.println(reason);
}