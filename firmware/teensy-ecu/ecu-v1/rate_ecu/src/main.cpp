#include <Arduino.h>

#include "config.h"
#include "ecu_state.h"
#include "sync_axis.h"
#include "can_bus.h"
#include "node_manager.h"

EcuState ecu;
SyncAxis syncAxis;
CanBus canBus;
NodeManager nodeManager;

uint32_t lastLoop = 0;
uint32_t lastPrint = 0;
uint32_t lastGlobalControl = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("ECU v1 start");

    ecu.begin();
    syncAxis.begin();
    nodeManager.begin();

    if (!canBus.begin()) {
        Serial.println("CAN begin failed");
    }

    ecu.setMode(SystemMode::MANUAL);
    ecu.setDrive(true);
    ecu.setSync(true);
    ecu.setBaseRpm(100.0f);

    lastLoop = millis();
}

void loop() {
    const uint32_t now = millis();
    const uint32_t dt = now - lastLoop;
    lastLoop = now;

    ecu.update();
    syncAxis.update(ecu.baseRpm(), dt);
    canBus.update(nodeManager);
    nodeManager.update();

    if (now - lastGlobalControl >= (1000 / cfg::GLOBAL_CONTROL_HZ)) {
        lastGlobalControl = now;
        canBus.sendGlobalControl(ecu, syncAxis);
    }

    if (now - lastPrint > 1000) {
        lastPrint = now;

        Serial.print("mode=");
        Serial.print(static_cast<uint8_t>(ecu.mode()));
        Serial.print(" rpm=");
        Serial.print(ecu.baseRpm(), 1);
        Serial.print(" pos_u16=");
        Serial.print(syncAxis.posU16());
        Serial.print(" flags=");
        Serial.println(ecu.flags());

        const auto& n1 = nodeManager.node(1);
        Serial.print("node1 online=");
        Serial.print(n1.online);
        Serial.print(" rpm=");
        Serial.print(n1.actual_rpm, 1);
        Serial.print(" pos=");
        Serial.print(n1.actual_pos);
        Serial.print(" err=");
        Serial.println(n1.error_code);
    }

    delay(5);
}