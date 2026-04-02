#include <Arduino.h>

#include "config.h"
#include "ecu_state.h"
#include "sync_axis.h"
#include "can_bus.h"
#include "node_manager.h"
#include "ethernet_link.h"

EcuState ecus[cfg::MAX_SENSOR_CHANNELS];
SyncAxis syncAxes[cfg::MAX_SENSOR_CHANNELS];
CanBus canBus;
NodeManager nodeManagers[cfg::MAX_SENSOR_CHANNELS];
EthernetLink ethernetLink;

uint32_t lastLoop = 0;
uint32_t lastPrint = 0;
uint32_t lastGlobalControl = 0;
uint32_t lastNodeCommand = 0;

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("ECU v1 start");

    for (uint8_t sensorIndex = 0; sensorIndex < cfg::ACTIVE_SENSOR_CHANNELS; ++sensorIndex) {
        ecus[sensorIndex].begin();
        syncAxes[sensorIndex].begin();
        nodeManagers[sensorIndex].begin();
    }

    if (!canBus.begin()) {
        Serial.println("CAN begin failed");
    }

    ethernetLink.begin();

    lastLoop = millis();
}

void loop() {
    const uint32_t now = millis();
    const uint32_t dt = now - lastLoop;
    lastLoop = now;

    ethernetLink.update(ecus, cfg::ACTIVE_SENSOR_CHANNELS);

    for (uint8_t sensorIndex = 0; sensorIndex < cfg::ACTIVE_SENSOR_CHANNELS; ++sensorIndex) {
        ecus[sensorIndex].update();
        syncAxes[sensorIndex].update(ecus[sensorIndex].baseRpm(), dt);
    }

    canBus.update(nodeManagers, cfg::ACTIVE_SENSOR_CHANNELS);
    for (uint8_t sensorIndex = 0; sensorIndex < cfg::ACTIVE_SENSOR_CHANNELS; ++sensorIndex) {
        nodeManagers[sensorIndex].update();
    }

    if (now - lastGlobalControl >= (1000 / cfg::GLOBAL_CONTROL_HZ)) {
        lastGlobalControl = now;
        for (uint8_t sensorIndex = 0; sensorIndex < cfg::ACTIVE_SENSOR_CHANNELS; ++sensorIndex) {
            canBus.sendGlobalControl(sensorIndex, ecus[sensorIndex], syncAxes[sensorIndex]);
        }
    }

    if (now - lastNodeCommand >= (1000 / cfg::NODE_COMMAND_HZ)) {
        lastNodeCommand = now;
        for (uint8_t sensorIndex = 0; sensorIndex < cfg::ACTIVE_SENSOR_CHANNELS; ++sensorIndex) {
            canBus.sendNodeCommands(sensorIndex, ecus[sensorIndex]);
        }
    }

    ethernetLink.sendStatus(ecus, nodeManagers, cfg::ACTIVE_SENSOR_CHANNELS);

    if (now - lastPrint > 1000) {
        lastPrint = now;

        for (uint8_t sensorIndex = 0; sensorIndex < cfg::ACTIVE_SENSOR_CHANNELS; ++sensorIndex) {
            const auto& n1 = nodeManagers[sensorIndex].node(1);
            Serial.print("s");
            Serial.print(sensorIndex);
            Serial.print(" mode=");
            Serial.print(static_cast<uint8_t>(ecus[sensorIndex].mode()));
            Serial.print(" rpm=");
            Serial.print(ecus[sensorIndex].baseRpm(), 3);
            Serial.print(" upm=");
            Serial.print(ecus[sensorIndex].rateSourceUpm(), 3);
            Serial.print(" sectionMask=0b");
            Serial.print(static_cast<uint32_t>(ecus[sensorIndex].sectionMask()), BIN);
            Serial.print(" pos_u16=");
            Serial.print(syncAxes[sensorIndex].posU16());
            Serial.print(" flags=");
            Serial.println(ecus[sensorIndex].flags());

            Serial.print("s");
            Serial.print(sensorIndex);
            Serial.print(" node1 online=");
            Serial.print(n1.online);
            Serial.print(" rpm=");
            Serial.print(n1.actual_rpm, 1);
            Serial.print(" pos=");
            Serial.print(n1.actual_pos);
            Serial.print(" err=");
            Serial.println(n1.error_code);
        }
    }

    delay(5);
}
