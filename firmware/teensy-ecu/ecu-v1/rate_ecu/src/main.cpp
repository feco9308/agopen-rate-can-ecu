#include <Arduino.h>

#include "config.h"
#include "ecu_state.h"
#include "sync_axis.h"
#include "can_bus.h"
#include "node_manager.h"
#include "ethernet_link.h"
#include "rate_math.h"
#include "runtime_config.h"

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

    runtime_cfg::begin();

    for (uint8_t sensorIndex = 0; sensorIndex < cfg::MAX_SENSOR_CHANNELS; ++sensorIndex) {
        ecus[sensorIndex].begin();
        runtime_cfg::applyToEcu(ecus[sensorIndex], sensorIndex);
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
    const uint8_t activeSensorCount = runtime_cfg::activeSensorCount();

    ethernetLink.update(ecus, nodeManagers, canBus, activeSensorCount);

    for (uint8_t sensorIndex = 0; sensorIndex < activeSensorCount; ++sensorIndex) {
        ecus[sensorIndex].update();
        syncAxes[sensorIndex].update(ecus[sensorIndex].baseRpm(), dt);
    }

    canBus.update(nodeManagers, activeSensorCount);
    for (uint8_t sensorIndex = 0; sensorIndex < activeSensorCount; ++sensorIndex) {
        nodeManagers[sensorIndex].update();
    }

    if (now - lastGlobalControl >= (1000 / cfg::GLOBAL_CONTROL_HZ)) {
        lastGlobalControl = now;
        for (uint8_t sensorIndex = 0; sensorIndex < activeSensorCount; ++sensorIndex) {
            canBus.sendGlobalControl(sensorIndex, ecus[sensorIndex], syncAxes[sensorIndex]);
        }
    }

    if (now - lastNodeCommand >= (1000 / cfg::NODE_COMMAND_HZ)) {
        lastNodeCommand = now;
        for (uint8_t sensorIndex = 0; sensorIndex < activeSensorCount; ++sensorIndex) {
            canBus.sendNodeCommands(sensorIndex, ecus[sensorIndex], syncAxes[sensorIndex], nodeManagers[sensorIndex]);
        }
    }

    ethernetLink.sendStatus(ecus, nodeManagers, activeSensorCount);

    if (now - lastPrint > 1000) {
        lastPrint = now;

        for (uint8_t sensorIndex = 0; sensorIndex < activeSensorCount; ++sensorIndex) {
            const auto& n1 = nodeManagers[sensorIndex].node(1);
            const float avgActualRpm = nodeManagers[sensorIndex].averageActualRpm(ecus[sensorIndex]);
            const float totalActualRpm = nodeManagers[sensorIndex].totalActualRpm(ecus[sensorIndex]);
            const uint16_t avgActualPos = nodeManagers[sensorIndex].averageActualPos(ecus[sensorIndex]);
            const float node1TrimRpm = n1.online
                ? rate_math::positionErrorToTrimRpm(
                    ecus[sensorIndex].baseRpm(),
                    rate_math::shortestPosErrorRev(syncAxes[sensorIndex].posU16(), n1.actual_pos),
                    runtime_cfg::positionKp(),
                    runtime_cfg::trimRpmLimit()
                )
                : 0.0f;
            const float actualUpm = rate_math::summedMotorRpmToFeedbackUpm(
                totalActualRpm,
                ecus[sensorIndex].upmScale(),
                ecus[sensorIndex].holesPerRev(),
                ecus[sensorIndex].combinedRatio()
            );
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
            Serial.print(ecus[sensorIndex].flags());
            Serial.print(" actualUpm=");
            Serial.print(actualUpm, 3);
            Serial.print(" avgMotorRpm=");
            Serial.print(avgActualRpm, 1);
            Serial.print(" totalMotorRpm=");
            Serial.print(totalActualRpm, 1);
            Serial.print(" avgMotorPos=");
            Serial.print(avgActualPos);
            Serial.print(" node1TrimRpm=");
            Serial.println(node1TrimRpm, 2);

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
