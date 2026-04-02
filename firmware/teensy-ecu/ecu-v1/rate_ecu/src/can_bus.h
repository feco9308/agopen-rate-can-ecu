#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "config.h"

class EcuState;
class SyncAxis;
class NodeManager;

class CanBus {
public:
    bool begin();
    void update(NodeManager* nodeManagers, uint8_t channelCount);

    void sendGlobalControl(uint8_t channelIndex, const EcuState& ecu, const SyncAxis& syncAxis);
    void sendNodeCommands(uint8_t channelIndex, const EcuState& ecu);
    void sendEstop(uint8_t channelIndex, uint8_t reason);

private:
    void handleFrame(const CAN_message_t& msg, NodeManager* nodeManagers, uint8_t channelCount);

private:
    uint8_t sequence_[cfg::MAX_SENSOR_CHANNELS]{};
    uint8_t node_sequence_[cfg::MAX_SENSOR_CHANNELS]{};
};
