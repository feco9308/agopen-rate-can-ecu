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
    void update(NodeManager& nodeManager);

    void sendGlobalControl(const EcuState& ecu, const SyncAxis& syncAxis);
    void sendEstop(uint8_t reason);

private:
    void handleFrame(const CAN_message_t& msg, NodeManager& nodeManager);

private:
    uint8_t sequence_ = 0;
};