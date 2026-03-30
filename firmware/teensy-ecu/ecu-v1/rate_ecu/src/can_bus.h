#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "config.h"

class EcuState;
class SyncAxis;

class CanBus {
public:
    bool begin();
    void update();

    void sendGlobalControl(const EcuState& ecu, const SyncAxis& syncAxis);
    void sendEstop(uint8_t reason);

private:
    uint8_t sequence_ = 0;
};