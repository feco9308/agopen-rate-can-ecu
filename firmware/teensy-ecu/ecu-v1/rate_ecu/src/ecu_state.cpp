#include "ecu_state.h"

void EcuState::begin() {
    mode_ = SystemMode::OFF;
    base_rpm_ = 0.0f;
    drive_ = false;
    sync_ = false;
}

void EcuState::update() {
    // később ide jön fault és állapotgép logika
}

void EcuState::setMode(SystemMode mode) {
    mode_ = mode;
}

SystemMode EcuState::mode() const {
    return mode_;
}

void EcuState::setBaseRpm(float rpm) {
    if (rpm < cfg::RPM_MIN) rpm = cfg::RPM_MIN;
    if (rpm > cfg::RPM_MAX) rpm = cfg::RPM_MAX;
    base_rpm_ = rpm;
}

float EcuState::baseRpm() const {
    return base_rpm_;
}

void EcuState::setDrive(bool en) {
    drive_ = en;
}

bool EcuState::drive() const {
    return drive_;
}

void EcuState::setSync(bool en) {
    sync_ = en;
}

bool EcuState::sync() const {
    return sync_;
}

uint8_t EcuState::flags() const {
    uint8_t f = 0;
    if (drive_) f |= CTRL_DRIVE_ENABLE;
    if (sync_)  f |= CTRL_SYNC_ENABLE;
    return f;
}