#pragma once
#include "config.h"

class EcuState {
public:
    void begin();
    void update();

    void setMode(SystemMode mode);
    SystemMode mode() const;

    void setBaseRpm(float rpm);
    float baseRpm() const;

    void setDrive(bool en);
    bool drive() const;

    void setSync(bool en);
    bool sync() const;

    uint8_t flags() const;

private:
    SystemMode mode_ = SystemMode::OFF;
    float base_rpm_ = 0.0f;
    bool drive_ = false;
    bool sync_ = false;
};