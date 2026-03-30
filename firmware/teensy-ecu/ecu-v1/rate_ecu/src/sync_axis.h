#pragma once
#include <Arduino.h>

class SyncAxis {
public:
    void begin();
    void update(float rpm, uint32_t dt_ms);

    uint16_t posU16() const;
    float posRev() const;

private:
    float pos_ = 0.0f;
};