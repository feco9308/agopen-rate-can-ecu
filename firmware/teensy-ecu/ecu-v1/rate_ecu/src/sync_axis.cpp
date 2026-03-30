#include "sync_axis.h"

void SyncAxis::begin() {
    pos_ = 0.0f;
}

void SyncAxis::update(float rpm, uint32_t dt_ms) {
    const float dt = dt_ms / 1000.0f;

    pos_ += (rpm / 60.0f) * dt;

    while (pos_ >= 1.0f) pos_ -= 1.0f;
    while (pos_ < 0.0f) pos_ += 1.0f;
}

uint16_t SyncAxis::posU16() const {
    return static_cast<uint16_t>(pos_ * 65535.0f);
}

float SyncAxis::posRev() const {
    return pos_;
}