#pragma once

#include <Arduino.h>

namespace service_can {

static constexpr uint32_t ID_DISCOVER      = 0x500;
static constexpr uint32_t ID_UID_A         = 0x501;
static constexpr uint32_t ID_UID_B         = 0x502;
static constexpr uint32_t ID_ASSIGN        = 0x503;
static constexpr uint32_t ID_SAVE_CFG      = 0x504;
static constexpr uint32_t ID_ACK           = 0x505;
static constexpr uint32_t ID_TEST_SPIN     = 0x506;
static constexpr uint32_t ID_DIAG_REQ      = 0x507;
static constexpr uint32_t ID_DIAG_RESP_A   = 0x508;
static constexpr uint32_t ID_DIAG_RESP_B   = 0x509;
static constexpr uint32_t ID_REBOOT        = 0x50A;
static constexpr uint32_t ID_IDENTIFY      = 0x50B;
static constexpr uint32_t ID_CFG_READ      = 0x50C;
static constexpr uint32_t ID_CFG_RESP      = 0x50D;
static constexpr uint32_t ID_SET_CAN_SRC   = 0x50E;

inline uint32_t read_u32(const uint8_t* data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

inline int16_t read_i16(const uint8_t* data) {
    return static_cast<int16_t>(data[0] | (data[1] << 8));
}

} // namespace service_can
