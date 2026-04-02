#pragma once

#include <Arduino.h>

namespace custom_pgn {

static constexpr uint16_t PGN_ECU_CFG_GET          = 32800;
static constexpr uint16_t PGN_ECU_CFG_SET          = 32801;
static constexpr uint16_t PGN_ECU_CFG_SAVE         = 32802;
static constexpr uint16_t PGN_ECU_CFG_LOAD         = 32803;
static constexpr uint16_t PGN_ECU_CFG_STATUS       = 32804;
static constexpr uint16_t PGN_ECU_DIAG_CONTROL     = 32805;
static constexpr uint16_t PGN_ECU_DIAG_STATUS      = 32806;
static constexpr uint16_t PGN_ECU_DIAG_SENSOR      = 32807;
static constexpr uint16_t PGN_ECU_DIAG_NODE_SUMMARY= 32808;
static constexpr uint16_t PGN_NODE_DISCOVER         = 32900;
static constexpr uint16_t PGN_NODE_UID_A            = 32901;
static constexpr uint16_t PGN_NODE_UID_B            = 32902;
static constexpr uint16_t PGN_NODE_ASSIGN           = 32903;
static constexpr uint16_t PGN_NODE_SAVE_CFG         = 32904;
static constexpr uint16_t PGN_NODE_ACK              = 32905;
static constexpr uint16_t PGN_NODE_TEST_SPIN        = 32906;
static constexpr uint16_t PGN_NODE_DIAG_REQ         = 32907;
static constexpr uint16_t PGN_NODE_DIAG_RESP_A      = 32908;
static constexpr uint16_t PGN_NODE_DIAG_RESP_B      = 32909;
static constexpr uint16_t PGN_NODE_REBOOT           = 32910;
static constexpr uint16_t PGN_NODE_IDENTIFY         = 32911;
static constexpr uint16_t PGN_NODE_CFG_READ         = 32912;
static constexpr uint16_t PGN_NODE_CFG_RESP         = 32913;
static constexpr uint16_t PGN_NODE_SET_CAN_SOURCE   = 32914;

static constexpr size_t PACKET_LEN = 11;

enum : uint8_t {
    CFG_BLOCK_MACHINE = 0,
    CFG_BLOCK_DRIVE   = 1,
    CFG_BLOCK_DIAG    = 2,
    CFG_BLOCK_CHANNEL = 3,
    CFG_BLOCK_NETWORK = 4
};

inline uint8_t crc(const uint8_t* data, size_t len_without_crc) {
    uint16_t sum = 0;
    for (size_t i = 0; i < len_without_crc; ++i) {
        sum = static_cast<uint16_t>(sum + data[i]);
    }
    return static_cast<uint8_t>(sum & 0xFF);
}

inline bool good_crc(const uint8_t* data, size_t len) {
    return (len == PACKET_LEN) && (crc(data, PACKET_LEN - 1) == data[PACKET_LEN - 1]);
}

inline uint16_t parse_pgn(const uint8_t* data) {
    return static_cast<uint16_t>(data[0] | (static_cast<uint16_t>(data[1]) << 8));
}

inline void build_packet(uint16_t pgn, const uint8_t payload[8], uint8_t out[PACKET_LEN]) {
    out[0] = static_cast<uint8_t>(pgn & 0xFF);
    out[1] = static_cast<uint8_t>((pgn >> 8) & 0xFF);
    for (uint8_t i = 0; i < 8; ++i) {
        out[2 + i] = payload[i];
    }
    out[10] = crc(out, PACKET_LEN - 1);
}

} // namespace custom_pgn
