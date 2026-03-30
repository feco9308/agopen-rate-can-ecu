#pragma once

#include <Arduino.h>

namespace legacy_rate {

static constexpr uint16_t UDP_LISTEN_PORT = 28888;
static constexpr uint16_t UDP_DEST_PORT   = 29999;

static constexpr uint16_t AGIO_LISTEN_PORT = 8888;
static constexpr uint16_t AGIO_DEST_PORT   = 9999;

static constexpr size_t MAX_READ_BUFFER = 100;
static constexpr size_t MAX_PGN_BUFFER  = 40;

enum : uint16_t {
    PGN_RATE_SETTINGS   = 32500,
    PGN_RELAY_SETTINGS  = 32501,
    PGN_PID_SETTINGS    = 32502,
    PGN_SUBNET_CHANGE   = 32503,
    PGN_ESP_STATUS      = 32600,
    PGN_MODULE_CONFIG   = 32700,
    PGN_NETWORK_CONFIG  = 32702,

    PGN_RATE_INFO_OUT   = 32400,
    PGN_ANALOG_INFO_OUT = 32401
};

inline uint8_t crc(const uint8_t* data, uint8_t length, uint8_t start = 0) {
    int sum = 0;
    for (uint8_t i = start; i < length; i++) sum += data[i];
    return static_cast<uint8_t>(sum);
}

inline bool good_crc(const uint8_t* data, uint8_t length) {
    if (length < 1) return false;
    return crc(data, length - 1, 0) == data[length - 1];
}

inline uint8_t parse_mod_id(uint8_t id) {
    return id >> 4;
}

inline uint8_t parse_sensor_id(uint8_t id) {
    return id & 0x0F;
}

inline uint8_t build_mod_sensor_id(uint8_t mod_id, uint8_t sensor_id) {
    return static_cast<uint8_t>((mod_id << 4) | (sensor_id & 0x0F));
}

struct RateSettingsPgn32500 {
    uint8_t mod_sensor_id = 0;
    float target_upm = 0.0f;
    float meter_cal = 0.0f;
    uint8_t command = 0;
    int16_t manual_pwm = 0;
};

struct RelaySettingsPgn32501 {
    uint8_t module_id = 0;
    uint8_t relay_lo = 0;
    uint8_t relay_hi = 0;
    uint8_t power_relay_lo = 0;
    uint8_t power_relay_hi = 0;
    uint8_t inverted_lo = 0;
    uint8_t inverted_hi = 0;
};

struct PidSettingsPgn32502 {
    uint8_t mod_sensor_id = 0;
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    uint8_t min_pwm = 0;
    uint8_t max_pwm = 0;
};

struct ModuleConfigPgn32700 {
    uint8_t module_id = 0;
    uint8_t sensor_count = 0;
    uint8_t command_flags = 0;
    uint8_t relay_control = 0;
};

struct NetworkConfigPgn32702 {
    char net_name[15]{};
    char net_password[15]{};
};

inline bool decode_32500(const uint8_t* data, size_t len, RateSettingsPgn32500& out) {
    if (len < 14 || !good_crc(data, 14)) return false;

    out.mod_sensor_id = data[2];

    const uint32_t rate_set =
        static_cast<uint32_t>(data[3]) |
        (static_cast<uint32_t>(data[4]) << 8) |
        (static_cast<uint32_t>(data[5]) << 16);
    out.target_upm = static_cast<float>(rate_set) * 0.001f;

    const uint32_t meter_cal =
        static_cast<uint32_t>(data[6]) |
        (static_cast<uint32_t>(data[7]) << 8) |
        (static_cast<uint32_t>(data[8]) << 16);
    out.meter_cal = static_cast<float>(meter_cal) * 0.001f;

    out.command = data[9];
    out.manual_pwm = static_cast<int16_t>(data[10] | (data[11] << 8));
    return true;
}

inline bool decode_32501(const uint8_t* data, size_t len, RelaySettingsPgn32501& out) {
    if (len < 10 || !good_crc(data, 10)) return false;

    out.module_id = data[2];
    out.relay_lo = data[3];
    out.relay_hi = data[4];
    out.power_relay_lo = data[5];
    out.power_relay_hi = data[6];
    out.inverted_lo = data[7];
    out.inverted_hi = data[8];
    return true;
}

inline bool decode_32502(const uint8_t* data, size_t len, PidSettingsPgn32502& out) {
    if (len < 19 || !good_crc(data, 19)) return false;

    out.mod_sensor_id = data[2];

    uint32_t tmp =
        static_cast<uint32_t>(data[3]) |
        (static_cast<uint32_t>(data[4]) << 8) |
        (static_cast<uint32_t>(data[5]) << 16) |
        (static_cast<uint32_t>(data[6]) << 24);
    out.kp = static_cast<float>(tmp) * 0.0001f;

    tmp =
        static_cast<uint32_t>(data[7]) |
        (static_cast<uint32_t>(data[8]) << 8) |
        (static_cast<uint32_t>(data[9]) << 16) |
        (static_cast<uint32_t>(data[10]) << 24);
    out.ki = static_cast<float>(tmp) * 0.0001f;

    tmp =
        static_cast<uint32_t>(data[11]) |
        (static_cast<uint32_t>(data[12]) << 8) |
        (static_cast<uint32_t>(data[13]) << 16) |
        (static_cast<uint32_t>(data[14]) << 24);
    out.kd = static_cast<float>(tmp) * 0.0001f;

    out.min_pwm = data[15];
    out.max_pwm = data[16];
    return true;
}

inline bool decode_32700(const uint8_t* data, size_t len, ModuleConfigPgn32700& out) {
    if (len < 31 || !good_crc(data, 31)) return false;
    out.module_id = data[2];
    out.sensor_count = data[3];
    out.command_flags = data[4];
    out.relay_control = data[5];
    return true;
}

inline bool decode_32702(const uint8_t* data, size_t len, NetworkConfigPgn32702& out) {
    if (len < 33 || !good_crc(data, 33)) return false;
    memset(out.net_name, 0, sizeof(out.net_name));
    memset(out.net_password, 0, sizeof(out.net_password));
    memcpy(out.net_name, &data[2], 14);
    memcpy(out.net_password, &data[17], 14);
    return true;
}

} // namespace legacy_rate