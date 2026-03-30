#include "ethernet_link.h"

#include "legacy_rate_protocol.h"
#include "config.h"

namespace {
constexpr uint32_t RC_TIMEOUT_MS = 1000;
constexpr uint32_t STATUS_TX_MS  = 200;

constexpr uint8_t DEFAULT_IP0 = 192;
constexpr uint8_t DEFAULT_IP1 = 168;
constexpr uint8_t DEFAULT_IP2 = 1;
constexpr uint8_t DEFAULT_IP3 = 200;
}

bool EthernetLink::begin() {
    static uint8_t local_mac[] = {0x0A, 0x0B, 0x42, 0x0C, 0x0D, DEFAULT_IP3};
    IPAddress local_ip(DEFAULT_IP0, DEFAULT_IP1, DEFAULT_IP2, DEFAULT_IP3);
    IPAddress subnet(255, 255, 255, 0);
    IPAddress gateway(DEFAULT_IP0, DEFAULT_IP1, DEFAULT_IP2, 1);

    Ethernet.begin(local_mac, local_ip, gateway, gateway, subnet);
    delay(1500);

    destination_ip_ = IPAddress(DEFAULT_IP0, DEFAULT_IP1, DEFAULT_IP2, 255);

    udp_.begin(legacy_rate::UDP_LISTEN_PORT);
    agio_.begin(legacy_rate::AGIO_LISTEN_PORT);

    Serial.print("Ethernet IP: ");
    Serial.println(Ethernet.localIP());
    Serial.print("RC listen port: ");
    Serial.println(legacy_rate::UDP_LISTEN_PORT);
    Serial.print("AGIO listen port: ");
    Serial.println(legacy_rate::AGIO_LISTEN_PORT);

    return true;
}

void EthernetLink::update(EcuState& ecu) {
    if (Ethernet.linkStatus() == LinkON) {
        uint8_t data[legacy_rate::MAX_PGN_BUFFER];
        int len = udp_.parsePacket();
        if (len > 0) {
            if (len > static_cast<int>(sizeof(data))) len = sizeof(data);
            IPAddress remoteIP = udp_.remoteIP();
            // uint16_t remotePort = udp_.remotePort();

            destination_ip_ = remoteIP;

            udp_.read(data, len);
            if (processUdpPacket(ecu, data, static_cast<size_t>(len))) {
                last_rc_rx_ms_ = millis();
            }
        }

        uint8_t agio_data[legacy_rate::MAX_READ_BUFFER];
        len = agio_.parsePacket();
        if (len > 0) {
            if (len > static_cast<int>(sizeof(agio_data))) len = sizeof(agio_data);
            agio_.read(agio_data, len);
            processAgioPacket(agio_data, static_cast<size_t>(len), ecu);
        }
    }

    applyTimeout(ecu);
}

bool EthernetLink::processUdpPacket(EcuState& ecu, const uint8_t* data, size_t len) {
    if (len < 2) return false;

    const uint16_t pgn = static_cast<uint16_t>(data[1] << 8) | data[0];

    switch (pgn) {
        case legacy_rate::PGN_RATE_SETTINGS: {
            legacy_rate::RateSettingsPgn32500 p{};
            if (!legacy_rate::decode_32500(data, len, p)) return false;

            module_id_ = legacy_rate::parse_mod_id(p.mod_sensor_id);
            const uint8_t sensor_id = legacy_rate::parse_sensor_id(p.mod_sensor_id);
            if (sensor_id >= sensor_count_) sensor_count_ = sensor_id + 1;

            const bool master_on = (p.command & 0x10) != 0;
            const bool auto_on   = (p.command & 0x40) != 0;

            ecu.setRateSourceUpm(p.target_upm);
            ecu.setMeterCal(p.meter_cal);
            ecu.setManualAdjust(p.manual_pwm);

            ecu.setDrive(master_on);
            ecu.setSync(auto_on);
            ecu.setMode(auto_on ? SystemMode::AUTO : SystemMode::MANUAL);

            // ideiglenes teszt bridge
            ecu.setBaseRpm(p.target_upm);

            Serial.print("[ETH RX 32500] upm=");
            Serial.print(p.target_upm, 3);
            Serial.print(" meterCal=");
            Serial.print(p.meter_cal, 3);
            Serial.print(" cmd=0x");
            Serial.print(p.command, HEX);
            Serial.print(" manual=");
            Serial.println(p.manual_pwm);

            return true;
        }

        case legacy_rate::PGN_RELAY_SETTINGS: {
            legacy_rate::RelaySettingsPgn32501 p{};
            if (!legacy_rate::decode_32501(data, len, p)) return false;
            ecu.setRelayState(p.relay_lo, p.relay_hi);
            return true;
        }

        case legacy_rate::PGN_PID_SETTINGS: {
            legacy_rate::PidSettingsPgn32502 p{};
            if (!legacy_rate::decode_32502(data, len, p)) return false;
            ecu.setPid(p.kp, p.ki, p.kd, p.min_pwm, p.max_pwm);
            return true;
        }

        case legacy_rate::PGN_SUBNET_CHANGE:
            if (len < 6 || !legacy_rate::good_crc(data, 6)) return false;
            return true;

        case legacy_rate::PGN_MODULE_CONFIG: {
            legacy_rate::ModuleConfigPgn32700 p{};
            if (!legacy_rate::decode_32700(data, len, p)) return false;
            module_id_ = p.module_id;
            sensor_count_ = p.sensor_count > 0 ? p.sensor_count : 1;
            return true;
        }

        case legacy_rate::PGN_NETWORK_CONFIG: {
            legacy_rate::NetworkConfigPgn32702 p{};
            if (!legacy_rate::decode_32702(data, len, p)) return false;
            return true;
        }

        default:
            return false;
    }
}

void EthernetLink::processAgioPacket(const uint8_t* data, size_t len, EcuState& ecu) {
    if (len < 10) return;
    if (!(data[0] == 128 && data[1] == 129 && data[2] == 127)) return;

    if (data[3] == 201) {
        if ((data[4] == 5) && (data[5] == 201) && (data[6] == 201)) {
            ecu.setAgioSubnet(data[7], data[8], data[9]);
            Serial.print("[AGIO] subnet=");
            Serial.print(data[7]);
            Serial.print(".");
            Serial.print(data[8]);
            Serial.print(".");
            Serial.println(data[9]);
        }
    }
}

void EthernetLink::applyTimeout(EcuState& ecu) {
    const uint32_t now = millis();
    if (last_rc_rx_ms_ != 0 && (now - last_rc_rx_ms_ > RC_TIMEOUT_MS)) {
        ecu.setDrive(false);
        ecu.setSync(false);
        ecu.setBaseRpm(0.0f);
        ecu.setMode(SystemMode::OFF);
    }
}

void EthernetLink::sendStatus(const EcuState& ecu, const NodeRuntimeState& node1) {
    if (Ethernet.linkStatus() != LinkON) return;

    Serial.print("[ETH TX] to ");
    Serial.print(destination_ip_);
    Serial.println(":29999");

    const uint32_t now = millis();
    if (now - last_status_tx_ms_ < STATUS_TX_MS) return;
    last_status_tx_ms_ = now;

    uint8_t data[20]{};

    // PGN 32400
    data[0] = 144;
    data[1] = 126;
    data[2] = legacy_rate::build_mod_sensor_id(module_id_, 0);

    const uint32_t applied = static_cast<uint32_t>(ecu.baseRpm() * 1000.0f);
    data[3] = applied & 0xFF;
    data[4] = (applied >> 8) & 0xFF;
    data[5] = (applied >> 16) & 0xFF;

    data[6] = 0;
    data[7] = 0;
    data[8] = 0;

    const int16_t pwm_like = ecu.manualAdjust();
    data[9]  = pwm_like & 0xFF;
    data[10] = (pwm_like >> 8) & 0xFF;

    data[11] = 0;
    if (node1.online) data[11] |= 0b00000001;
    if (Ethernet.linkStatus() == LinkON) data[11] |= 0b01000000;

    data[12] = legacy_rate::crc(data, 12, 0);

    udp_.beginPacket(destination_ip_, legacy_rate::UDP_DEST_PORT);
    udp_.write(data, 13);
    udp_.endPacket();

    // PGN 32401
    memset(data, 0, sizeof(data));
    data[0] = 145;
    data[1] = 126;
    data[2] = module_id_;
    data[11] = 0x34;
    data[12] = 0x12;
    data[13] = 0;
    data[14] = legacy_rate::crc(data, 14, 0);

    udp_.beginPacket(destination_ip_, 29999);
    udp_.write(data, 15);
    udp_.endPacket();
}