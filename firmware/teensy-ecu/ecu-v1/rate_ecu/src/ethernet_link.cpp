#include "ethernet_link.h"

#include "legacy_rate_protocol.h"
#include "config.h"
#include "rate_math.h"

namespace {
constexpr uint32_t RC_TIMEOUT_MS = 1000;
constexpr uint32_t STATUS_TX_MS  = 200;

constexpr uint8_t DEFAULT_IP0 = 192;
constexpr uint8_t DEFAULT_IP1 = 168;
constexpr uint8_t DEFAULT_IP2 = 1;
constexpr uint8_t DEFAULT_IP3 = 200;

// debug kapcsolók
constexpr bool ETH_DEBUG_RX       = true;
constexpr bool ETH_DEBUG_RAW_32500 = false;
constexpr bool ETH_DEBUG_RAW_32501 = false;
constexpr bool ETH_DEBUG_AGIO     = false;
constexpr bool ETH_DEBUG_TX       = false;
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

void EthernetLink::update(EcuState* ecus, uint8_t ecuCount) {
    if (Ethernet.linkStatus() == LinkON) {
        uint8_t data[legacy_rate::MAX_PGN_BUFFER];
        int len = udp_.parsePacket();

        if (len > 0) {
            if (len > static_cast<int>(sizeof(data))) {
                len = sizeof(data);
            }

            udp_.read(data, len);

            if (processUdpPacket(ecus, ecuCount, data, static_cast<size_t>(len))) {
                last_rc_rx_ms_ = millis();
            }
        }

        uint8_t agio_data[legacy_rate::MAX_READ_BUFFER];
        len = agio_.parsePacket();

        if (len > 0) {
            if (len > static_cast<int>(sizeof(agio_data))) {
                len = sizeof(agio_data);
            }

            agio_.read(agio_data, len);
            processAgioPacket(agio_data, static_cast<size_t>(len), ecus, ecuCount);
        }
    }

    applyTimeout(ecus, ecuCount);
}

bool EthernetLink::processUdpPacket(EcuState* ecus, uint8_t ecuCount, const uint8_t* data, size_t len) {
    if (len < 2) return false;

    const uint16_t pgn = static_cast<uint16_t>(data[1] << 8) | data[0];

    switch (pgn) {
        case legacy_rate::PGN_RATE_SETTINGS: {
            if (ETH_DEBUG_RAW_32500) {
                Serial.print("[ETH RX 32500 RAW] ");
                for (size_t i = 0; i < len; i++) {
                    if (data[i] < 16) Serial.print('0');
                    Serial.print(data[i], HEX);
                    Serial.print(' ');
                }
                Serial.println();
            }

            legacy_rate::RateSettingsPgn32500 p{};
            if (!legacy_rate::decode_32500(data, len, p)) return false;

            const uint8_t incoming_module_id = legacy_rate::parse_mod_id(p.mod_sensor_id);
            const uint8_t sensor_id = legacy_rate::parse_sensor_id(p.mod_sensor_id);

            // csak a saját modulnak szóló packetet fogadjuk
            if (incoming_module_id != module_id_) {
                return false;
            }

            if (sensor_id >= ecuCount) {
                return false;
            }

            EcuState& ecu = ecus[sensor_id];

            const bool master_on = (p.command & 0x10) != 0;
            const bool auto_on   = (p.command & 0x40) != 0;

            ecu.setRateSourceUpm(p.target_upm);
            ecu.setMeterCal(p.meter_cal);
            ecu.setManualAdjust(p.manual_pwm);

            ecu.setDrive(master_on);
            ecu.setSync(auto_on);
            ecu.setMode(auto_on ? SystemMode::AUTO : SystemMode::MANUAL);

            const uint8_t active_sections = rate_math::countActiveSections(
                ecu.sectionMask(), cfg::PGN_SECTION_BIT_COUNT
            );
            const float disc_rpm = rate_math::upmToDiscRpm(
                p.target_upm, ecu.holesPerRev(), active_sections
            );
            ecu.setBaseRpm(master_on ? disc_rpm : 0.0f);

            if (ETH_DEBUG_RX) {
                Serial.print("[ETH RX 32500] upm=");
                Serial.print(p.target_upm, 3);
                Serial.print(" meterCal=");
                Serial.print(p.meter_cal, 3);
                Serial.print(" cmd=0x");
                Serial.print(p.command, HEX);
                Serial.print(" manual=");
                Serial.print(p.manual_pwm);
                Serial.print(" holes=");
                Serial.print(ecu.holesPerRev());
                Serial.print(" activeSections=");
                Serial.print(active_sections);
                Serial.print(" upmPerSection=");
                Serial.print(active_sections > 0 ? (p.target_upm / active_sections) : 0.0f, 3);
                Serial.print(" discRpm=");
                Serial.println(disc_rpm, 3);
            }

            return true;
        }

        case legacy_rate::PGN_RELAY_SETTINGS: {
            if (ETH_DEBUG_RAW_32501) {
                Serial.print("[ETH RX 32501 RAW] ");
                for (size_t i = 0; i < len; i++) {
                    if (data[i] < 16) Serial.print('0');
                    Serial.print(data[i], HEX);
                    Serial.print(' ');
                }
                Serial.println();
            }

            legacy_rate::RelaySettingsPgn32501 p{};
            if (!legacy_rate::decode_32501(data, len, p)) return false;

            const uint8_t incoming_module_id = legacy_rate::parse_mod_id(p.mod_sensor_id);

            if (incoming_module_id != module_id_) {
                return false;
            }


            // AOG szakaszolás -> 6 bites maszkként használjuk.
            const uint16_t section_mask = static_cast<uint16_t>(
                p.relay_lo | (static_cast<uint16_t>(p.relay_hi) << 8)
            );
            for (uint8_t sensorIndex = 0; sensorIndex < ecuCount; ++sensorIndex) {
                ecus[sensorIndex].setRelayState(p.relay_lo, p.relay_hi);
                ecus[sensorIndex].setSectionMask(section_mask);
            }

            if (ETH_DEBUG_RX) {
                Serial.print("[ETH RX 32501] relayLo=0x");
                Serial.print(p.relay_lo, HEX);
                Serial.print(" relayHi=0x");
                Serial.print(p.relay_hi, HEX);
                Serial.print(" sectionMask=0b");
                Serial.println(static_cast<uint32_t>(section_mask), BIN);
            }
            return true;
        }

        case legacy_rate::PGN_PID_SETTINGS: {
            legacy_rate::PidSettingsPgn32502 p{};
            if (!legacy_rate::decode_32502(data, len, p)) return false;

            const uint8_t incoming_module_id = legacy_rate::parse_mod_id(p.mod_sensor_id);
            const uint8_t sensor_id = legacy_rate::parse_sensor_id(p.mod_sensor_id);

            if (incoming_module_id != module_id_) {
                return false;
            }

            if (sensor_id >= ecuCount) {
                return false;
            }

            ecus[sensor_id].setPid(p.kp, p.ki, p.kd, p.min_pwm, p.max_pwm);
            return true;
        }

        case legacy_rate::PGN_SUBNET_CHANGE:
            if (len < 6 || !legacy_rate::good_crc(data, 6)) return false;
            return true;

        case legacy_rate::PGN_MODULE_CONFIG: {
            legacy_rate::ModuleConfigPgn32700 p{};
            if (!legacy_rate::decode_32700(data, len, p)) return false;

            module_id_ = p.module_id;
            sensor_count_ = ecuCount;

            if (ETH_DEBUG_RX) {
                Serial.print("[ETH RX 32700] module=");
                Serial.print(module_id_);
                Serial.print(" sensors=");
                Serial.println(sensor_count_);
            }

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

void EthernetLink::processAgioPacket(const uint8_t* data, size_t len, EcuState* ecus, uint8_t ecuCount) {
    if (len < 10) return;
    if (!(data[0] == 128 && data[1] == 129 && data[2] == 127)) return;

    if (data[3] == 201) {
        if ((data[4] == 5) && (data[5] == 201) && (data[6] == 201)) {
            for (uint8_t sensorIndex = 0; sensorIndex < ecuCount; ++sensorIndex) {
                ecus[sensorIndex].setAgioSubnet(data[7], data[8], data[9]);
            }

            if (ETH_DEBUG_AGIO) {
                Serial.print("[AGIO] subnet=");
                Serial.print(data[7]);
                Serial.print(".");
                Serial.print(data[8]);
                Serial.print(".");
                Serial.println(data[9]);
            }
        }
    }
}

void EthernetLink::applyTimeout(EcuState* ecus, uint8_t ecuCount) {
    const uint32_t now = millis();

    if (last_rc_rx_ms_ != 0 && (now - last_rc_rx_ms_ > RC_TIMEOUT_MS)) {
        for (uint8_t sensorIndex = 0; sensorIndex < ecuCount; ++sensorIndex) {
            ecus[sensorIndex].setDrive(false);
            ecus[sensorIndex].setSync(false);
            ecus[sensorIndex].setBaseRpm(0.0f);
            ecus[sensorIndex].setMode(SystemMode::OFF);
        }
    }
}

void EthernetLink::sendStatus(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount) {
    if (Ethernet.linkStatus() != LinkON) return;

    const uint32_t now = millis();
    if (now - last_status_tx_ms_ < STATUS_TX_MS) return;
    last_status_tx_ms_ = now;

    const EcuState& ecu = ecus[0];
    const NodeRuntimeState& node1 = nodeManagers[0].node(1);

    uint8_t data[20]{};

    // ---------------------------
    // PGN 32400 - rate info
    // ---------------------------
    data[0] = 144;
    data[1] = 126;

    // modul/szenzor azonosító
    data[2] = legacy_rate::build_mod_sensor_id(module_id_, 0);

    // applied rate (1000x)
    const uint32_t applied = static_cast<uint32_t>(ecu.rateSourceUpm() * 1000.0f);
    data[3] = static_cast<uint8_t>(applied & 0xFF);
    data[4] = static_cast<uint8_t>((applied >> 8) & 0xFF);
    data[5] = static_cast<uint8_t>((applied >> 16) & 0xFF);

    // accumulated quantity
    data[6] = 0;
    data[7] = 0;
    data[8] = 0;

    // pwm / manual adjust
    const int16_t pwm_like = ecu.manualAdjust();
    data[9]  = static_cast<uint8_t>(pwm_like & 0xFF);
    data[10] = static_cast<uint8_t>((pwm_like >> 8) & 0xFF);

    // status byte
    data[11] = 0;

    // Valós szenzor online állapot: ettől lesz zöld, ha a motor modul tényleg látszik.
    if (node1.online) {
        data[11] |= 0b00000001;
    }

    // ethernet connected
    if (Ethernet.linkStatus() == LinkON) {
        data[11] |= 0b01000000;
    }

    // pin config ok
    if (good_pins_) {
        data[11] |= 0b10000000;
    }

    data[12] = legacy_rate::crc(data, 12, 0);

    udp_.beginPacket(destination_ip_, legacy_rate::UDP_DEST_PORT);
    udp_.write(data, 13);
    udp_.endPacket();

    for (uint8_t sensorId = 1; sensorId < sensorCount; ++sensorId) {
        memset(data, 0, sizeof(data));
        data[0] = 144;
        data[1] = 126;
        data[2] = legacy_rate::build_mod_sensor_id(module_id_, sensorId);

        const EcuState& sensorEcu = ecus[sensorId];
        const NodeRuntimeState& sensorNode1 = nodeManagers[sensorId].node(1);
        const uint32_t appliedSensor = static_cast<uint32_t>(sensorEcu.rateSourceUpm() * 1000.0f);
        data[3] = static_cast<uint8_t>(appliedSensor & 0xFF);
        data[4] = static_cast<uint8_t>((appliedSensor >> 8) & 0xFF);
        data[5] = static_cast<uint8_t>((appliedSensor >> 16) & 0xFF);

        const int16_t pwm_like_sensor = sensorEcu.manualAdjust();
        data[9]  = static_cast<uint8_t>(pwm_like_sensor & 0xFF);
        data[10] = static_cast<uint8_t>((pwm_like_sensor >> 8) & 0xFF);

        // The Rate App only distinguishes sensor 0 and sensor 1+ in PGN32400 status bits.
        if (sensorNode1.online) {
            data[11] |= 0b00000010;
        }
        if (Ethernet.linkStatus() == LinkON) {
            data[11] |= 0b01000000;
        }
        if (good_pins_) {
            data[11] |= 0b10000000;
        }

        data[12] = legacy_rate::crc(data, 12, 0);

        udp_.beginPacket(destination_ip_, legacy_rate::UDP_DEST_PORT);
        udp_.write(data, 13);
        udp_.endPacket();
    }

    // ---------------------------
    // PGN 32401 - analog/module info
    // ---------------------------
    memset(data, 0, sizeof(data));

    data[0] = 145;
    data[1] = 126;
    data[2] = module_id_;

    // analog 0..3
    data[3]  = 0;
    data[4]  = 0;
    data[5]  = 0;
    data[6]  = 0;
    data[7]  = 0;
    data[8]  = 0;
    data[9]  = 0;
    data[10] = 0;

    // InoID low/high
    data[11] = static_cast<uint8_t>(ino_id_ & 0xFF);
    data[12] = static_cast<uint8_t>((ino_id_ >> 8) & 0xFF);

    // status
    data[13] = 0;

    data[14] = legacy_rate::crc(data, 14, 0);

    udp_.beginPacket(destination_ip_, legacy_rate::UDP_DEST_PORT);
    udp_.write(data, 15);
    udp_.endPacket();

    if (ETH_DEBUG_TX) {
        Serial.print("[ETH TX] 32400/32401 -> ");
        Serial.print(destination_ip_);
        Serial.print(":");
        Serial.println(legacy_rate::UDP_DEST_PORT);
    }
}
