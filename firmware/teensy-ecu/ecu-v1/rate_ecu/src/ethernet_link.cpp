#include "ethernet_link.h"

#include <string.h>

#include "blockage_output.h"
#include "config.h"
#include "custom_pgn_protocol.h"
#include "legacy_rate_protocol.h"
#include "planter_output.h"
#include "rate_math.h"
#include "runtime_config.h"

namespace {
constexpr uint32_t RC_TIMEOUT_WARN_MS = 1000;
constexpr uint32_t RC_TIMEOUT_HARD_MS = 3000;
constexpr uint32_t STATUS_TX_MS  = 200;
constexpr uint32_t MONITOR_CONFIG_TX_MS = 1000;

constexpr uint8_t DEFAULT_IP0 = 192;
constexpr uint8_t DEFAULT_IP1 = 168;
constexpr uint8_t DEFAULT_IP2 = 1;

constexpr bool ETH_DEBUG_RX        = true;
constexpr bool ETH_DEBUG_RAW_32500 = false;
constexpr bool ETH_DEBUG_RAW_32501 = false;
constexpr bool ETH_DEBUG_AGIO      = false;
constexpr bool ETH_DEBUG_TX        = false;
constexpr bool ETH_DEBUG_CUSTOM    = true;

template <typename T>
T clampValue(T value, T min_value, T max_value) {
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

int16_t clampInt16(int32_t value) {
    return static_cast<int16_t>(clampValue<int32_t>(value, -32768, 32767));
}
}

bool EthernetLink::begin() {
    module_id_ = runtime_cfg::moduleId();
    sensor_count_ = runtime_cfg::activeSensorCount();

    static uint8_t local_mac[] = {
        0x0A, 0x0B, 0x42, 0x0C, 0x0D, runtime_cfg::ipLastOctet()
    };

    IPAddress local_ip(DEFAULT_IP0, DEFAULT_IP1, DEFAULT_IP2, runtime_cfg::ipLastOctet());
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

void EthernetLink::update(EcuState* ecus, NodeManager* nodeManagers, CanBus& canBus, uint8_t ecuCount) {
    if (Ethernet.linkStatus() == LinkON) {
        uint8_t data[legacy_rate::MAX_PGN_BUFFER];
        int len = udp_.parsePacket();

        if (len > 0) {
            const IPAddress remote_ip = udp_.remoteIP();
            const uint16_t remote_port = udp_.remotePort();
            if (len > static_cast<int>(sizeof(data))) {
                len = sizeof(data);
            }

            udp_.read(data, len);

            bool refreshRcTimeout = false;
            if (processUdpPacket(ecus, nodeManagers, canBus, ecuCount, remote_ip, remote_port, data, static_cast<size_t>(len), refreshRcTimeout) &&
                refreshRcTimeout) {
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

bool EthernetLink::processUdpPacket(EcuState* ecus,
                                    NodeManager* nodeManagers,
                                    CanBus& canBus,
                                    uint8_t ecuCount,
                                    const IPAddress& remote_ip,
                                    uint16_t remote_port,
                                    const uint8_t* data,
                                    size_t len,
                                    bool& refreshRcTimeout) {
    refreshRcTimeout = false;
    if (len < 2) return false;

    const uint16_t pgn = static_cast<uint16_t>(data[1] << 8) | data[0];
    if (pgn >= custom_pgn::PGN_ECU_CFG_GET && pgn <= custom_pgn::PGN_NODE_SET_CAN_SOURCE) {
        custom_destination_ip_ = remote_ip;
        custom_destination_port_ = (remote_port == 0) ? custom_pgn::UDP_DEST_PORT : remote_port;
    }

    switch (pgn) {
        case custom_pgn::PGN_ECU_CFG_GET: {
            if (!custom_pgn::good_crc(data, len)) return false;
            sendConfigStatus(data[2], data[3], ecus, ecuCount);
            return true;
        }

        case custom_pgn::PGN_ECU_CFG_SET: {
            if (!custom_pgn::good_crc(data, len)) return false;

            const uint8_t block_id = data[2];
            const uint8_t index = data[3];

            switch (block_id) {
                case custom_pgn::CFG_BLOCK_MACHINE:
                    runtime_cfg::setActiveSensorCount(data[4]);
                    runtime_cfg::setConfiguredRowCount(data[5]);
                    runtime_cfg::setHolesPerRev(static_cast<uint16_t>(data[6] | (data[7] << 8)));
                    runtime_cfg::setUpmScale(static_cast<float>(data[8] | (data[9] << 8)) / 10.0f);
                    applyRuntimeConfigToEcus(ecus);
                    sensor_count_ = runtime_cfg::activeSensorCount();
                    break;

                case custom_pgn::CFG_BLOCK_DRIVE:
                    runtime_cfg::setTrimRpmLimit(static_cast<float>(static_cast<int16_t>(data[4] | (data[5] << 8))) / 10.0f);
                    runtime_cfg::setPositionKp(static_cast<float>(data[6] | (data[7] << 8)) / 1000.0f);
                    applyRuntimeConfigToEcus(ecus);
                    break;

                case custom_pgn::CFG_BLOCK_DIAG:
                    runtime_cfg::setDiagEnabled(data[4] != 0);
                    runtime_cfg::setDiagStreamEnabled(data[5] != 0);
                    runtime_cfg::setDiagPeriodMs(static_cast<uint16_t>(data[6]) * 10u);
                    runtime_cfg::setDiagDetailLevel(data[7]);
                    applyRuntimeConfigToEcus(ecus);
                    break;

                case custom_pgn::CFG_BLOCK_NETWORK:
                    runtime_cfg::setIpLastOctet(data[4]);
                    runtime_cfg::setModuleId(data[5]);
                    module_id_ = runtime_cfg::moduleId();
                    break;

                case custom_pgn::CFG_BLOCK_CHANNEL:
                    runtime_cfg::setDriveRatio(index, static_cast<float>(data[4] | (data[5] << 8)) / 100.0f);
                    runtime_cfg::setMotorRatio(index, static_cast<float>(data[6] | (data[7] << 8)) / 100.0f);
                    applyRuntimeConfigToEcus(ecus);
                    break;

                case custom_pgn::CFG_BLOCK_MONITOR:
                    if (index == 0) {
                        runtime_cfg::setMonitorOutputEnabled(data[4] != 0);
                        runtime_cfg::setMonitorOutputMode(data[5]);
                        runtime_cfg::setMonitorRows(data[6]);
                        runtime_cfg::setBlockageRowsPerModule(data[7]);
                    } else if (index == 1) {
                        runtime_cfg::setPlanterRowWidthCm(
                            static_cast<float>(static_cast<uint16_t>(data[4] | (data[5] << 8))) / 10.0f
                        );
                        runtime_cfg::setPlanterTargetPopulation(
                            static_cast<uint32_t>(data[6]) |
                            (static_cast<uint32_t>(data[7]) << 8) |
                            (static_cast<uint32_t>(data[8]) << 16) |
                            (static_cast<uint32_t>(data[9]) << 24)
                        );
                    } else if (index == 2) {
                        runtime_cfg::setPlanterDoublesFactor(
                            static_cast<float>(static_cast<uint16_t>(data[4] | (data[5] << 8))) / 100.0f
                        );
                        runtime_cfg::setPlanterMetric(data[6] != 0);
                        runtime_cfg::setBlockageThreshold(data[7]);
                    }
                    break;

                default:
                    break;
            }

            sendConfigStatus(block_id, index, ecus, ecuCount);
            return true;
        }

        case custom_pgn::PGN_ECU_CFG_SAVE: {
            if (!custom_pgn::good_crc(data, len)) return false;
            runtime_cfg::save();
            sendDiagStatus(ecus, ecuCount);
            return true;
        }

        case custom_pgn::PGN_ECU_CFG_LOAD: {
            if (!custom_pgn::good_crc(data, len)) return false;
            runtime_cfg::load();
            applyRuntimeConfigToEcus(ecus);
            module_id_ = runtime_cfg::moduleId();
            sensor_count_ = runtime_cfg::activeSensorCount();
            sendDiagStatus(ecus, ecuCount);
            return true;
        }

        case custom_pgn::PGN_ECU_DIAG_CONTROL: {
            if (!custom_pgn::good_crc(data, len)) return false;

            runtime_cfg::setDiagEnabled(data[2] != 0);
            runtime_cfg::setDiagStreamEnabled(data[3] != 0);
            diag_sensor_mask_ = data[4];
            diag_node_mask_ = static_cast<uint16_t>(data[5] | (data[6] << 8));
            runtime_cfg::setDiagPeriodMs(static_cast<uint16_t>(data[7]) * 10u);
            runtime_cfg::setDiagDetailLevel(data[8]);
            applyRuntimeConfigToEcus(ecus);

            sendDiagStatus(ecus, ecuCount);
            return true;
        }

        case custom_pgn::PGN_ECU_DIAG_NODE_DETAIL_REQ: {
            if (!custom_pgn::good_crc(data, len)) return false;

            const uint8_t sensor_mask = data[2];
            const uint16_t node_mask = static_cast<uint16_t>(data[3] | (data[4] << 8));
            sendDiagNodeDetails(nodeManagers, ecuCount, sensor_mask, node_mask);
            return true;
        }

        case custom_pgn::PGN_NODE_DISCOVER:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceDiscover(data[3], data[4] != 0, data[5] != 0);
            return true;

        case custom_pgn::PGN_NODE_ASSIGN:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceAssign(
                static_cast<uint32_t>(data[2]) |
                    (static_cast<uint32_t>(data[3]) << 8) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 24),
                data[6], data[7], data[8] != 0, data[9] != 0
            );
            return true;

        case custom_pgn::PGN_NODE_SAVE_CFG:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceSaveCfg(
                static_cast<uint32_t>(data[2]) |
                    (static_cast<uint32_t>(data[3]) << 8) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 24),
                data[6]
            );
            return true;

        case custom_pgn::PGN_NODE_TEST_SPIN:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceTestSpin(
                static_cast<uint32_t>(data[2]) |
                    (static_cast<uint32_t>(data[3]) << 8) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 24),
                static_cast<int16_t>(data[6] | (data[7] << 8)),
                data[8],
                data[9]
            );
            return true;

        case custom_pgn::PGN_NODE_DIAG_REQ:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceDiagReq(
                static_cast<uint32_t>(data[2]) |
                    (static_cast<uint32_t>(data[3]) << 8) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 24),
                data[6],
                data[7] != 0,
                data[8] != 0
            );
            return true;

        case custom_pgn::PGN_NODE_REBOOT:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceReboot(
                static_cast<uint32_t>(data[2]) |
                    (static_cast<uint32_t>(data[3]) << 8) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 24)
            );
            return true;

        case custom_pgn::PGN_NODE_IDENTIFY:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceIdentify(
                static_cast<uint32_t>(data[2]) |
                    (static_cast<uint32_t>(data[3]) << 8) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 24),
                data[6],
                data[7]
            );
            return true;

        case custom_pgn::PGN_NODE_CFG_READ:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceCfgRead(
                static_cast<uint32_t>(data[2]) |
                    (static_cast<uint32_t>(data[3]) << 8) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 24),
                data[6]
            );
            return true;

        case custom_pgn::PGN_NODE_SET_CAN_SOURCE:
            if (!custom_pgn::good_crc(data, len)) return false;
            canBus.sendServiceSetCanSource(
                static_cast<uint32_t>(data[2]) |
                    (static_cast<uint32_t>(data[3]) << 8) |
                    (static_cast<uint32_t>(data[4]) << 16) |
                    (static_cast<uint32_t>(data[5]) << 24),
                data[6],
                data[7],
                data[8],
                data[9] != 0
            );
            return true;

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

            const uint8_t active_sections = ecu.activeSectionCount();
            const float motor_rpm = rate_math::targetUpmToMotorRpm(
                p.target_upm, ecu.upmScale(), ecu.holesPerRev(), active_sections, ecu.combinedRatio()
            );
            ecu.setBaseRpm(master_on ? motor_rpm : 0.0f);

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
                Serial.print(" drive=");
                Serial.print(ecu.driveRatio(), 3);
                Serial.print(" motor=");
                Serial.print(ecu.motorRatio(), 3);
                Serial.print(" combined=");
                Serial.print(ecu.combinedRatio(), 3);
                Serial.print(" scale=");
                Serial.print(ecu.upmScale(), 3);
                Serial.print(" activeSections=");
                Serial.print(active_sections);
                Serial.print(" upmPerSection=");
                Serial.print(active_sections > 0 ? ((p.target_upm * ecu.upmScale()) / active_sections) : 0.0f, 3);
                Serial.print(" motorRpm=");
                Serial.println(motor_rpm, 3);
            }

            refreshRcTimeout = true;
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

            refreshRcTimeout = true;
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
            refreshRcTimeout = true;
            return true;
        }

        case legacy_rate::PGN_SUBNET_CHANGE:
            if (len < 6 || !legacy_rate::good_crc(data, 6)) return false;
            refreshRcTimeout = true;
            return true;

        case legacy_rate::PGN_MODULE_CONFIG: {
            legacy_rate::ModuleConfigPgn32700 p{};
            if (!legacy_rate::decode_32700(data, len, p)) return false;

            runtime_cfg::setModuleId(p.module_id);
            module_id_ = p.module_id;
            sensor_count_ = runtime_cfg::activeSensorCount();

            if (ETH_DEBUG_RX) {
                Serial.print("[ETH RX 32700] module=");
                Serial.print(module_id_);
                Serial.print(" sensors=");
                Serial.println(sensor_count_);
            }

            refreshRcTimeout = true;
            return true;
        }

        case legacy_rate::PGN_NETWORK_CONFIG: {
            legacy_rate::NetworkConfigPgn32702 p{};
            if (!legacy_rate::decode_32702(data, len, p)) return false;
            refreshRcTimeout = true;
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

    if (last_rc_rx_ms_ == 0) return;

    const uint32_t age_ms = now - last_rc_rx_ms_;
    if (age_ms <= RC_TIMEOUT_HARD_MS) {
        return;
    }

    for (uint8_t sensorIndex = 0; sensorIndex < ecuCount; ++sensorIndex) {
        ecus[sensorIndex].setDrive(false);
        ecus[sensorIndex].setSync(false);
        ecus[sensorIndex].setBaseRpm(0.0f);
        ecus[sensorIndex].setMode(SystemMode::OFF);
    }
}

void EthernetLink::applyRuntimeConfigToEcus(EcuState* ecus) {
    runtime_cfg::applyToEcus(ecus, cfg::MAX_SENSOR_CHANNELS);
}

void EthernetLink::sendCustomPacket(uint16_t pgn, const uint8_t payload[8]) {
    uint8_t packet[custom_pgn::PACKET_LEN];
    custom_pgn::build_packet(pgn, payload, packet);

    udp_.beginPacket(custom_destination_ip_, custom_destination_port_);
    udp_.write(packet, sizeof(packet));
    udp_.endPacket();

    if (ETH_DEBUG_CUSTOM) {
        Serial.print("[ETH TX CUSTOM] PGN ");
        Serial.print(pgn);
        Serial.print(" -> ");
        Serial.print(custom_destination_ip_);
        Serial.print(":");
        Serial.println(custom_destination_port_);
    }
}

void EthernetLink::sendConfigStatus(uint8_t block_id, uint8_t index, const EcuState* ecus, uint8_t ecuCount) {
    uint8_t payload[8]{};
    payload[0] = block_id;
    payload[1] = index;

    switch (block_id) {
        case custom_pgn::CFG_BLOCK_MACHINE: {
            payload[2] = runtime_cfg::activeSensorCount();
            payload[3] = runtime_cfg::configuredRowCount();
            const uint16_t holes = runtime_cfg::holesPerRev();
            const uint16_t scale_x10 = static_cast<uint16_t>(runtime_cfg::upmScale() * 10.0f);
            payload[4] = static_cast<uint8_t>(holes & 0xFF);
            payload[5] = static_cast<uint8_t>((holes >> 8) & 0xFF);
            payload[6] = static_cast<uint8_t>(scale_x10 & 0xFF);
            payload[7] = static_cast<uint8_t>((scale_x10 >> 8) & 0xFF);
            break;
        }

        case custom_pgn::CFG_BLOCK_DRIVE: {
            const int16_t trim_x10 = static_cast<int16_t>(runtime_cfg::trimRpmLimit() * 10.0f);
            const uint16_t kp_x1000 = static_cast<uint16_t>(runtime_cfg::positionKp() * 1000.0f);
            payload[2] = static_cast<uint8_t>(trim_x10 & 0xFF);
            payload[3] = static_cast<uint8_t>((trim_x10 >> 8) & 0xFF);
            payload[4] = static_cast<uint8_t>(kp_x1000 & 0xFF);
            payload[5] = static_cast<uint8_t>((kp_x1000 >> 8) & 0xFF);
            break;
        }

        case custom_pgn::CFG_BLOCK_DIAG:
            payload[2] = runtime_cfg::diagEnabled() ? 1 : 0;
            payload[3] = runtime_cfg::diagStreamEnabled() ? 1 : 0;
            payload[4] = static_cast<uint8_t>(runtime_cfg::diagPeriodMs() / 10u);
            payload[5] = runtime_cfg::diagDetailLevel();
            payload[6] = diag_sensor_mask_;
            payload[7] = static_cast<uint8_t>(diag_node_mask_ & 0xFF);
            break;

        case custom_pgn::CFG_BLOCK_CHANNEL:
            if (index < cfg::MAX_SENSOR_CHANNELS) {
                const uint16_t drive_x100 = static_cast<uint16_t>(runtime_cfg::driveRatio(index) * 100.0f);
                const uint16_t motor_x100 = static_cast<uint16_t>(runtime_cfg::motorRatio(index) * 100.0f);
                payload[2] = static_cast<uint8_t>(drive_x100 & 0xFF);
                payload[3] = static_cast<uint8_t>((drive_x100 >> 8) & 0xFF);
                payload[4] = static_cast<uint8_t>(motor_x100 & 0xFF);
                payload[5] = static_cast<uint8_t>((motor_x100 >> 8) & 0xFF);
                payload[6] = (index < ecuCount) ? 1 : 0;
                payload[7] = index;
            }
            break;

        case custom_pgn::CFG_BLOCK_NETWORK:
            payload[2] = runtime_cfg::ipLastOctet();
            payload[3] = runtime_cfg::moduleId();
            payload[4] = static_cast<uint8_t>(Ethernet.localIP()[3]);
            payload[5] = module_id_;
            break;

        case custom_pgn::CFG_BLOCK_MONITOR:
            if (index == 0) {
                payload[2] = runtime_cfg::monitorOutputEnabled() ? 1 : 0;
                payload[3] = runtime_cfg::monitorOutputMode();
                payload[4] = runtime_cfg::monitorRows();
                payload[5] = runtime_cfg::blockageRowsPerModule();
            } else if (index == 1) {
                const uint16_t row_width_x10 =
                    static_cast<uint16_t>(runtime_cfg::planterRowWidthCm() * 10.0f);
                const uint32_t target_population = runtime_cfg::planterTargetPopulation();
                payload[2] = static_cast<uint8_t>(row_width_x10 & 0xFF);
                payload[3] = static_cast<uint8_t>((row_width_x10 >> 8) & 0xFF);
                payload[4] = static_cast<uint8_t>(target_population & 0xFF);
                payload[5] = static_cast<uint8_t>((target_population >> 8) & 0xFF);
                payload[6] = static_cast<uint8_t>((target_population >> 16) & 0xFF);
                payload[7] = static_cast<uint8_t>((target_population >> 24) & 0xFF);
            } else if (index == 2) {
                const uint16_t doubles_x100 =
                    static_cast<uint16_t>(runtime_cfg::planterDoublesFactor() * 100.0f);
                payload[2] = static_cast<uint8_t>(doubles_x100 & 0xFF);
                payload[3] = static_cast<uint8_t>((doubles_x100 >> 8) & 0xFF);
                payload[4] = runtime_cfg::planterMetric() ? 1 : 0;
                payload[5] = runtime_cfg::blockageThreshold();
            }
            break;

        default:
            break;
    }

    sendCustomPacket(custom_pgn::PGN_ECU_CFG_STATUS, payload);

    if (ETH_DEBUG_CUSTOM) {
        Serial.print("[ETH TX CUSTOM] CFG_STATUS block=");
        Serial.print(block_id);
        Serial.print(" index=");
        Serial.println(index);
    }
}

void EthernetLink::sendDiagStatus(const EcuState* ecus, uint8_t ecuCount) {
    uint8_t payload[8]{};
    payload[0] = runtime_cfg::activeSensorCount();
    payload[1] = runtime_cfg::configuredRowCount();
    payload[2] = (Ethernet.linkStatus() == LinkON) ? 1 : 0;
    payload[3] = 1;
    payload[4] = runtime_cfg::diagEnabled() ? 1 : 0;
    payload[5] = (last_rc_rx_ms_ != 0 && (millis() - last_rc_rx_ms_ > RC_TIMEOUT_WARN_MS)) ? 1 : 0;
    const uint16_t uptime_s = static_cast<uint16_t>(millis() / 1000u);
    payload[6] = static_cast<uint8_t>(uptime_s & 0xFF);
    payload[7] = static_cast<uint8_t>((uptime_s >> 8) & 0xFF);
    sendCustomPacket(custom_pgn::PGN_ECU_DIAG_STATUS, payload);

    if (runtime_cfg::diagEnabled()) {
        for (uint8_t sensorIndex = 0; sensorIndex < ecuCount; ++sensorIndex) {
            if ((diag_sensor_mask_ & (1u << sensorIndex)) == 0) continue;
            sendDiagSensor(sensorIndex, ecus[sensorIndex]);
        }
    }
}

void EthernetLink::sendDiagSensor(uint8_t sensorIndex, const EcuState& ecu) {
    uint8_t payload[8]{};
    payload[0] = sensorIndex;
    payload[1] = static_cast<uint8_t>(ecu.mode());

    const uint16_t base_rpm_u16 = clampValue<uint32_t>(
        static_cast<uint32_t>(ecu.baseRpm() + 0.5f), 0u, 65535u
    );
    payload[2] = static_cast<uint8_t>(base_rpm_u16 & 0xFF);
    payload[3] = static_cast<uint8_t>((base_rpm_u16 >> 8) & 0xFF);

    const uint32_t target_upm_x1000 = static_cast<uint32_t>(ecu.rateSourceUpm() * 1000.0f);
    payload[4] = static_cast<uint8_t>(target_upm_x1000 & 0xFF);
    payload[5] = static_cast<uint8_t>((target_upm_x1000 >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>((target_upm_x1000 >> 16) & 0xFF);
    payload[7] = static_cast<uint8_t>((target_upm_x1000 >> 24) & 0xFF);

    sendCustomPacket(custom_pgn::PGN_ECU_DIAG_SENSOR, payload);
}

void EthernetLink::sendDiagNodeSummary(uint8_t sensorIndex, const EcuState& ecu, const NodeManager& nodeManager) {
    uint8_t payload[8]{};
    payload[0] = sensorIndex;
    payload[1] = nodeManager.onlineNodeCount(ecu);

    const uint16_t avg_rpm_u16 = clampValue<uint32_t>(static_cast<uint32_t>(nodeManager.averageActualRpm(ecu) + 0.5f), 0u, 65535u);
    const uint16_t total_rpm_u16 = clampValue<uint32_t>(static_cast<uint32_t>(nodeManager.totalActualRpm(ecu) + 0.5f), 0u, 65535u);
    const uint16_t avg_pos_u16 = nodeManager.averageActualPos(ecu);

    payload[2] = static_cast<uint8_t>(avg_rpm_u16 & 0xFF);
    payload[3] = static_cast<uint8_t>((avg_rpm_u16 >> 8) & 0xFF);
    payload[4] = static_cast<uint8_t>(total_rpm_u16 & 0xFF);
    payload[5] = static_cast<uint8_t>((total_rpm_u16 >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>(avg_pos_u16 & 0xFF);
    payload[7] = static_cast<uint8_t>((avg_pos_u16 >> 8) & 0xFF);

    sendCustomPacket(custom_pgn::PGN_ECU_DIAG_NODE_SUMMARY, payload);
}

void EthernetLink::sendDiagNodeDetailA(uint8_t sensorIndex, uint8_t nodeId, const NodeRuntimeState& nodeState) {
    uint8_t payload[8]{};
    payload[0] = sensorIndex;
    payload[1] = nodeId;
    payload[2] = nodeState.status_flags;
    payload[3] = nodeState.error_code;

    const uint16_t actual_rpm_u16 = clampValue<uint32_t>(static_cast<uint32_t>(nodeState.actual_rpm + 0.5f), 0u, 65535u);
    payload[4] = static_cast<uint8_t>(actual_rpm_u16 & 0xFF);
    payload[5] = static_cast<uint8_t>((actual_rpm_u16 >> 8) & 0xFF);
    payload[6] = static_cast<uint8_t>(nodeState.actual_pos & 0xFF);
    payload[7] = static_cast<uint8_t>((nodeState.actual_pos >> 8) & 0xFF);

    sendCustomPacket(custom_pgn::PGN_ECU_DIAG_NODE_DETAIL_A, payload);
}

void EthernetLink::sendDiagNodeDetailB(uint8_t sensorIndex, uint8_t nodeId, const NodeRuntimeState& nodeState) {
    uint8_t payload[8]{};
    payload[0] = sensorIndex;
    payload[1] = nodeId;
    payload[2] = static_cast<uint8_t>(clampValue<int32_t>(static_cast<int32_t>(nodeState.bus_voltage * 10.0f), 0, 255));
    payload[3] = static_cast<uint8_t>(clampValue<int32_t>(static_cast<int32_t>(nodeState.motor_current * 10.0f), 0, 255));
    payload[4] = nodeState.controller_temp;
    payload[5] = nodeState.motor_temp;
    payload[6] = nodeState.warning_flags;
    payload[7] = nodeState.fault_flags;

    sendCustomPacket(custom_pgn::PGN_ECU_DIAG_NODE_DETAIL_B, payload);
}

void EthernetLink::sendDiagNodeDetails(const NodeManager* nodeManagers,
                                       uint8_t sensorCount,
                                       uint8_t sensorMask,
                                       uint16_t nodeMask) {
    uint8_t maxNodeCommands = runtime_cfg::configuredRowCount();
    if (maxNodeCommands > cfg::NODE_COUNT_MAX) {
        maxNodeCommands = cfg::NODE_COUNT_MAX;
    }

    for (uint8_t sensorIndex = 0; sensorIndex < sensorCount; ++sensorIndex) {
        if ((sensorMask & (1u << sensorIndex)) == 0) continue;

        for (uint8_t nodeId = 1; nodeId <= maxNodeCommands; ++nodeId) {
            if ((nodeMask & (1u << (nodeId - 1))) == 0) continue;

            const NodeRuntimeState& nodeState = nodeManagers[sensorIndex].node(nodeId);
            sendDiagNodeDetailA(sensorIndex, nodeId, nodeState);
            sendDiagNodeDetailB(sensorIndex, nodeId, nodeState);
        }
    }
}

void EthernetLink::sendMonitorPacket(uint16_t port, const uint8_t* data, size_t len) {
    udp_.beginPacket(destination_ip_, port);
    udp_.write(data, len);
    udp_.endPacket();
}

void EthernetLink::sendPlanterOutput(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount) {
    if (!runtime_cfg::monitorOutputEnabled() ||
        runtime_cfg::monitorOutputMode() != static_cast<uint8_t>(MonitorOutputMode::PLANTER) ||
        sensorCount == 0) {
        return;
    }

    const uint8_t planterRows = clampValue<uint8_t>(
        runtime_cfg::monitorRows(), 1u,
        (runtime_cfg::configuredRowCount() < cfg::NODE_COUNT_MAX)
            ? runtime_cfg::configuredRowCount()
            : cfg::NODE_COUNT_MAX
    );

    const EcuState& ecu = ecus[0];
    const NodeManager& nodeManager = nodeManagers[0];
    uint32_t rowPopulation[cfg::NODE_COUNT_MAX]{};
    uint16_t overrideMask = 0;
    uint32_t populationSum = 0;
    uint8_t activeRows = 0;

    for (uint8_t nodeId = 1; nodeId <= planterRows; ++nodeId) {
        const bool sectionEnabled = ecu.sectionEnabled(nodeId);
        const NodeRuntimeState& nodeState = nodeManager.node(nodeId);
        const float targetRpm = sectionEnabled ? ecu.baseRpm() : 0.0f;
        rowPopulation[nodeId - 1] = planter_output::estimatePopulation(
            runtime_cfg::planterTargetPopulation(),
            targetRpm,
            nodeState.actual_rpm
        );

        if (sectionEnabled) {
            populationSum += rowPopulation[nodeId - 1];
            ++activeRows;
        } else {
            overrideMask |= static_cast<uint16_t>(1u << (nodeId - 1u));
        }
    }

    const uint32_t summaryPopulation = (activeRows > 0) ? (populationSum / activeRows) : 0u;

    const uint32_t now = millis();
    if (now - last_planter_cfg_tx_ms_ >= MONITOR_CONFIG_TX_MS) {
        last_planter_cfg_tx_ms_ = now;
        uint8_t payload[8]{};
        uint8_t packet[14]{};
        const uint16_t rowWidthX10 = static_cast<uint16_t>(runtime_cfg::planterRowWidthCm() * 10.0f);
        const uint16_t targetPopulationD10 = static_cast<uint16_t>(
            clampValue<uint32_t>(runtime_cfg::planterTargetPopulation() / 10u, 0u, 65535u)
        );
        const uint8_t doublesX100 = static_cast<uint8_t>(clampValue<uint32_t>(
            static_cast<uint32_t>(runtime_cfg::planterDoublesFactor() * 100.0f + 0.5f), 0u, 255u
        ));

        payload[0] = planterRows;
        payload[1] = 0;
        payload[2] = static_cast<uint8_t>((rowWidthX10 >> 8) & 0xFF);
        payload[3] = static_cast<uint8_t>(rowWidthX10 & 0xFF);
        payload[4] = static_cast<uint8_t>((targetPopulationD10 >> 8) & 0xFF);
        payload[5] = static_cast<uint8_t>(targetPopulationD10 & 0xFF);
        payload[6] = doublesX100;
        payload[7] = runtime_cfg::planterMetric() ? 1 : 0;
        planter_output::buildPacket(planter_output::PGN_CONFIG, payload, packet);
        sendMonitorPacket(planter_output::UDP_PORT, packet, sizeof(packet));
    }

    {
        uint8_t payload[8]{};
        uint8_t packet[14]{};
        for (uint8_t i = 0; i < 8; ++i) {
            payload[i] = static_cast<uint8_t>(clampValue<uint32_t>(rowPopulation[i] / 1000u, 0u, 255u));
        }
        planter_output::buildPacket(planter_output::PGN_POP_BY_ROW_1_8, payload, packet);
        sendMonitorPacket(planter_output::UDP_PORT, packet, sizeof(packet));
    }

    {
        uint8_t payload[8]{};
        uint8_t packet[14]{};
        for (uint8_t i = 0; i < 8; ++i) {
            payload[i] = static_cast<uint8_t>(
                clampValue<uint32_t>(rowPopulation[8 + i] / 1000u, 0u, 255u)
            );
        }
        planter_output::buildPacket(planter_output::PGN_POP_BY_ROW_9_16, payload, packet);
        sendMonitorPacket(planter_output::UDP_PORT, packet, sizeof(packet));
    }

    {
        uint8_t payload[8]{};
        uint8_t packet[14]{};
        planter_output::buildPacket(planter_output::PGN_DOUBLES, payload, packet);
        sendMonitorPacket(planter_output::UDP_PORT, packet, sizeof(packet));
        planter_output::buildPacket(planter_output::PGN_SKIPS, payload, packet);
        sendMonitorPacket(planter_output::UDP_PORT, packet, sizeof(packet));
    }

    {
        uint8_t payload[8]{};
        uint8_t packet[14]{};
        const uint16_t summaryPopulationD10 = static_cast<uint16_t>(
            clampValue<uint32_t>(summaryPopulation / 10u, 0u, 65535u)
        );
        const uint16_t singulationX10 = 1000u;
        payload[0] = static_cast<uint8_t>(summaryPopulationD10 & 0xFF);
        payload[1] = static_cast<uint8_t>((summaryPopulationD10 >> 8) & 0xFF);
        payload[2] = 0;
        payload[3] = 0;
        payload[4] = 0;
        payload[5] = 0;
        payload[6] = static_cast<uint8_t>(singulationX10 & 0xFF);
        payload[7] = static_cast<uint8_t>((singulationX10 >> 8) & 0xFF);
        planter_output::buildPacket(planter_output::PGN_SUMMARY, payload, packet);
        sendMonitorPacket(planter_output::UDP_PORT, packet, sizeof(packet));
    }

    {
        uint8_t payload[8]{};
        uint8_t packet[14]{};
        for (uint8_t rowIndex = 0; rowIndex < planterRows; ++rowIndex) {
            const uint8_t nodeId = rowIndex + 1;
            const uint8_t status = planter_output::estimateRowStatus(
                ecu.sectionEnabled(nodeId),
                nodeManager.node(nodeId),
                ecu.sectionEnabled(nodeId) ? ecu.baseRpm() : 0.0f
            );
            const uint8_t payloadIndex = rowIndex / 4u;
            const uint8_t shift = static_cast<uint8_t>((rowIndex % 4u) * 2u);
            payload[payloadIndex] |= static_cast<uint8_t>((status & 0x03u) << shift);
        }
        payload[4] = ++planter_feedback_counter_;
        payload[5] = static_cast<uint8_t>(overrideMask & 0xFF);
        payload[6] = static_cast<uint8_t>((overrideMask >> 8) & 0xFF);
        payload[7] = 0;
        planter_output::buildPacket(planter_output::PGN_ROW_STATUS, payload, packet);
        sendMonitorPacket(planter_output::UDP_PORT, packet, sizeof(packet));
    }
}

void EthernetLink::sendBlockageOutput(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount) {
    if (!runtime_cfg::monitorOutputEnabled() ||
        runtime_cfg::monitorOutputMode() != static_cast<uint8_t>(MonitorOutputMode::BLOCKAGE) ||
        sensorCount == 0) {
        return;
    }

    const uint8_t configuredRows = runtime_cfg::configuredRowCount();
    const uint8_t totalRows = clampValue<uint8_t>(
        runtime_cfg::monitorRows(), 1u,
        static_cast<uint8_t>(min<uint16_t>(100u, static_cast<uint16_t>(sensorCount) * configuredRows))
    );
    const uint8_t rowsPerModule = max<uint8_t>(1u, runtime_cfg::blockageRowsPerModule());

    for (uint8_t globalRow = 0; globalRow < totalRows; ++globalRow) {
        const uint8_t sensorIndex = globalRow / configuredRows;
        const uint8_t nodeId = static_cast<uint8_t>((globalRow % configuredRows) + 1u);
        if (sensorIndex >= sensorCount || nodeId > cfg::NODE_COUNT_MAX) {
            break;
        }

        const EcuState& ecu = ecus[sensorIndex];
        const NodeRuntimeState& nodeState = nodeManagers[sensorIndex].node(nodeId);
        const float targetRpm = ecu.sectionEnabled(nodeId) ? ecu.baseRpm() : 0.0f;
        const uint8_t rateByte = blockage_output::estimateRateByte(
            runtime_cfg::planterTargetPopulation(),
            targetRpm,
            nodeState.actual_rpm
        );
        const uint8_t moduleId = static_cast<uint8_t>(globalRow / rowsPerModule);
        const uint8_t rowInModule = static_cast<uint8_t>(globalRow % rowsPerModule);
        uint8_t packet[5]{};
        blockage_output::buildPacket32100(moduleId, rowInModule, rateByte, packet);
        sendMonitorPacket(blockage_output::UDP_PORT, packet, sizeof(packet));
    }
}

void EthernetLink::sendMonitorOutput(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount) {
    if (!runtime_cfg::monitorOutputEnabled()) {
        return;
    }

    switch (static_cast<MonitorOutputMode>(runtime_cfg::monitorOutputMode())) {
        case MonitorOutputMode::PLANTER:
            sendPlanterOutput(ecus, nodeManagers, sensorCount);
            break;
        case MonitorOutputMode::BLOCKAGE:
            sendBlockageOutput(ecus, nodeManagers, sensorCount);
            break;
        case MonitorOutputMode::OFF:
        default:
            break;
    }
}

void EthernetLink::sendCustomDiagStream(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount) {
    if (!runtime_cfg::diagEnabled() || !runtime_cfg::diagStreamEnabled()) return;

    const uint32_t now = millis();
    const uint16_t period_ms = runtime_cfg::diagPeriodMs();
    if (now - last_custom_diag_tx_ms_ < period_ms) return;
    last_custom_diag_tx_ms_ = now;

    sendDiagStatus(ecus, sensorCount);

    for (uint8_t sensorIndex = 0; sensorIndex < sensorCount; ++sensorIndex) {
        if ((diag_sensor_mask_ & (1u << sensorIndex)) == 0) continue;
        sendDiagNodeSummary(sensorIndex, ecus[sensorIndex], nodeManagers[sensorIndex]);
    }
}

void EthernetLink::sendStatus(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount) {
    if (Ethernet.linkStatus() != LinkON) return;

    const uint32_t now = millis();
    if (now - last_status_tx_ms_ < STATUS_TX_MS) {
        sendCustomDiagStream(ecus, nodeManagers, sensorCount);
        return;
    }
    last_status_tx_ms_ = now;

    const EcuState& ecu = ecus[0];
    uint8_t data[20]{};

    data[0] = 144;
    data[1] = 126;
    data[2] = legacy_rate::build_mod_sensor_id(module_id_, 0);

    const float actualUpm0 = rate_math::summedMotorRpmToFeedbackUpm(
        nodeManagers[0].totalActualRpm(ecu), ecu.upmScale(), ecu.holesPerRev(), ecu.combinedRatio()
    );
    const uint32_t applied = static_cast<uint32_t>(actualUpm0 * 1000.0f);
    data[3] = static_cast<uint8_t>(applied & 0xFF);
    data[4] = static_cast<uint8_t>((applied >> 8) & 0xFF);
    data[5] = static_cast<uint8_t>((applied >> 16) & 0xFF);

    data[6] = 0;
    data[7] = 0;
    data[8] = 0;

    const int16_t pwm_like = ecu.manualAdjust();
    data[9]  = static_cast<uint8_t>(pwm_like & 0xFF);
    data[10] = static_cast<uint8_t>((pwm_like >> 8) & 0xFF);

    data[11] = 0;
    if (nodeManagers[0].hasOnlineNode(ecu)) {
        data[11] |= 0b00000001;
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

    for (uint8_t sensorId = 1; sensorId < sensorCount; ++sensorId) {
        memset(data, 0, sizeof(data));
        data[0] = 144;
        data[1] = 126;
        data[2] = legacy_rate::build_mod_sensor_id(module_id_, sensorId);

        const EcuState& sensorEcu = ecus[sensorId];
        const float actualUpm = rate_math::summedMotorRpmToFeedbackUpm(
            nodeManagers[sensorId].totalActualRpm(sensorEcu),
            sensorEcu.upmScale(),
            sensorEcu.holesPerRev(),
            sensorEcu.combinedRatio()
        );
        const uint32_t appliedSensor = static_cast<uint32_t>(actualUpm * 1000.0f);
        data[3] = static_cast<uint8_t>(appliedSensor & 0xFF);
        data[4] = static_cast<uint8_t>((appliedSensor >> 8) & 0xFF);
        data[5] = static_cast<uint8_t>((appliedSensor >> 16) & 0xFF);

        const int16_t pwm_like_sensor = sensorEcu.manualAdjust();
        data[9]  = static_cast<uint8_t>(pwm_like_sensor & 0xFF);
        data[10] = static_cast<uint8_t>((pwm_like_sensor >> 8) & 0xFF);

        if (nodeManagers[sensorId].hasOnlineNode(sensorEcu)) {
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

    memset(data, 0, sizeof(data));
    data[0] = 145;
    data[1] = 126;
    data[2] = module_id_;
    data[11] = static_cast<uint8_t>(ino_id_ & 0xFF);
    data[12] = static_cast<uint8_t>((ino_id_ >> 8) & 0xFF);
    data[13] = 0;
    data[14] = legacy_rate::crc(data, 14, 0);

    udp_.beginPacket(destination_ip_, legacy_rate::UDP_DEST_PORT);
    udp_.write(data, 15);
    udp_.endPacket();

    sendMonitorOutput(ecus, nodeManagers, sensorCount);
    sendCustomDiagStream(ecus, nodeManagers, sensorCount);

    if (ETH_DEBUG_TX) {
        Serial.print("[ETH TX] 32400/32401 -> ");
        Serial.print(destination_ip_);
        Serial.print(":");
        Serial.println(legacy_rate::UDP_DEST_PORT);
    }
}
