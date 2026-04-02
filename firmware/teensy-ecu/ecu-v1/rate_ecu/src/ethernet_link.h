#pragma once

#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include "ecu_state.h"
#include "node_manager.h"

class EthernetLink {
public:
    bool begin();
    void update(EcuState* ecus, uint8_t ecuCount);
    void sendStatus(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount);

private:
    bool processUdpPacket(EcuState* ecus, uint8_t ecuCount, const uint8_t* data, size_t len);
    void processAgioPacket(const uint8_t* data, size_t len, EcuState* ecus, uint8_t ecuCount);
    void applyTimeout(EcuState* ecus, uint8_t ecuCount);

private:
    EthernetUDP udp_;
    EthernetUDP agio_;

    IPAddress destination_ip_{192, 168, 1, 255};
    uint32_t last_rc_rx_ms_ = 0;
    uint32_t last_status_tx_ms_ = 0;

    uint8_t module_id_ = 0;
    uint8_t sensor_count_ = cfg::ACTIVE_SENSOR_CHANNELS;

    uint16_t ino_id_ = 0x0001;
    bool good_pins_ = true;
};
