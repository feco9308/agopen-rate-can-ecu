#pragma once

#include <Arduino.h>
#include <NativeEthernet.h>
#include <NativeEthernetUdp.h>

#include "can_bus.h"
#include "custom_pgn_protocol.h"
#include "ecu_state.h"
#include "node_manager.h"

class EthernetLink {
public:
    bool begin();
    void update(EcuState* ecus, NodeManager* nodeManagers, CanBus& canBus, uint8_t ecuCount);
    void sendStatus(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount);

private:
    bool processUdpPacket(EcuState* ecus,
                          NodeManager* nodeManagers,
                          CanBus& canBus,
                          uint8_t ecuCount,
                          const IPAddress& remote_ip,
                          uint16_t remote_port,
                          const uint8_t* data,
                          size_t len,
                          bool& refreshRcTimeout);
    void processAgioPacket(const uint8_t* data, size_t len, EcuState* ecus, uint8_t ecuCount);
    void applyTimeout(EcuState* ecus, uint8_t ecuCount);
    void applyRuntimeConfigToEcus(EcuState* ecus);
    void sendCustomPacket(uint16_t pgn, const uint8_t payload[8]);
    void sendConfigStatus(uint8_t block_id, uint8_t index, const EcuState* ecus, uint8_t ecuCount);
    void sendDiagStatus(const EcuState* ecus, uint8_t ecuCount);
    void sendDiagSensor(uint8_t sensorIndex, const EcuState& ecu);
    void sendDiagNodeSummary(uint8_t sensorIndex, const EcuState& ecu, const NodeManager& nodeManager);
    void sendDiagNodeDetailA(uint8_t sensorIndex, uint8_t nodeId, const NodeRuntimeState& nodeState);
    void sendDiagNodeDetailB(uint8_t sensorIndex, uint8_t nodeId, const NodeRuntimeState& nodeState);
    void sendDiagNodeDetails(const NodeManager* nodeManagers,
                             uint8_t sensorCount,
                             uint8_t sensorMask,
                             uint16_t nodeMask);
    void sendMonitorPacket(uint16_t port, const uint8_t* data, size_t len);
    void sendPlanterOutput(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount);
    void sendBlockageOutput(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount);
    void sendMonitorOutput(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount);
    void sendCustomDiagStream(const EcuState* ecus, const NodeManager* nodeManagers, uint8_t sensorCount);

private:
    EthernetUDP udp_;
    EthernetUDP agio_;

    IPAddress destination_ip_{192, 168, 1, 255};
    IPAddress custom_destination_ip_{192, 168, 1, 255};
    uint32_t last_rc_rx_ms_ = 0;
    uint32_t last_status_tx_ms_ = 0;
    uint32_t last_custom_diag_tx_ms_ = 0;
    uint32_t last_planter_cfg_tx_ms_ = 0;
    uint16_t custom_destination_port_ = custom_pgn::UDP_DEST_PORT;
    uint8_t planter_feedback_counter_ = 0;

    uint8_t module_id_ = 0;
    uint8_t sensor_count_ = cfg::DEFAULT_ACTIVE_SENSOR_CHANNELS;
    uint8_t diag_sensor_mask_ = 0x0F;
    uint16_t diag_node_mask_ = 0xFFFF;

    uint16_t ino_id_ = 0x0001;
    bool good_pins_ = true;
};
