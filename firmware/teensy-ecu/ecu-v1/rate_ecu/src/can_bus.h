#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>
#include "config.h"

class EcuState;
class SyncAxis;
class NodeManager;

class CanBus {
public:
    bool begin();
    void update(NodeManager* nodeManagers, uint8_t channelCount);

    void sendGlobalControl(uint8_t channelIndex, const EcuState& ecu, const SyncAxis& syncAxis);
    void sendNodeCommands(uint8_t channelIndex,
                          const EcuState& ecu,
                          const SyncAxis& syncAxis,
                          const NodeManager& nodeManager);
    void sendEstop(uint8_t channelIndex, uint8_t reason);
    void sendServiceDiscover(uint8_t response_delay_slots, bool request_uid, bool request_cfg);
    void sendServiceAssign(uint32_t uid32, uint8_t sensor_channel, uint8_t node_id, bool apply_now, bool save_now);
    void sendServiceSaveCfg(uint32_t uid32, uint8_t scope = 0);
    void sendServiceTestSpin(uint32_t uid32, int16_t rpm_x10, uint8_t duration_s, uint8_t mode);
    void sendServiceDiagReq(uint32_t uid32, uint8_t detail_level, bool include_runtime, bool include_cfg);
    void sendServiceReboot(uint32_t uid32);
    void sendServiceIdentify(uint32_t uid32, uint8_t mode, uint8_t duration_s);
    void sendServiceCfgRead(uint32_t uid32, uint8_t block_id);
    void sendServiceSetCanSource(uint32_t uid32,
                                 uint8_t sensor_channel,
                                 uint8_t node_id,
                                 uint8_t can_profile_index,
                                 bool save_now);

private:
    void handleFrame(const CAN_message_t& msg, NodeManager* nodeManagers, uint8_t channelCount);
    void sendServiceFrame(uint32_t id, const uint8_t payload[8]);

private:
    uint8_t sequence_[cfg::MAX_SENSOR_CHANNELS]{};
    uint8_t node_sequence_[cfg::MAX_SENSOR_CHANNELS]{};
};
