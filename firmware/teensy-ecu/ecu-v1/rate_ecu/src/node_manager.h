#pragma once

#include "config.h"

class EcuState;

class NodeManager {
public:
    void begin();
    void update();

    void onStatusFrame(uint8_t nodeId, const NodeStatusFastFrame& frame);
    void onDiagFrame(uint8_t nodeId, const NodeDiagFrame& frame);

    const NodeRuntimeState& node(uint8_t nodeId) const;
    NodeRuntimeState& node(uint8_t nodeId);
    uint8_t onlineNodeCount(const EcuState& ecu) const;
    bool hasOnlineNode(const EcuState& ecu) const;
    float averageActualRpm(const EcuState& ecu) const;
    float totalActualRpm(const EcuState& ecu) const;
    uint16_t averageActualPos(const EcuState& ecu) const;

private:
    NodeRuntimeState nodes_[cfg::NODE_COUNT_MAX + 1];
};
