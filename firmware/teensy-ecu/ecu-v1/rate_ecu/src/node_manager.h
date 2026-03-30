#pragma once

#include "config.h"

class NodeManager {
public:
    void begin();
    void update();

    void onStatusFrame(uint8_t nodeId, const NodeStatusFastFrame& frame);

    const NodeRuntimeState& node(uint8_t nodeId) const;
    NodeRuntimeState& node(uint8_t nodeId);

private:
    NodeRuntimeState nodes_[cfg::NODE_COUNT_MAX + 1];
};