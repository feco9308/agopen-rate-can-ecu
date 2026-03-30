#include "node_manager.h"
#include <Arduino.h>

void NodeManager::begin() {
    for (auto& n : nodes_) {
        n = NodeRuntimeState{};
    }
}

void NodeManager::update() {
    const uint32_t now = millis();

    for (uint8_t i = 1; i <= cfg::NODE_COUNT_MAX; i++) {
        if (nodes_[i].online && (now - nodes_[i].last_seen_ms > cfg::NODE_TIMEOUT_MS)) {
            nodes_[i].online = false;
        }
    }
}

void NodeManager::onStatusFrame(uint8_t nodeId, const NodeStatusFastFrame& frame) {
    if (nodeId == 0 || nodeId > cfg::NODE_COUNT_MAX) return;

    auto& n = nodes_[nodeId];
    n.online = true;
    n.last_seen_ms = millis();
    n.status_flags = frame.status_flags;
    n.error_code = frame.error_code;
    n.actual_rpm = frame.actual_rpm_x10 / 10.0f;
    n.actual_pos = frame.actual_pos_u16;
    n.alive_counter = frame.alive_counter;
    n.sync_error = frame.sync_error_x256rev;
}

const NodeRuntimeState& NodeManager::node(uint8_t nodeId) const {
    return nodes_[nodeId];
}

NodeRuntimeState& NodeManager::node(uint8_t nodeId) {
    return nodes_[nodeId];
}