#include "node_manager.h"
#include <Arduino.h>
#include <math.h>
#include "ecu_state.h"
#include "runtime_config.h"

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
    n.actual_rpm = static_cast<float>(frame.actual_rpm_u16);
    n.actual_pos = frame.actual_pos_u16;
    n.alive_counter = frame.alive_counter;
    n.sync_error = frame.sync_error_x256rev;
}

void NodeManager::onDiagFrame(uint8_t nodeId, const NodeDiagFrame& frame) {
    if (nodeId == 0 || nodeId > cfg::NODE_COUNT_MAX) return;

    auto& n = nodes_[nodeId];
    n.bus_voltage = frame.bus_voltage_x10 / 10.0f;
    n.motor_current = frame.motor_current_x10 / 10.0f;
    n.controller_temp = frame.controller_temp_c;
    n.motor_temp = frame.motor_temp_c;
    n.fault_flags = frame.fault_flags;
    n.warning_flags = frame.warning_flags;
}

void NodeManager::onPresenceFrame(uint8_t nodeId, const NodePresenceFrame& frame) {
    if (nodeId == 0 || nodeId > cfg::NODE_COUNT_MAX) return;

    auto& n = nodes_[nodeId];
    n.seed_flags = frame.seed_flags;
    n.blockage_pct = frame.blockage_pct;
    n.slowdown_pct = frame.slowdown_pct;
    n.skip_pct = frame.skip_pct;
    n.double_pct = frame.double_pct;
    n.singulation_pct = frame.singulation_pct;
    n.population_x1k = frame.population_x1k_u16;
}

const NodeRuntimeState& NodeManager::node(uint8_t nodeId) const {
    return nodes_[nodeId];
}

NodeRuntimeState& NodeManager::node(uint8_t nodeId) {
    return nodes_[nodeId];
}

uint8_t NodeManager::onlineNodeCount(const EcuState& ecu) const {
    uint8_t count = 0;
    const uint8_t configured_rows = runtime_cfg::configuredRowCount();
    const uint8_t maxTrackedNodes = (configured_rows < cfg::NODE_COUNT_MAX)
        ? configured_rows
        : cfg::NODE_COUNT_MAX;

    for (uint8_t nodeId = 1; nodeId <= maxTrackedNodes; ++nodeId) {
        if (ecu.sectionEnabled(nodeId) && nodes_[nodeId].online) {
            ++count;
        }
    }

    return count;
}

bool NodeManager::hasOnlineNode(const EcuState& ecu) const {
    return onlineNodeCount(ecu) > 0;
}

float NodeManager::averageActualRpm(const EcuState& ecu) const {
    float totalRpm = 0.0f;
    uint8_t count = 0;
    const uint8_t configured_rows = runtime_cfg::configuredRowCount();
    const uint8_t maxTrackedNodes = (configured_rows < cfg::NODE_COUNT_MAX)
        ? configured_rows
        : cfg::NODE_COUNT_MAX;

    for (uint8_t nodeId = 1; nodeId <= maxTrackedNodes; ++nodeId) {
        if (ecu.sectionEnabled(nodeId) && nodes_[nodeId].online) {
            totalRpm += nodes_[nodeId].actual_rpm;
            ++count;
        }
    }

    return (count > 0) ? (totalRpm / static_cast<float>(count)) : 0.0f;
}

float NodeManager::totalActualRpm(const EcuState& ecu) const {
    float totalRpm = 0.0f;
    const uint8_t configured_rows = runtime_cfg::configuredRowCount();
    const uint8_t maxTrackedNodes = (configured_rows < cfg::NODE_COUNT_MAX)
        ? configured_rows
        : cfg::NODE_COUNT_MAX;

    for (uint8_t nodeId = 1; nodeId <= maxTrackedNodes; ++nodeId) {
        if (ecu.sectionEnabled(nodeId) && nodes_[nodeId].online) {
            totalRpm += nodes_[nodeId].actual_rpm;
        }
    }

    return totalRpm;
}

uint16_t NodeManager::averageActualPos(const EcuState& ecu) const {
    float sumSin = 0.0f;
    float sumCos = 0.0f;
    uint8_t count = 0;
    const uint8_t configured_rows = runtime_cfg::configuredRowCount();
    const uint8_t maxTrackedNodes = (configured_rows < cfg::NODE_COUNT_MAX)
        ? configured_rows
        : cfg::NODE_COUNT_MAX;

    for (uint8_t nodeId = 1; nodeId <= maxTrackedNodes; ++nodeId) {
        if (ecu.sectionEnabled(nodeId) && nodes_[nodeId].online) {
            const float angle = (static_cast<float>(nodes_[nodeId].actual_pos) / 65535.0f) * 2.0f * PI;
            sumSin += sinf(angle);
            sumCos += cosf(angle);
            ++count;
        }
    }

    if (count == 0) {
        return 0;
    }

    float avgAngle = atan2f(sumSin, sumCos);
    if (avgAngle < 0.0f) {
        avgAngle += 2.0f * PI;
    }

    return static_cast<uint16_t>((avgAngle / (2.0f * PI)) * 65535.0f);
}
