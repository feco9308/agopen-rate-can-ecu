#pragma once

#include <Arduino.h>

class EcuState;

namespace runtime_cfg {

struct PersistentConfig {
    uint16_t version;
    uint8_t active_sensor_count;
    uint8_t configured_row_count;
    uint16_t holes_per_rev;
    uint16_t upm_scale_x10;
    uint16_t gear_ratio_x100;
    int16_t trim_limit_rpm_x10;
    uint16_t position_kp_x1000;
    uint8_t diag_enable;
    uint8_t diag_stream_enable;
    uint8_t diag_period_div10_ms;
    uint8_t diag_detail_level;
    uint8_t ip_last_octet;
    uint8_t module_id;
    uint16_t checksum;
};

void begin();
bool load();
bool save();

uint8_t activeSensorCount();
void setActiveSensorCount(uint8_t count);

uint8_t configuredRowCount();
void setConfiguredRowCount(uint8_t count);

uint16_t holesPerRev();
void setHolesPerRev(uint16_t holes);

float upmScale();
void setUpmScale(float scale);

float gearRatio();
void setGearRatio(float ratio);

float trimRpmLimit();
void setTrimRpmLimit(float rpm);

float positionKp();
void setPositionKp(float kp);

bool diagEnabled();
void setDiagEnabled(bool enabled);

bool diagStreamEnabled();
void setDiagStreamEnabled(bool enabled);

uint16_t diagPeriodMs();
void setDiagPeriodMs(uint16_t period_ms);

uint8_t diagDetailLevel();
void setDiagDetailLevel(uint8_t level);

uint8_t ipLastOctet();
void setIpLastOctet(uint8_t octet);

uint8_t moduleId();
void setModuleId(uint8_t module_id);

void applyToEcu(EcuState& ecu);
void applyToEcus(EcuState* ecus, uint8_t max_count);

} // namespace runtime_cfg
