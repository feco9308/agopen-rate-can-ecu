#pragma once

#include <Arduino.h>

#include "config.h"

class EcuState;

namespace runtime_cfg {

struct PersistentConfig {
    uint16_t version;
    uint8_t active_sensor_count;
    uint8_t configured_row_count;
    uint16_t holes_per_rev;
    uint16_t upm_scale_x10;
    uint16_t drive_ratio_x100[cfg::MAX_SENSOR_CHANNELS];
    uint16_t motor_ratio_x100[cfg::MAX_SENSOR_CHANNELS];
    int16_t trim_limit_rpm_x10;
    uint16_t position_kp_x1000;
    uint8_t diag_enable;
    uint8_t diag_stream_enable;
    uint8_t diag_period_div10_ms;
    uint8_t diag_detail_level;
    uint8_t ip_last_octet;
    uint8_t module_id;
    uint8_t monitor_output_enable;
    uint8_t monitor_output_mode;
    uint8_t monitor_rows;
    uint16_t planter_row_width_cm_x10;
    uint32_t planter_target_population;
    uint16_t planter_doubles_factor_x100;
    uint8_t planter_metric;
    uint8_t blockage_rows_per_module;
    uint8_t blockage_threshold;
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

float driveRatio(uint8_t sensor_index);
void setDriveRatio(uint8_t sensor_index, float ratio);

float motorRatio(uint8_t sensor_index);
void setMotorRatio(uint8_t sensor_index, float ratio);

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

bool monitorOutputEnabled();
void setMonitorOutputEnabled(bool enabled);

uint8_t monitorOutputMode();
void setMonitorOutputMode(uint8_t mode);

uint8_t monitorRows();
void setMonitorRows(uint8_t rows);

float planterRowWidthCm();
void setPlanterRowWidthCm(float row_width_cm);

uint32_t planterTargetPopulation();
void setPlanterTargetPopulation(uint32_t target_population);

float planterDoublesFactor();
void setPlanterDoublesFactor(float factor);

bool planterMetric();
void setPlanterMetric(bool metric);

uint8_t blockageRowsPerModule();
void setBlockageRowsPerModule(uint8_t rows_per_module);

uint8_t blockageThreshold();
void setBlockageThreshold(uint8_t threshold);

void applyToEcu(EcuState& ecu, uint8_t sensor_index);
void applyToEcus(EcuState* ecus, uint8_t max_count);

} // namespace runtime_cfg
