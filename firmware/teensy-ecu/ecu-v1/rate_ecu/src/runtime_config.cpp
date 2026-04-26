#include "runtime_config.h"

#include <EEPROM.h>

#include "config.h"
#include "ecu_state.h"

namespace {

constexpr uint16_t CONFIG_VERSION = 0x0105;
constexpr int EEPROM_ADDR = 0;

runtime_cfg::PersistentConfig g_config{};

uint16_t checksumConfig(const runtime_cfg::PersistentConfig& cfg) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&cfg);
    uint16_t sum = 0;
    for (size_t i = 0; i < sizeof(runtime_cfg::PersistentConfig) - sizeof(cfg.checksum); ++i) {
        sum = static_cast<uint16_t>(sum + bytes[i]);
    }
    return sum;
}

void clampConfig(runtime_cfg::PersistentConfig& config) {
    if (config.active_sensor_count == 0 || config.active_sensor_count > cfg::MAX_SENSOR_CHANNELS) {
        config.active_sensor_count = cfg::DEFAULT_ACTIVE_SENSOR_CHANNELS;
    }

    if (config.configured_row_count == 0 || config.configured_row_count > cfg::NODE_COUNT_MAX) {
        config.configured_row_count = cfg::DEFAULT_CONFIGURED_ROW_COUNT;
    }
    if (config.configured_row_count > cfg::PGN_SECTION_BIT_COUNT) {
        config.configured_row_count = cfg::PGN_SECTION_BIT_COUNT;
    }

    if (config.holes_per_rev == 0) {
        config.holes_per_rev = cfg::DEFAULT_HOLES_PER_REV;
    }

    if (config.upm_scale_x10 == 0) {
        config.upm_scale_x10 = static_cast<uint16_t>(cfg::DEFAULT_UPM_SCALE * 10.0f);
    }

    for (uint8_t sensorIndex = 0; sensorIndex < cfg::MAX_SENSOR_CHANNELS; ++sensorIndex) {
        if (config.drive_ratio_x100[sensorIndex] == 0) {
            config.drive_ratio_x100[sensorIndex] =
                static_cast<uint16_t>(cfg::DEFAULT_DRIVE_RATIO * 100.0f);
        }
        if (config.motor_ratio_x100[sensorIndex] == 0) {
            config.motor_ratio_x100[sensorIndex] =
                static_cast<uint16_t>(cfg::DEFAULT_MOTOR_RATIO * 100.0f);
        }
    }

    if (config.trim_limit_rpm_x10 <= 0) {
        config.trim_limit_rpm_x10 = static_cast<int16_t>(cfg::TRIM_RPM_LIMIT * 10.0f);
    }

    if (config.position_kp_x1000 == 0) {
        config.position_kp_x1000 = static_cast<uint16_t>(cfg::POSITION_KP * 1000.0f);
    }

    if (config.diag_period_div10_ms == 0) {
        config.diag_period_div10_ms = 20;
    }

    if (config.ip_last_octet == 0) {
        config.ip_last_octet = 200;
    }

    if (config.rate_app_mode > static_cast<uint8_t>(RateAppMode::SK21)) {
        config.rate_app_mode = static_cast<uint8_t>(RateAppMode::LEGACY);
    }

    if (config.monitor_output_mode > static_cast<uint8_t>(MonitorOutputMode::BLOCKAGE)) {
        config.monitor_output_mode = static_cast<uint8_t>(MonitorOutputMode::OFF);
    }

    if (config.monitor_rows == 0 || config.monitor_rows > 100) {
        config.monitor_rows = config.configured_row_count;
    }
    const uint8_t maxMonitorRows = static_cast<uint8_t>(
        min<uint16_t>(100u, static_cast<uint16_t>(config.active_sensor_count) * config.configured_row_count)
    );
    if (config.monitor_rows > maxMonitorRows) {
        config.monitor_rows = maxMonitorRows;
    }

    if (config.planter_row_width_cm_x10 == 0) {
        config.planter_row_width_cm_x10 =
            static_cast<uint16_t>(cfg::DEFAULT_PLANTER_ROW_WIDTH_CM * 10.0f);
    }

    if (config.planter_target_population == 0) {
        config.planter_target_population = cfg::DEFAULT_PLANTER_TARGET_POPULATION;
    }

    if (config.planter_doubles_factor_x100 == 0) {
        config.planter_doubles_factor_x100 =
            static_cast<uint16_t>(cfg::DEFAULT_PLANTER_DOUBLES_FACTOR * 100.0f);
    }

    if (config.blockage_rows_per_module == 0 || config.blockage_rows_per_module > cfg::NODE_COUNT_MAX) {
        config.blockage_rows_per_module = cfg::DEFAULT_BLOCKAGE_ROWS_PER_MODULE;
    }

    if (config.blockage_threshold == 0) {
        config.blockage_threshold = cfg::DEFAULT_BLOCKAGE_THRESHOLD;
    }
}

void setDefaults(runtime_cfg::PersistentConfig& config) {
    config.version = CONFIG_VERSION;
    config.active_sensor_count = cfg::DEFAULT_ACTIVE_SENSOR_CHANNELS;
    config.configured_row_count = cfg::DEFAULT_CONFIGURED_ROW_COUNT;
    config.holes_per_rev = cfg::DEFAULT_HOLES_PER_REV;
    config.upm_scale_x10 = static_cast<uint16_t>(cfg::DEFAULT_UPM_SCALE * 10.0f);
    for (uint8_t sensorIndex = 0; sensorIndex < cfg::MAX_SENSOR_CHANNELS; ++sensorIndex) {
        config.drive_ratio_x100[sensorIndex] =
            static_cast<uint16_t>(cfg::DEFAULT_DRIVE_RATIO * 100.0f);
        config.motor_ratio_x100[sensorIndex] =
            static_cast<uint16_t>(cfg::DEFAULT_MOTOR_RATIO * 100.0f);
    }
    config.trim_limit_rpm_x10 = static_cast<int16_t>(cfg::TRIM_RPM_LIMIT * 10.0f);
    config.position_kp_x1000 = static_cast<uint16_t>(cfg::POSITION_KP * 1000.0f);
    config.diag_enable = 0;
    config.diag_stream_enable = 0;
    config.diag_period_div10_ms = 20;
    config.diag_detail_level = 0;
    config.ip_last_octet = 200;
    config.module_id = 0;
    config.rate_app_mode = static_cast<uint8_t>(RateAppMode::LEGACY);
    config.monitor_output_enable = 0;
    config.monitor_output_mode = static_cast<uint8_t>(MonitorOutputMode::OFF);
    config.monitor_rows = cfg::DEFAULT_CONFIGURED_ROW_COUNT;
    config.planter_row_width_cm_x10 =
        static_cast<uint16_t>(cfg::DEFAULT_PLANTER_ROW_WIDTH_CM * 10.0f);
    config.planter_target_population = cfg::DEFAULT_PLANTER_TARGET_POPULATION;
    config.planter_doubles_factor_x100 =
        static_cast<uint16_t>(cfg::DEFAULT_PLANTER_DOUBLES_FACTOR * 100.0f);
    config.planter_metric = cfg::DEFAULT_PLANTER_METRIC ? 1 : 0;
    config.blockage_rows_per_module = cfg::DEFAULT_BLOCKAGE_ROWS_PER_MODULE;
    config.blockage_threshold = cfg::DEFAULT_BLOCKAGE_THRESHOLD;
    config.checksum = checksumConfig(config);
}

} // namespace

namespace runtime_cfg {

void begin() {
    setDefaults(g_config);
    load();
}

bool load() {
    PersistentConfig stored{};
    EEPROM.get(EEPROM_ADDR, stored);

    if (stored.version != CONFIG_VERSION) {
        return false;
    }

    const uint16_t expected = checksumConfig(stored);
    if (stored.checksum != expected) {
        return false;
    }

    g_config = stored;
    clampConfig(g_config);
    g_config.checksum = checksumConfig(g_config);
    return true;
}

bool save() {
    clampConfig(g_config);
    g_config.version = CONFIG_VERSION;
    g_config.checksum = checksumConfig(g_config);
    EEPROM.put(EEPROM_ADDR, g_config);
    return true;
}

uint8_t activeSensorCount() { return g_config.active_sensor_count; }
void setActiveSensorCount(uint8_t count) {
    g_config.active_sensor_count = count;
    clampConfig(g_config);
}

uint8_t configuredRowCount() { return g_config.configured_row_count; }
void setConfiguredRowCount(uint8_t count) {
    g_config.configured_row_count = count;
    clampConfig(g_config);
}

uint16_t holesPerRev() { return g_config.holes_per_rev; }
void setHolesPerRev(uint16_t holes) {
    g_config.holes_per_rev = holes;
    clampConfig(g_config);
}

float upmScale() { return g_config.upm_scale_x10 / 10.0f; }
void setUpmScale(float scale) {
    g_config.upm_scale_x10 = (scale <= 0.0f) ? 0 : static_cast<uint16_t>(scale * 10.0f);
    clampConfig(g_config);
}

float driveRatio(uint8_t sensor_index) {
    if (sensor_index >= cfg::MAX_SENSOR_CHANNELS) {
        sensor_index = 0;
    }
    return g_config.drive_ratio_x100[sensor_index] / 100.0f;
}
void setDriveRatio(uint8_t sensor_index, float ratio) {
    if (sensor_index >= cfg::MAX_SENSOR_CHANNELS) return;
    g_config.drive_ratio_x100[sensor_index] =
        (ratio <= 0.0f) ? 0 : static_cast<uint16_t>(ratio * 100.0f);
    clampConfig(g_config);
}

float motorRatio(uint8_t sensor_index) {
    if (sensor_index >= cfg::MAX_SENSOR_CHANNELS) {
        sensor_index = 0;
    }
    return g_config.motor_ratio_x100[sensor_index] / 100.0f;
}
void setMotorRatio(uint8_t sensor_index, float ratio) {
    if (sensor_index >= cfg::MAX_SENSOR_CHANNELS) return;
    g_config.motor_ratio_x100[sensor_index] =
        (ratio <= 0.0f) ? 0 : static_cast<uint16_t>(ratio * 100.0f);
    clampConfig(g_config);
}

float trimRpmLimit() { return g_config.trim_limit_rpm_x10 / 10.0f; }
void setTrimRpmLimit(float rpm) {
    g_config.trim_limit_rpm_x10 = (rpm <= 0.0f) ? 0 : static_cast<int16_t>(rpm * 10.0f);
    clampConfig(g_config);
}

float positionKp() { return g_config.position_kp_x1000 / 1000.0f; }
void setPositionKp(float kp) {
    g_config.position_kp_x1000 = (kp <= 0.0f) ? 0 : static_cast<uint16_t>(kp * 1000.0f);
    clampConfig(g_config);
}

bool diagEnabled() { return g_config.diag_enable != 0; }
void setDiagEnabled(bool enabled) { g_config.diag_enable = enabled ? 1 : 0; }

bool diagStreamEnabled() { return g_config.diag_stream_enable != 0; }
void setDiagStreamEnabled(bool enabled) { g_config.diag_stream_enable = enabled ? 1 : 0; }

uint16_t diagPeriodMs() { return static_cast<uint16_t>(g_config.diag_period_div10_ms) * 10u; }
void setDiagPeriodMs(uint16_t period_ms) {
    const uint16_t clamped = (period_ms < 10u) ? 10u : period_ms;
    g_config.diag_period_div10_ms = static_cast<uint8_t>(clamped / 10u);
    clampConfig(g_config);
}

uint8_t diagDetailLevel() { return g_config.diag_detail_level; }
void setDiagDetailLevel(uint8_t level) { g_config.diag_detail_level = level; }

uint8_t ipLastOctet() { return g_config.ip_last_octet; }
void setIpLastOctet(uint8_t octet) {
    g_config.ip_last_octet = octet;
    clampConfig(g_config);
}

uint8_t moduleId() { return g_config.module_id; }
void setModuleId(uint8_t module_id) { g_config.module_id = module_id; }

uint8_t rateAppMode() { return g_config.rate_app_mode; }
void setRateAppMode(uint8_t mode) {
    g_config.rate_app_mode = mode;
    clampConfig(g_config);
}

bool monitorOutputEnabled() { return g_config.monitor_output_enable != 0; }
void setMonitorOutputEnabled(bool enabled) { g_config.monitor_output_enable = enabled ? 1 : 0; }

uint8_t monitorOutputMode() { return g_config.monitor_output_mode; }
void setMonitorOutputMode(uint8_t mode) {
    g_config.monitor_output_mode = mode;
    clampConfig(g_config);
}

uint8_t monitorRows() { return g_config.monitor_rows; }
void setMonitorRows(uint8_t rows) {
    g_config.monitor_rows = rows;
    clampConfig(g_config);
}

float planterRowWidthCm() { return g_config.planter_row_width_cm_x10 / 10.0f; }
void setPlanterRowWidthCm(float row_width_cm) {
    g_config.planter_row_width_cm_x10 =
        (row_width_cm <= 0.0f) ? 0 : static_cast<uint16_t>(row_width_cm * 10.0f);
    clampConfig(g_config);
}

uint32_t planterTargetPopulation() { return g_config.planter_target_population; }
void setPlanterTargetPopulation(uint32_t target_population) {
    g_config.planter_target_population = target_population;
    clampConfig(g_config);
}

float planterDoublesFactor() { return g_config.planter_doubles_factor_x100 / 100.0f; }
void setPlanterDoublesFactor(float factor) {
    g_config.planter_doubles_factor_x100 =
        (factor <= 0.0f) ? 0 : static_cast<uint16_t>(factor * 100.0f);
    clampConfig(g_config);
}

bool planterMetric() { return g_config.planter_metric != 0; }
void setPlanterMetric(bool metric) { g_config.planter_metric = metric ? 1 : 0; }

uint8_t blockageRowsPerModule() { return g_config.blockage_rows_per_module; }
void setBlockageRowsPerModule(uint8_t rows_per_module) {
    g_config.blockage_rows_per_module = rows_per_module;
    clampConfig(g_config);
}

uint8_t blockageThreshold() { return g_config.blockage_threshold; }
void setBlockageThreshold(uint8_t threshold) {
    g_config.blockage_threshold = threshold;
    clampConfig(g_config);
}

void applyToEcu(EcuState& ecu, uint8_t sensor_index) {
    ecu.setHolesPerRev(holesPerRev());
    ecu.setDriveRatio(driveRatio(sensor_index));
    ecu.setMotorRatio(motorRatio(sensor_index));
    ecu.setUpmScale(upmScale());
    ecu.setDiag(diagEnabled());
}

void applyToEcus(EcuState* ecus, uint8_t max_count) {
    if (ecus == nullptr) return;
    for (uint8_t i = 0; i < max_count; ++i) {
        applyToEcu(ecus[i], i);
    }
}

} // namespace runtime_cfg
