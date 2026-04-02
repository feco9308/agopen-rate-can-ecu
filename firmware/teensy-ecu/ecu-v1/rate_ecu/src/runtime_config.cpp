#include "runtime_config.h"

#include <EEPROM.h>

#include "config.h"
#include "ecu_state.h"

namespace {

constexpr uint16_t CONFIG_VERSION = 0x0101;
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

    if (config.gear_ratio_x100 == 0) {
        config.gear_ratio_x100 = static_cast<uint16_t>(cfg::DEFAULT_GEAR_RATIO * 100.0f);
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
}

void setDefaults(runtime_cfg::PersistentConfig& config) {
    config.version = CONFIG_VERSION;
    config.active_sensor_count = cfg::DEFAULT_ACTIVE_SENSOR_CHANNELS;
    config.configured_row_count = cfg::DEFAULT_CONFIGURED_ROW_COUNT;
    config.holes_per_rev = cfg::DEFAULT_HOLES_PER_REV;
    config.upm_scale_x10 = static_cast<uint16_t>(cfg::DEFAULT_UPM_SCALE * 10.0f);
    config.gear_ratio_x100 = static_cast<uint16_t>(cfg::DEFAULT_GEAR_RATIO * 100.0f);
    config.trim_limit_rpm_x10 = static_cast<int16_t>(cfg::TRIM_RPM_LIMIT * 10.0f);
    config.position_kp_x1000 = static_cast<uint16_t>(cfg::POSITION_KP * 1000.0f);
    config.diag_enable = 0;
    config.diag_stream_enable = 0;
    config.diag_period_div10_ms = 20;
    config.diag_detail_level = 0;
    config.ip_last_octet = 200;
    config.module_id = 0;
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

float gearRatio() { return g_config.gear_ratio_x100 / 100.0f; }
void setGearRatio(float ratio) {
    g_config.gear_ratio_x100 = (ratio <= 0.0f) ? 0 : static_cast<uint16_t>(ratio * 100.0f);
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

void applyToEcu(EcuState& ecu) {
    ecu.setHolesPerRev(holesPerRev());
    ecu.setGearRatio(gearRatio());
    ecu.setUpmScale(upmScale());
}

void applyToEcus(EcuState* ecus, uint8_t max_count) {
    if (ecus == nullptr) return;
    for (uint8_t i = 0; i < max_count; ++i) {
        applyToEcu(ecus[i]);
    }
}

} // namespace runtime_cfg
