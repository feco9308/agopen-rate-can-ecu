#include "ecu_state.h"
#include "rate_math.h"

void EcuState::begin() {
    mode_ = SystemMode::OFF;
    base_rpm_ = 0.0f;
    drive_ = false;
    sync_ = false;
    rate_source_upm_ = 0.0f;
    meter_cal_ = 0.0f;
    manual_adjust_ = 0;
    relay_lo_ = 0;
    relay_hi_ = 0;
    section_mask_ = static_cast<uint16_t>(cfg::sectionMaskLimit());
    holes_per_rev_ = cfg::DEFAULT_HOLES_PER_REV;
    kp_ = 0.0f;
    ki_ = 0.0f;
    kd_ = 0.0f;
    min_pwm_ = 0;
    max_pwm_ = 255;
}

void EcuState::update() {}

void EcuState::setMode(SystemMode mode) { mode_ = mode; }
SystemMode EcuState::mode() const { return mode_; }

void EcuState::setBaseRpm(float rpm) {
    if (rpm < cfg::RPM_MIN) rpm = cfg::RPM_MIN;
    if (rpm > cfg::RPM_MAX) rpm = cfg::RPM_MAX;
    base_rpm_ = rpm;
}
float EcuState::baseRpm() const { return base_rpm_; }

void EcuState::setDrive(bool en) { drive_ = en; }
bool EcuState::drive() const { return drive_; }

void EcuState::setSync(bool en) { sync_ = en; }
bool EcuState::sync() const { return sync_; }

uint8_t EcuState::flags() const {
    uint8_t f = 0;
    if (drive_) f |= CTRL_DRIVE_ENABLE;
    if (sync_)  f |= CTRL_SYNC_ENABLE;
    return f;
}

void EcuState::setRateSourceUpm(float v) { rate_source_upm_ = v; }
float EcuState::rateSourceUpm() const { return rate_source_upm_; }

void EcuState::setMeterCal(float v) { meter_cal_ = v; }
float EcuState::meterCal() const { return meter_cal_; }

void EcuState::setManualAdjust(int16_t v) { manual_adjust_ = v; }
int16_t EcuState::manualAdjust() const { return manual_adjust_; }

void EcuState::setRelayState(uint8_t lo, uint8_t hi) {
    relay_lo_ = lo;
    relay_hi_ = hi;
}
uint8_t EcuState::relayLo() const { return relay_lo_; }
uint8_t EcuState::relayHi() const { return relay_hi_; }

void EcuState::setSectionMask(uint16_t mask) {
    section_mask_ = static_cast<uint16_t>(mask & cfg::sectionMaskLimit());
}
uint16_t EcuState::sectionMask() const { return section_mask_; }

bool EcuState::sectionEnabled(uint8_t nodeId) const {
    if (nodeId == 0 ||
        nodeId > cfg::NODE_COUNT_MAX ||
        nodeId > cfg::ACTIVE_SECTION_COUNT ||
        nodeId > cfg::PGN_SECTION_BIT_COUNT) {
        return false;
    }

    const uint16_t bit = static_cast<uint16_t>(1u << (nodeId - 1u));
    return (section_mask_ & bit) != 0;
}

uint8_t EcuState::activeSectionCount() const {
    const uint8_t tracked_sections = (cfg::ACTIVE_SECTION_COUNT < cfg::PGN_SECTION_BIT_COUNT)
        ? cfg::ACTIVE_SECTION_COUNT
        : cfg::PGN_SECTION_BIT_COUNT;

    return rate_math::countActiveSections(section_mask_, tracked_sections);
}

void EcuState::setHolesPerRev(uint16_t holes) {
    holes_per_rev_ = (holes == 0) ? cfg::DEFAULT_HOLES_PER_REV : holes;
}
uint16_t EcuState::holesPerRev() const { return holes_per_rev_; }

void EcuState::setPid(float kp, float ki, float kd, uint8_t min_pwm, uint8_t max_pwm) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    min_pwm_ = min_pwm;
    max_pwm_ = max_pwm;
}

float EcuState::kp() const { return kp_; }
float EcuState::ki() const { return ki_; }
float EcuState::kd() const { return kd_; }
uint8_t EcuState::minPwm() const { return min_pwm_; }
uint8_t EcuState::maxPwm() const { return max_pwm_; }

void EcuState::setAgioSubnet(uint8_t ip0, uint8_t ip1, uint8_t ip2) {
    agio_ip0_ = ip0;
    agio_ip1_ = ip1;
    agio_ip2_ = ip2;
}

uint8_t EcuState::agioIp0() const { return agio_ip0_; }
uint8_t EcuState::agioIp1() const { return agio_ip1_; }
uint8_t EcuState::agioIp2() const { return agio_ip2_; }
