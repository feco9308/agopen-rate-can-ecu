#pragma once
#include "config.h"

class EcuState {
public:
    void begin();
    void update();

    void setMode(SystemMode mode);
    SystemMode mode() const;

    void setBaseRpm(float rpm);
    float baseRpm() const;

    void setDrive(bool en);
    bool drive() const;

    void setSync(bool en);
    bool sync() const;
    void setDiag(bool en);
    bool diag() const;

    uint8_t flags() const;

    void setRateSourceUpm(float v);
    float rateSourceUpm() const;

    void setMeterCal(float v);
    float meterCal() const;

    void setManualAdjust(int16_t v);
    int16_t manualAdjust() const;

    void setRelayState(uint8_t lo, uint8_t hi);
    uint8_t relayLo() const;
    uint8_t relayHi() const;

    void setSectionMask(uint16_t mask);
    uint16_t sectionMask() const;
    bool sectionEnabled(uint8_t nodeId) const;
    uint8_t activeSectionCount() const;

    void setHolesPerRev(uint16_t holes);
    uint16_t holesPerRev() const;
    void setGearRatio(float ratio);
    float gearRatio() const;
    void setUpmScale(float scale);
    float upmScale() const;

    void setPid(float kp, float ki, float kd, uint8_t min_pwm, uint8_t max_pwm);
    float kp() const;
    float ki() const;
    float kd() const;
    uint8_t minPwm() const;
    uint8_t maxPwm() const;

    void setAgioSubnet(uint8_t ip0, uint8_t ip1, uint8_t ip2);
    uint8_t agioIp0() const;
    uint8_t agioIp1() const;
    uint8_t agioIp2() const;

private:
    SystemMode mode_ = SystemMode::OFF;
    float base_rpm_ = 0.0f;
    bool drive_ = false;
    bool sync_ = false;
    bool diag_ = false;

    float rate_source_upm_ = 0.0f;
    float meter_cal_ = 0.0f;
    int16_t manual_adjust_ = 0;

    uint8_t relay_lo_ = 0;
    uint8_t relay_hi_ = 0;
    uint16_t section_mask_ = static_cast<uint16_t>(cfg::sectionMaskLimit());
    uint16_t holes_per_rev_ = cfg::DEFAULT_HOLES_PER_REV;
    float gear_ratio_ = cfg::DEFAULT_GEAR_RATIO;
    float upm_scale_ = cfg::DEFAULT_UPM_SCALE;

    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
    uint8_t min_pwm_ = 0;
    uint8_t max_pwm_ = 255;

    uint8_t agio_ip0_ = 192;
    uint8_t agio_ip1_ = 168;
    uint8_t agio_ip2_ = 1;
};
