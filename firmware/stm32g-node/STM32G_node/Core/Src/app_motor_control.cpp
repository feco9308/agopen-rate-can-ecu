#include "app_motor_control.h"

#include "simplefoc_math.h"

namespace
{
constexpr float kPowerSupplyVoltage = 11.0f;
constexpr float kDefaultVoltageLimit = 2.2f;
constexpr float kStartupVoltage = 1.8f;
constexpr float kStartupAssistVoltage = 2.0f;
constexpr float kStartupAssistVoltageMin = 0.9f;
constexpr float kVelocityKp = 0.045f;
constexpr float kVelocityKi = 0.35f;
constexpr float kVelocityRamp = 4.0f;
constexpr float kSectorAngle = SIMPLEFOC_2PI / 6.0f;
constexpr uint32_t kStartupTransitionTimeoutMs = 80u;
}

static float app_motor_control_absf(float value)
{
  return (value < 0.0f) ? -value : value;
}

static uint16_t app_motor_control_clamp_permille(int32_t value)
{
  if (value < 0)
  {
    return 0u;
  }

  if (value > 1000)
  {
    return 1000u;
  }

  return (uint16_t)value;
}

static HAL_StatusTypeDef app_motor_control_apply_voltage(AppMotorControl *control, float uq, float electrical_angle)
{
  float sine_value;
  float cosine_value;
  float ua;
  float ub;
  float uc;
  float center;
  uint16_t duty_a;
  uint16_t duty_b;
  uint16_t duty_c;

  if ((control == 0) || (control->pwm == 0))
  {
    return HAL_ERROR;
  }

  uq = SimpleFoc_Constrain(uq, -control->voltage_limit, control->voltage_limit);
  electrical_angle = SimpleFoc_NormalizeAngle(electrical_angle);
  SimpleFoc_SinCos(electrical_angle, &sine_value, &cosine_value);

  ua = -sine_value * uq;
  ub = (-0.5f * ua) + (SIMPLEFOC_SQRT3_2 * cosine_value * uq);
  uc = (-0.5f * ua) - (SIMPLEFOC_SQRT3_2 * cosine_value * uq);
  center = kPowerSupplyVoltage * 0.5f;

  ua += center;
  ub += center;
  uc += center;

  duty_a = app_motor_control_clamp_permille((int32_t)((ua / kPowerSupplyVoltage) * 1000.0f));
  duty_b = app_motor_control_clamp_permille((int32_t)((ub / kPowerSupplyVoltage) * 1000.0f));
  duty_c = app_motor_control_clamp_permille((int32_t)((uc / kPowerSupplyVoltage) * 1000.0f));

  control->duty_permille = duty_a;
  control->applied_uq = uq;
  return AppMotorPwm_SetDutyPermille(control->pwm, duty_a, duty_b, duty_c);
}

static HAL_StatusTypeDef app_motor_control_apply_idle(AppMotorControl *control)
{
  if ((control == 0) || (control->pwm == 0))
  {
    return HAL_ERROR;
  }

  control->applied_uq = 0.0f;
  control->duty_permille = 500u;

  (void)AppMotorPwm_DisableOutputs(control->pwm);
  return AppMotorPwm_SetDutyPermille(control->pwm, 500u, 500u, 500u);
}

static float app_motor_control_estimate_angle(const AppMotorControl *control, const AppHall *hall, uint32_t now_ms)
{
  float sector_center;
  float transition_dt_s;
  float sector_period_s;
  float advance = 0.0f;
  int8_t direction_sign = 1;
  uint32_t last_transition_ms;

  sector_center = ((float)AppHall_GetSequenceIndex(hall) * kSectorAngle) + SIMPLEFOC_PI_6;
  last_transition_ms = AppHall_GetLastTransitionMs(hall);

  if ((last_transition_ms != 0u) && (control->measured_velocity_rad_s > 0.1f))
  {
    transition_dt_s = (float)(now_ms - last_transition_ms) * 0.001f;
    sector_period_s = kSectorAngle / control->measured_velocity_rad_s;

    if (AppHall_GetDirection(hall) == 2u)
    {
      direction_sign = -1;
    }

    if (sector_period_s > 0.0001f)
    {
      advance = SimpleFoc_Constrain(transition_dt_s / sector_period_s, 0.0f, 1.0f) * kSectorAngle;
      sector_center += ((float)direction_sign * advance);
    }
  }

  return SimpleFoc_NormalizeAngle(sector_center);
}

static float app_motor_control_step_velocity_pi(AppMotorControl *control, float target_velocity, float measured_velocity, float dt_s)
{
  float error;
  float proportional;
  float integral;
  float output;
  float output_rate;

  error = target_velocity - measured_velocity;
  proportional = control->velocity_kp * error;
  integral = control->velocity_integral + (control->velocity_ki * dt_s * 0.5f * (error + control->velocity_error_prev));
  integral = SimpleFoc_Constrain(integral, -control->voltage_limit, control->voltage_limit);

  output = proportional + integral;
  output = SimpleFoc_Constrain(output, -control->voltage_limit, control->voltage_limit);

  if (dt_s > 0.0001f)
  {
    output_rate = (output - control->velocity_output_prev) / dt_s;
    if (output_rate > kVelocityRamp)
    {
      output = control->velocity_output_prev + (kVelocityRamp * dt_s);
    }
    else if (output_rate < -kVelocityRamp)
    {
      output = control->velocity_output_prev - (kVelocityRamp * dt_s);
    }
  }

  control->velocity_integral = integral;
  control->velocity_error_prev = error;
  control->velocity_output_prev = output;
  return output;
}

static HAL_StatusTypeDef app_motor_control_apply_startup_assist(AppMotorControl *control, float dt_s)
{
  float electrical_velocity;
  float startup_voltage;

  if (control == 0)
  {
    return HAL_ERROR;
  }

  electrical_velocity = control->target_velocity_rad_s * (float)control->pole_pairs;
  if (app_motor_control_absf(electrical_velocity) < 3.0f)
  {
    electrical_velocity = (electrical_velocity < 0.0f) ? -3.0f : 3.0f;
  }

  startup_voltage = (control->target_velocity_rad_s / (40.0f * SIMPLEFOC_RPM_TO_RADS)) * kStartupAssistVoltage;
  startup_voltage = SimpleFoc_Constrain(startup_voltage, kStartupAssistVoltageMin, kStartupAssistVoltage);

  control->open_loop_electrical_angle = SimpleFoc_NormalizeAngle(control->open_loop_electrical_angle + (electrical_velocity * dt_s));
  control->startup_active = 1u;
  return app_motor_control_apply_voltage(control, (electrical_velocity < 0.0f) ? -startup_voltage : startup_voltage, control->open_loop_electrical_angle);
}

extern "C" HAL_StatusTypeDef AppMotorControl_Init(AppMotorControl *control, AppMotorPwm *pwm, uint16_t pole_pairs)
{
  if ((control == 0) || (pwm == 0))
  {
    return HAL_ERROR;
  }

  control->pwm = pwm;
  control->pole_pairs = (pole_pairs == 0u) ? 7u : pole_pairs;
  control->target_rpm = 0u;
  control->applied_rpm = 0u;
  control->electrical_pos_q16 = 0u;
  control->duty_permille = 500u;
  control->target_velocity_rad_s = 0.0f;
  control->measured_velocity_rad_s = 0.0f;
  control->applied_uq = 0.0f;
  control->voltage_limit = kDefaultVoltageLimit;
  control->velocity_kp = kVelocityKp;
  control->velocity_ki = kVelocityKi;
  control->velocity_integral = 0.0f;
  control->velocity_error_prev = 0.0f;
  control->velocity_output_prev = 0.0f;
  control->open_loop_electrical_angle = 0.0f;
  control->last_update_ms = 0u;
  control->run_enabled = 0u;
  control->startup_active = 0u;

  return app_motor_control_apply_idle(control);
}

extern "C" HAL_StatusTypeDef AppMotorControl_SetCommand(AppMotorControl *control, uint8_t run_enabled, uint16_t target_rpm)
{
  if (control == 0)
  {
    return HAL_ERROR;
  }

  control->run_enabled = (run_enabled != 0u) ? 1u : 0u;
  control->target_rpm = target_rpm;
  control->target_velocity_rad_s = (float)target_rpm * SIMPLEFOC_RPM_TO_RADS;
  return HAL_OK;
}

extern "C" HAL_StatusTypeDef AppMotorControl_Poll(AppMotorControl *control, const AppHall *hall, uint32_t now_ms)
{
  float dt_s;
  float uq;
  float electrical_angle;
  float startup_velocity_threshold;
  uint32_t last_transition_ms;
  uint8_t needs_startup_assist;

  if ((control == 0) || (control->pwm == 0) || (hall == 0))
  {
    return HAL_ERROR;
  }

  if (control->last_update_ms == 0u)
  {
    control->last_update_ms = now_ms;
    control->applied_rpm = AppHall_GetRpm(hall);
    control->electrical_pos_q16 = AppHall_GetPosQ16(hall);
    return app_motor_control_apply_idle(control);
  }

  dt_s = (float)(now_ms - control->last_update_ms) * 0.001f;
  if (dt_s <= 0.0f)
  {
    return HAL_OK;
  }

  control->last_update_ms = now_ms;
  control->applied_rpm = AppHall_GetRpm(hall);
  control->electrical_pos_q16 = AppHall_GetPosQ16(hall);
  control->measured_velocity_rad_s = (float)control->applied_rpm * SIMPLEFOC_RPM_TO_RADS;

  if ((control->run_enabled == 0u) || (control->target_rpm == 0u) || (AppHall_IsReady(hall) == 0u))
  {
    control->velocity_integral = 0.0f;
    control->velocity_error_prev = 0.0f;
    control->velocity_output_prev = 0.0f;
    control->open_loop_electrical_angle = 0.0f;
    control->startup_active = 0u;
    return app_motor_control_apply_idle(control);
  }

  if (AppMotorPwm_IsEnabled(control->pwm) == 0u)
  {
    if (AppMotorPwm_EnableOutputs(control->pwm) != HAL_OK)
    {
      return HAL_ERROR;
    }
  }

  last_transition_ms = AppHall_GetLastTransitionMs(hall);
  needs_startup_assist = 0u;
  startup_velocity_threshold = SimpleFoc_Constrain(control->target_velocity_rad_s * 0.25f,
                                                   0.4f * SIMPLEFOC_RPM_TO_RADS,
                                                   4.0f * SIMPLEFOC_RPM_TO_RADS);
  if ((control->measured_velocity_rad_s < startup_velocity_threshold) &&
      ((last_transition_ms == 0u) || ((now_ms - last_transition_ms) > kStartupTransitionTimeoutMs)))
  {
    needs_startup_assist = 1u;
  }

  if (needs_startup_assist != 0u)
  {
    if (control->startup_active == 0u)
    {
      control->open_loop_electrical_angle = app_motor_control_estimate_angle(control, hall, now_ms);
    }
    return app_motor_control_apply_startup_assist(control, dt_s);
  }

  control->startup_active = 0u;

  uq = app_motor_control_step_velocity_pi(control,
                                          control->target_velocity_rad_s,
                                          control->measured_velocity_rad_s,
                                          dt_s);

  if ((control->applied_rpm < 40u) && (app_motor_control_absf(uq) < kStartupVoltage))
  {
    uq = (control->target_velocity_rad_s >= 0.0f) ? kStartupVoltage : -kStartupVoltage;
  }

  electrical_angle = app_motor_control_estimate_angle(control, hall, now_ms);
  return app_motor_control_apply_voltage(control, uq, electrical_angle);
}

extern "C" uint16_t AppMotorControl_GetAppliedRpm(const AppMotorControl *control)
{
  if (control == 0)
  {
    return 0u;
  }

  return control->applied_rpm;
}

extern "C" uint16_t AppMotorControl_GetElectricalPosQ16(const AppMotorControl *control)
{
  if (control == 0)
  {
    return 0u;
  }

  return control->electrical_pos_q16;
}
