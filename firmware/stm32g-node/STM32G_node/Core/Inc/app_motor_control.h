#ifndef APP_MOTOR_CONTROL_H
#define APP_MOTOR_CONTROL_H

#include "app_hall.h"
#include "app_motor_pwm.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  AppMotorPwm *pwm;
  uint16_t pole_pairs;
  uint16_t target_rpm;
  uint16_t applied_rpm;
  uint16_t electrical_pos_q16;
  uint16_t duty_permille;
  float target_velocity_rad_s;
  float measured_velocity_rad_s;
  float applied_uq;
  float voltage_limit;
  float velocity_kp;
  float velocity_ki;
  float velocity_integral;
  float velocity_error_prev;
  float velocity_output_prev;
  float open_loop_electrical_angle;
  uint32_t last_update_ms;
  uint8_t run_enabled;
  uint8_t startup_active;
} AppMotorControl;

HAL_StatusTypeDef AppMotorControl_Init(AppMotorControl *control, AppMotorPwm *pwm, uint16_t pole_pairs);
HAL_StatusTypeDef AppMotorControl_SetCommand(AppMotorControl *control, uint8_t run_enabled, uint16_t target_rpm);
HAL_StatusTypeDef AppMotorControl_Poll(AppMotorControl *control, const AppHall *hall, uint32_t now_ms);
uint16_t AppMotorControl_GetAppliedRpm(const AppMotorControl *control);
uint16_t AppMotorControl_GetElectricalPosQ16(const AppMotorControl *control);

#ifdef __cplusplus
}
#endif

#endif
