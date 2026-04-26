#ifndef APP_MOTOR_PWM_H
#define APP_MOTOR_PWM_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_tim.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  TIM_HandleTypeDef htim;
  uint16_t period_ticks;
  uint8_t initialized;
  uint8_t outputs_enabled;
} AppMotorPwm;

HAL_StatusTypeDef AppMotorPwm_Init(AppMotorPwm *pwm);
HAL_StatusTypeDef AppMotorPwm_SetDutyTicks(AppMotorPwm *pwm, uint16_t phase_a, uint16_t phase_b, uint16_t phase_c);
HAL_StatusTypeDef AppMotorPwm_SetDutyPermille(AppMotorPwm *pwm, uint16_t phase_a, uint16_t phase_b, uint16_t phase_c);
HAL_StatusTypeDef AppMotorPwm_EnableOutputs(AppMotorPwm *pwm);
HAL_StatusTypeDef AppMotorPwm_DisableOutputs(AppMotorPwm *pwm);
uint8_t AppMotorPwm_IsInitialized(const AppMotorPwm *pwm);
uint8_t AppMotorPwm_IsEnabled(const AppMotorPwm *pwm);

#ifdef __cplusplus
}
#endif

#endif
