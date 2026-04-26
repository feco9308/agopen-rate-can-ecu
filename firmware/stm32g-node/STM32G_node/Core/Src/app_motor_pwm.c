#include "app_motor_pwm.h"

#include "main.h"
#include "stm32g4xx_hal_tim.h"

enum
{
  APP_MOTOR_PWM_TIMER_CLOCK_HZ = 16000000u,
  APP_MOTOR_PWM_TARGET_HZ = 20000u,
  APP_MOTOR_PWM_PERIOD_TICKS = (APP_MOTOR_PWM_TIMER_CLOCK_HZ / (2u * APP_MOTOR_PWM_TARGET_HZ)) - 1u,
  APP_MOTOR_PWM_DEADTIME_TICKS = 16u
};

static HAL_StatusTypeDef app_motor_pwm_gpio_init(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  gpio.Pin = MOTOR_PWM_AH_Pin | MOTOR_PWM_BH_Pin | MOTOR_PWM_CH_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = MOTOR_PWM_AL_Pin | MOTOR_PWM_BL_Pin | MOTOR_PWM_CL_Pin;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Alternate = GPIO_AF6_TIM1;
  HAL_GPIO_Init(GPIOB, &gpio);

  return HAL_OK;
}

static HAL_StatusTypeDef app_motor_pwm_channel_init(AppMotorPwm *pwm, uint32_t channel)
{
  TIM_OC_InitTypeDef oc = {0};

  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = (uint32_t)(pwm->period_ticks / 2u);
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;

  return HAL_TIM_PWM_ConfigChannel(&pwm->htim, &oc, channel);
}

HAL_StatusTypeDef AppMotorPwm_Init(AppMotorPwm *pwm)
{
  TIM_ClockConfigTypeDef clk = {0};
  TIM_MasterConfigTypeDef master = {0};
  TIM_BreakDeadTimeConfigTypeDef bd = {0};

  if (pwm == 0)
  {
    return HAL_ERROR;
  }

  __HAL_RCC_TIM1_CLK_ENABLE();

  pwm->htim.Instance = TIM1;
  pwm->htim.Init.Prescaler = 0u;
  pwm->htim.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
  pwm->htim.Init.Period = APP_MOTOR_PWM_PERIOD_TICKS;
  pwm->htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  pwm->htim.Init.RepetitionCounter = 0u;
  pwm->htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

  if (HAL_TIM_Base_Init(&pwm->htim) != HAL_OK)
  {
    return HAL_ERROR;
  }

  clk.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&pwm->htim, &clk) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Init(&pwm->htim) != HAL_OK)
  {
    return HAL_ERROR;
  }

  master.MasterOutputTrigger = TIM_TRGO_RESET;
  master.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  master.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&pwm->htim, &master) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (app_motor_pwm_channel_init(pwm, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (app_motor_pwm_channel_init(pwm, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (app_motor_pwm_channel_init(pwm, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  bd.OffStateRunMode = TIM_OSSR_DISABLE;
  bd.OffStateIDLEMode = TIM_OSSI_DISABLE;
  bd.LockLevel = TIM_LOCKLEVEL_OFF;
  bd.DeadTime = APP_MOTOR_PWM_DEADTIME_TICKS;
  bd.BreakState = TIM_BREAK_DISABLE;
  bd.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  bd.BreakFilter = 0u;
  bd.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  bd.Break2State = TIM_BREAK2_DISABLE;
  bd.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  bd.Break2Filter = 0u;
  bd.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  bd.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&pwm->htim, &bd) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (app_motor_pwm_gpio_init() != HAL_OK)
  {
    return HAL_ERROR;
  }

  pwm->period_ticks = APP_MOTOR_PWM_PERIOD_TICKS;
  pwm->initialized = 1u;
  pwm->outputs_enabled = 0u;

  return AppMotorPwm_SetDutyTicks(pwm,
                                  (uint16_t)(pwm->period_ticks / 2u),
                                  (uint16_t)(pwm->period_ticks / 2u),
                                  (uint16_t)(pwm->period_ticks / 2u));
}

HAL_StatusTypeDef AppMotorPwm_SetDutyTicks(AppMotorPwm *pwm, uint16_t phase_a, uint16_t phase_b, uint16_t phase_c)
{
  if ((pwm == 0) || (pwm->initialized == 0u))
  {
    return HAL_ERROR;
  }

  if (phase_a > pwm->period_ticks)
  {
    phase_a = pwm->period_ticks;
  }
  if (phase_b > pwm->period_ticks)
  {
    phase_b = pwm->period_ticks;
  }
  if (phase_c > pwm->period_ticks)
  {
    phase_c = pwm->period_ticks;
  }

  __HAL_TIM_SET_COMPARE(&pwm->htim, TIM_CHANNEL_1, phase_a);
  __HAL_TIM_SET_COMPARE(&pwm->htim, TIM_CHANNEL_2, phase_b);
  __HAL_TIM_SET_COMPARE(&pwm->htim, TIM_CHANNEL_3, phase_c);

  return HAL_OK;
}

HAL_StatusTypeDef AppMotorPwm_SetDutyPermille(AppMotorPwm *pwm, uint16_t phase_a, uint16_t phase_b, uint16_t phase_c)
{
  uint32_t a_ticks;
  uint32_t b_ticks;
  uint32_t c_ticks;

  if ((pwm == 0) || (pwm->initialized == 0u))
  {
    return HAL_ERROR;
  }

  if (phase_a > 1000u)
  {
    phase_a = 1000u;
  }
  if (phase_b > 1000u)
  {
    phase_b = 1000u;
  }
  if (phase_c > 1000u)
  {
    phase_c = 1000u;
  }

  a_ticks = ((uint32_t)phase_a * pwm->period_ticks) / 1000u;
  b_ticks = ((uint32_t)phase_b * pwm->period_ticks) / 1000u;
  c_ticks = ((uint32_t)phase_c * pwm->period_ticks) / 1000u;

  return AppMotorPwm_SetDutyTicks(pwm, (uint16_t)a_ticks, (uint16_t)b_ticks, (uint16_t)c_ticks);
}

HAL_StatusTypeDef AppMotorPwm_EnableOutputs(AppMotorPwm *pwm)
{
  if ((pwm == 0) || (pwm->initialized == 0u) || (pwm->outputs_enabled != 0u))
  {
    return HAL_ERROR;
  }

  if (HAL_TIM_PWM_Start(&pwm->htim, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIMEx_PWMN_Start(&pwm->htim, TIM_CHANNEL_1) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&pwm->htim, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIMEx_PWMN_Start(&pwm->htim, TIM_CHANNEL_2) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIM_PWM_Start(&pwm->htim, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_TIMEx_PWMN_Start(&pwm->htim, TIM_CHANNEL_3) != HAL_OK)
  {
    return HAL_ERROR;
  }

  pwm->outputs_enabled = 1u;
  return HAL_OK;
}

HAL_StatusTypeDef AppMotorPwm_DisableOutputs(AppMotorPwm *pwm)
{
  if ((pwm == 0) || (pwm->initialized == 0u))
  {
    return HAL_ERROR;
  }

  (void)HAL_TIMEx_PWMN_Stop(&pwm->htim, TIM_CHANNEL_1);
  (void)HAL_TIM_PWM_Stop(&pwm->htim, TIM_CHANNEL_1);
  (void)HAL_TIMEx_PWMN_Stop(&pwm->htim, TIM_CHANNEL_2);
  (void)HAL_TIM_PWM_Stop(&pwm->htim, TIM_CHANNEL_2);
  (void)HAL_TIMEx_PWMN_Stop(&pwm->htim, TIM_CHANNEL_3);
  (void)HAL_TIM_PWM_Stop(&pwm->htim, TIM_CHANNEL_3);

  pwm->outputs_enabled = 0u;
  return HAL_OK;
}

uint8_t AppMotorPwm_IsInitialized(const AppMotorPwm *pwm)
{
  if (pwm == 0)
  {
    return 0u;
  }

  return pwm->initialized;
}

uint8_t AppMotorPwm_IsEnabled(const AppMotorPwm *pwm)
{
  if (pwm == 0)
  {
    return 0u;
  }

  return pwm->outputs_enabled;
}
