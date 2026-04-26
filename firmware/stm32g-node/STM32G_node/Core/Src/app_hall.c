#include "app_hall.h"

#include "main.h"

enum
{
  APP_HALL_POLE_PAIRS = 7u,
  APP_HALL_TIMEOUT_US = 80000u
};

static const uint8_t k_hall_sequence[6] = {1u, 5u, 4u, 6u, 2u, 3u};

static void app_hall_timing_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0u;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t app_hall_micros(void)
{
  uint32_t cpu_hz = HAL_RCC_GetHCLKFreq();

  if (cpu_hz == 0u)
  {
    return 0u;
  }

  return DWT->CYCCNT / (cpu_hz / 1000000u);
}

static uint8_t app_hall_read_raw(void)
{
  uint8_t raw = 0u;

  if (HAL_GPIO_ReadPin(HALL_A_GPIO_Port, HALL_A_Pin) != GPIO_PIN_RESET)
  {
    raw |= 0x01u;
  }
  if (HAL_GPIO_ReadPin(HALL_B_GPIO_Port, HALL_B_Pin) != GPIO_PIN_RESET)
  {
    raw |= 0x02u;
  }
  if (HAL_GPIO_ReadPin(HALL_C_GPIO_Port, HALL_C_Pin) != GPIO_PIN_RESET)
  {
    raw |= 0x04u;
  }

  return raw;
}

static uint8_t app_hall_find_index(uint8_t raw_state)
{
  uint8_t i;

  for (i = 0u; i < 6u; ++i)
  {
    if (k_hall_sequence[i] == raw_state)
    {
      return i;
    }
  }

  return 0xFFu;
}

static uint16_t app_hall_index_to_pos_q16(uint8_t index)
{
  return (uint16_t)(((uint32_t)index * 65536u) / 6u);
}

HAL_StatusTypeDef AppHall_Init(AppHall *hall)
{
  GPIO_InitTypeDef gpio = {0};
  uint8_t index;

  if (hall == 0)
  {
    return HAL_ERROR;
  }

  __HAL_RCC_GPIOB_CLK_ENABLE();
  app_hall_timing_init();

  gpio.Pin = HALL_A_Pin | HALL_B_Pin | HALL_C_Pin;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  hall->raw_state = app_hall_read_raw();
  index = app_hall_find_index(hall->raw_state);

  hall->initialized = 1u;
  hall->valid_state = (index != 0xFFu) ? 1u : 0u;
  hall->sequence_index = (index != 0xFFu) ? index : 0u;
  hall->direction = 0u;
  hall->rpm = 0u;
  hall->pos_q16 = (index != 0xFFu) ? app_hall_index_to_pos_q16(index) : 0u;
  hall->transition_count = 0u;
  hall->last_transition_us = 0u;
  hall->last_transition_ms = 0u;

  return HAL_OK;
}

HAL_StatusTypeDef AppHall_Poll(AppHall *hall, uint32_t now_ms)
{
  uint8_t raw_state;
  uint8_t next_index;
  uint32_t now_us;
  uint32_t dt_us;
  uint8_t delta;

  if ((hall == 0) || (hall->initialized == 0u))
  {
    return HAL_ERROR;
  }

  raw_state = app_hall_read_raw();
  now_us = app_hall_micros();
  hall->raw_state = raw_state;
  next_index = app_hall_find_index(raw_state);

  if (next_index == 0xFFu)
  {
    hall->valid_state = 0u;
    if ((hall->last_transition_us != 0u) && ((now_us - hall->last_transition_us) > APP_HALL_TIMEOUT_US))
    {
      hall->rpm = 0u;
    }
    return HAL_OK;
  }

  hall->valid_state = 1u;
  hall->pos_q16 = app_hall_index_to_pos_q16(next_index);

  if (next_index == hall->sequence_index)
  {
    if ((hall->last_transition_us != 0u) && ((now_us - hall->last_transition_us) > APP_HALL_TIMEOUT_US))
    {
      hall->rpm = 0u;
    }
    return HAL_OK;
  }

  delta = (uint8_t)((next_index + 6u - hall->sequence_index) % 6u);
  if ((delta != 1u) && (delta != 5u))
  {
    return HAL_OK;
  }

  if (hall->last_transition_us != 0u)
  {
    dt_us = now_us - hall->last_transition_us;
    if (dt_us != 0u)
    {
      hall->rpm = (uint16_t)(60000000u / (6u * APP_HALL_POLE_PAIRS * dt_us));
    }
  }

  if (delta == 1u)
  {
    hall->direction = 1u;
  }
  else if (delta == 5u)
  {
    hall->direction = 2u;
  }
  else
  {
    hall->direction = 0u;
  }

  hall->sequence_index = next_index;
  hall->last_transition_us = now_us;
  hall->last_transition_ms = now_ms;
  hall->transition_count++;

  return HAL_OK;
}

uint8_t AppHall_IsReady(const AppHall *hall)
{
  if ((hall == 0) || (hall->initialized == 0u))
  {
    return 0u;
  }

  return hall->valid_state;
}

uint16_t AppHall_GetRpm(const AppHall *hall)
{
  if (hall == 0)
  {
    return 0u;
  }

  return hall->rpm;
}

uint16_t AppHall_GetPosQ16(const AppHall *hall)
{
  if (hall == 0)
  {
    return 0u;
  }

  return hall->pos_q16;
}

uint8_t AppHall_GetRawState(const AppHall *hall)
{
  if (hall == 0)
  {
    return 0u;
  }

  return hall->raw_state;
}

uint8_t AppHall_GetSequenceIndex(const AppHall *hall)
{
  if (hall == 0)
  {
    return 0u;
  }

  return hall->sequence_index;
}

uint8_t AppHall_GetDirection(const AppHall *hall)
{
  if (hall == 0)
  {
    return 0u;
  }

  return hall->direction;
}

uint32_t AppHall_GetLastTransitionMs(const AppHall *hall)
{
  if (hall == 0)
  {
    return 0u;
  }

  return hall->last_transition_ms;
}
