#ifndef APP_HALL_H
#define APP_HALL_H

#include "stm32g4xx_hal.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint8_t initialized;
  uint8_t raw_state;
  uint8_t valid_state;
  uint8_t sequence_index;
  uint8_t direction;
  uint16_t rpm;
  uint16_t pos_q16;
  uint32_t transition_count;
  uint32_t last_transition_us;
  uint32_t last_transition_ms;
} AppHall;

HAL_StatusTypeDef AppHall_Init(AppHall *hall);
HAL_StatusTypeDef AppHall_Poll(AppHall *hall, uint32_t now_ms);
uint8_t AppHall_IsReady(const AppHall *hall);
uint16_t AppHall_GetRpm(const AppHall *hall);
uint16_t AppHall_GetPosQ16(const AppHall *hall);
uint8_t AppHall_GetRawState(const AppHall *hall);
uint8_t AppHall_GetSequenceIndex(const AppHall *hall);
uint8_t AppHall_GetDirection(const AppHall *hall);
uint32_t AppHall_GetLastTransitionMs(const AppHall *hall);

#ifdef __cplusplus
}
#endif

#endif
