#ifndef APP_DEBUG_H
#define APP_DEBUG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  APP_DEBUG_STAGE_RESET = 0,
  APP_DEBUG_STAGE_CLOCK_OK = 1,
  APP_DEBUG_STAGE_GPIO_OK = 2,
  APP_DEBUG_STAGE_FDCAN_INIT_OK = 3,
  APP_DEBUG_STAGE_FDCAN_START_OK = 4,
  APP_DEBUG_STAGE_NODE_INIT_OK = 5,
  APP_DEBUG_STAGE_MAIN_LOOP = 6
} AppDebugStage;

typedef enum
{
  APP_DEBUG_FAULT_NONE = 0,
  APP_DEBUG_FAULT_CLOCK = 1,
  APP_DEBUG_FAULT_FDCAN_INIT = 2,
  APP_DEBUG_FAULT_FDCAN_START = 3,
  APP_DEBUG_FAULT_CAN_TX = 4,
  APP_DEBUG_FAULT_CAN_RX = 5,
  APP_DEBUG_FAULT_SENSOR_INIT = 6,
  APP_DEBUG_FAULT_SENSOR_READ = 7,
  APP_DEBUG_FAULT_DRIVER_INIT = 8,
  APP_DEBUG_FAULT_MOTOR_CONTROL = 9,
  APP_DEBUG_FAULT_DRIVER_SPI_VERIFY = 10,
  APP_DEBUG_FAULT_DRIVER_SPI_VERIFY_CTRL1 = 11
} AppDebugFault;

typedef enum
{
  APP_DEBUG_BLOCK_OFF = 0,
  APP_DEBUG_BLOCK_INIT = 1,
  APP_DEBUG_BLOCK_READY = 2,
  APP_DEBUG_BLOCK_FAULT = 3
} AppDebugBlockState;

typedef struct
{
  AppDebugStage stage;
  AppDebugFault fault;
  AppDebugBlockState sensor;
  AppDebugBlockState driver;
  AppDebugBlockState control;
  uint32_t loop_counter;
  uint32_t can_tx_count;
  uint32_t can_rx_count;
  uint32_t last_can_tx_ms;
  uint32_t last_can_rx_ms;
} AppDebugState;

void AppDebug_Init(AppDebugState *state);
void AppDebug_SetStage(AppDebugState *state, AppDebugStage stage);
void AppDebug_SetFault(AppDebugState *state, AppDebugFault fault);
void AppDebug_ClearFault(AppDebugState *state, AppDebugFault fault);
void AppDebug_SetSensorState(AppDebugState *state, AppDebugBlockState block_state);
void AppDebug_SetDriverState(AppDebugState *state, AppDebugBlockState block_state);
void AppDebug_SetControlState(AppDebugState *state, AppDebugBlockState block_state);
void AppDebug_OnMainLoop(AppDebugState *state);
void AppDebug_OnCanTx(AppDebugState *state, uint32_t now_ms);
void AppDebug_OnCanRx(AppDebugState *state, uint32_t now_ms);
uint8_t AppDebug_GetStatusFlags(const AppDebugState *state);
uint8_t AppDebug_GetErrorCode(const AppDebugState *state);
uint8_t AppDebug_GetWarningFlags(const AppDebugState *state);

#ifdef __cplusplus
}
#endif

#endif
