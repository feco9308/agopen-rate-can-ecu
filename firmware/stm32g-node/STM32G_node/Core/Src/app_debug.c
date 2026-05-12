#include "app_debug.h"

#include <string.h>

void AppDebug_Init(AppDebugState *state)
{
  if (state == 0)
  {
    return;
  }

  memset(state, 0, sizeof(*state));
  state->stage = APP_DEBUG_STAGE_RESET;
  state->sensor = APP_DEBUG_BLOCK_OFF;
  state->driver = APP_DEBUG_BLOCK_OFF;
  state->control = APP_DEBUG_BLOCK_INIT;
}

void AppDebug_SetStage(AppDebugState *state, AppDebugStage stage)
{
  if (state == 0)
  {
    return;
  }

  state->stage = stage;
}

void AppDebug_SetFault(AppDebugState *state, AppDebugFault fault)
{
  if (state == 0)
  {
    return;
  }

  state->fault = fault;

  if (fault != APP_DEBUG_FAULT_NONE)
  {
    state->control = APP_DEBUG_BLOCK_FAULT;
  }
}

void AppDebug_ClearFault(AppDebugState *state, AppDebugFault fault)
{
  if (state == 0)
  {
    return;
  }

  if (state->fault == fault)
  {
    state->fault = APP_DEBUG_FAULT_NONE;
  }
}

void AppDebug_SetSensorState(AppDebugState *state, AppDebugBlockState block_state)
{
  if (state == 0)
  {
    return;
  }

  state->sensor = block_state;
}

void AppDebug_SetDriverState(AppDebugState *state, AppDebugBlockState block_state)
{
  if (state == 0)
  {
    return;
  }

  state->driver = block_state;
}

void AppDebug_SetControlState(AppDebugState *state, AppDebugBlockState block_state)
{
  if (state == 0)
  {
    return;
  }

  state->control = block_state;
}

void AppDebug_OnMainLoop(AppDebugState *state)
{
  if (state == 0)
  {
    return;
  }

  state->loop_counter++;
  state->stage = APP_DEBUG_STAGE_MAIN_LOOP;
}

void AppDebug_OnCanTx(AppDebugState *state, uint32_t now_ms)
{
  if (state == 0)
  {
    return;
  }

  state->can_tx_count++;
  state->last_can_tx_ms = now_ms;
}

void AppDebug_OnCanRx(AppDebugState *state, uint32_t now_ms)
{
  if (state == 0)
  {
    return;
  }

  state->can_rx_count++;
  state->last_can_rx_ms = now_ms;
}

uint8_t AppDebug_GetStatusFlags(const AppDebugState *state)
{
  uint8_t flags = 0u;

  if (state == 0)
  {
    return 0u;
  }

  if (state->fault != APP_DEBUG_FAULT_NONE)
  {
    flags |= 0x08u;
  }

  if (state->sensor == APP_DEBUG_BLOCK_READY)
  {
    flags |= 0x10u;
  }

  if (state->driver == APP_DEBUG_BLOCK_READY)
  {
    flags |= 0x20u;
  }

  if (state->control == APP_DEBUG_BLOCK_READY)
  {
    flags |= 0x40u;
  }

  if (state->can_rx_count != 0u)
  {
    flags |= 0x80u;
  }

  return flags;
}

uint8_t AppDebug_GetErrorCode(const AppDebugState *state)
{
  if (state == 0)
  {
    return 0u;
  }

  return (uint8_t)state->fault;
}

uint8_t AppDebug_GetWarningFlags(const AppDebugState *state)
{
  uint8_t warnings = 0u;

  if (state == 0)
  {
    return 0u;
  }

  warnings |= (uint8_t)((uint8_t)state->stage & 0x0Fu);
  warnings |= (uint8_t)(((uint8_t)state->sensor & 0x03u) << 4);
  warnings |= (uint8_t)(((uint8_t)state->driver & 0x03u) << 6);

  return warnings;
}
