#include <Arduino.h>
#include <SimpleFOC.h>
extern "C" {
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
#include "stm32g4xx_hal_spi.h"
}

// Motor + Hall assumptions for the current prototype:
// - DRV8301 in 3PWM mode
// - Hall sensor wires powered from 3V3 and connected directly to the MCU
// - Motor phase order: U=A, V=B, W=C
// - Pole-pairs for the currently connected motor: 2

static constexpr int kPolePairs = 2;
static constexpr uint8_t kNodeId = 1;
static constexpr uint8_t kSectionIndex = 0;
static constexpr uint16_t kGlobalControlId = 0x080;
static constexpr uint16_t kNodeCmdId = 0x100 + kNodeId;
static constexpr uint16_t kPresenceId = 0x140 + kNodeId;
static constexpr uint16_t kDrvDebugId = 0x160 + kNodeId;
static constexpr uint16_t kMotorDebugId = 0x1E0 + kNodeId;
static constexpr uint16_t kStatusId = 0x180 + kNodeId;
static constexpr uint32_t kCanTimeoutMs = 300;
static constexpr uint16_t kBringupMaxRpm = 120;
static constexpr float kOpenLoopRpmScale = 0.36f;
static constexpr bool kDrvOnlyBringup = false;

// PWM pins -> DRV8301 inputs in 3PWM mode.
// Original motor phase mapping: U=A, V=B, W=C.
static constexpr int kPwmA = PA8;
static constexpr int kPwmB = PA9;
static constexpr int kPwmC = PA10;

// Optional enable/fault
static constexpr int kDrvEnable = PB12;
static constexpr int kDrvFault = PB0;
static constexpr int kHeartbeatLed = PC6;

// DRV8301 SPI1 pins
static constexpr uint16_t kDrvCsPin = GPIO_PIN_4;
static GPIO_TypeDef *const kDrvCsPort = GPIOA;

// Hall pins
static constexpr int kHallA = PB6;
static constexpr int kHallB = PB7;
static constexpr int kHallC = PB11;

BLDCMotor motor(kPolePairs);
BLDCDriver3PWM driver(kPwmA, kPwmB, kPwmC);
HallSensor sensor(kHallA, kHallB, kHallC, kPolePairs);
Commander command(Serial);
FDCAN_HandleTypeDef hfdcan1;
SPI_HandleTypeDef hspi1;

struct CanControlState
{
  uint8_t system_mode = 0;
  uint8_t control_flags = 0;
  uint8_t node_command = 0;
  uint8_t node_flags = 0;
  uint16_t base_rpm = 0;
  int16_t trim_rpm_x10 = 0;
  uint16_t section_mask = 0;
  bool run_allowed = false;
  bool section_active = false;
  uint32_t last_global_rx_ms = 0;
  uint32_t last_node_rx_ms = 0;
  uint8_t alive_counter = 0;
} g_can;

float target_velocity = 0.0f;
uint16_t target_rpm = 0;
bool open_loop_debug = false;
uint8_t debug_tx_slot = 0;
uint8_t phase_test_mode = 0;

struct Drv8301State
{
  uint16_t status1 = 0;
  uint16_t status2 = 0;
  uint16_t control1 = 0;
  uint16_t control2 = 0;
  bool spi_ok = false;
  bool ctrl1_ok = false;
  bool ctrl2_ok = false;
} g_drv;

void doA() { sensor.handleA(); }
void doB() { sensor.handleB(); }
void doC() { sensor.handleC(); }

void onMotor(char *cmd)
{
  command.motor(&motor, cmd);
}

void onTarget(char *cmd)
{
  command.scalar(&target_velocity, cmd);
}

static uint16_t read_u16_le(const uint8_t *data)
{
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8);
}

static int16_t read_i16_le(const uint8_t *data)
{
  return (int16_t)read_u16_le(data);
}

static void write_u16_le(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value & 0xFFu);
  data[1] = (uint8_t)((value >> 8) & 0xFFu);
}

static uint16_t clamp_target_rpm(int32_t rpm)
{
  if (rpm <= 0)
    return 0;
  if (rpm >= kBringupMaxRpm)
    return kBringupMaxRpm;
  return (uint16_t)rpm;
}

static int16_t clamp_i16(int32_t value)
{
  if (value < -32768)
    return -32768;
  if (value > 32767)
    return 32767;
  return (int16_t)value;
}

static bool can_section_bit_is_set(uint16_t mask, uint8_t section_index)
{
  return (section_index < 16u) ? ((mask & (uint16_t)(1u << section_index)) != 0u) : false;
}

static float rpm_to_rad_per_sec(uint16_t rpm)
{
  return (float)rpm * 0.104719755f;
}

static uint32_t can_abs_diff_u32(uint32_t a, uint32_t b)
{
  return (a > b) ? (a - b) : (b - a);
}

static bool drv8301_spi_gpio_init()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  HAL_GPIO_WritePin(kDrvCsPort, kDrvCsPin, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = kDrvCsPin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(kDrvCsPort, &GPIO_InitStruct);
  return true;
}

extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
  if (hspi->Instance == SPI1)
  {
    (void)drv8301_spi_gpio_init();
  }
}

static bool drv8301_spi_init()
{
  if (!drv8301_spi_gpio_init())
  {
    return false;
  }

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  return HAL_SPI_Init(&hspi1) == HAL_OK;
}

static HAL_StatusTypeDef drv8301_transfer16(uint16_t tx_word, uint16_t *rx_word)
{
  uint8_t tx_buf[2] = {(uint8_t)(tx_word >> 8), (uint8_t)(tx_word & 0xFFu)};
  uint8_t rx_buf[2] = {};

  if (rx_word == nullptr)
  {
    return HAL_ERROR;
  }

  HAL_GPIO_WritePin(kDrvCsPort, kDrvCsPin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2u, 20u);
  HAL_GPIO_WritePin(kDrvCsPort, kDrvCsPin, GPIO_PIN_SET);

  if (status == HAL_OK)
  {
    *rx_word = (uint16_t)(((uint16_t)rx_buf[0] << 8) | rx_buf[1]);
  }
  return status;
}

static HAL_StatusTypeDef drv8301_read_register(uint8_t reg, uint16_t *value)
{
  uint16_t rx_word = 0;
  uint16_t cmd = (uint16_t)(0x8000u | (((uint16_t)reg & 0x0Fu) << 11));
  HAL_StatusTypeDef status = drv8301_transfer16(cmd, &rx_word);
  if (status != HAL_OK)
  {
    g_drv.spi_ok = false;
    return status;
  }

  status = drv8301_transfer16(0u, &rx_word);
  if (status != HAL_OK)
  {
    g_drv.spi_ok = false;
    return status;
  }

  *value = (uint16_t)(rx_word & 0x07FFu);
  g_drv.spi_ok = true;
  return HAL_OK;
}

static HAL_StatusTypeDef drv8301_write_register(uint8_t reg, uint16_t value)
{
  uint16_t rx_word = 0;
  uint16_t cmd = (uint16_t)((((uint16_t)reg & 0x0Fu) << 11) | (value & 0x07FFu));
  HAL_StatusTypeDef status = drv8301_transfer16(cmd, &rx_word);
  g_drv.spi_ok = (status == HAL_OK);
  return status;
}

static bool drv8301_read_all()
{
  bool ok = true;
  ok = (drv8301_read_register(0x00u, &g_drv.status1) == HAL_OK) && ok;
  ok = (drv8301_read_register(0x01u, &g_drv.status2) == HAL_OK) && ok;
  ok = (drv8301_read_register(0x02u, &g_drv.control1) == HAL_OK) && ok;
  ok = (drv8301_read_register(0x03u, &g_drv.control2) == HAL_OK) && ok;
  g_drv.spi_ok = ok;
  return ok;
}

static bool drv8301_configure()
{
  // DRV8301 Control Register 1 bit layout (TI datasheet, Rev. F):
  // D10..D6 = OC_ADJ_SET, D5..D4 = OCP_MODE, D3 = PWM_MODE, D2 = GATE_RESET, D1..D0 = GATE_CURRENT
  //
  // For bring-up in 3PWM mode we use:
  // - OC_ADJ_SET = 20 (0.648 V VDS threshold) to avoid overly sensitive false OC trips
  // - OCP_MODE   = 00 (current limit)
  // - PWM_MODE   = 1  (3 PWM inputs)
  // - GATE_RESET = 0  (normal)
  // - GATE_CURRENT = 00 (1.7 A peak gate drive)
  constexpr uint16_t kControl1Expected = (uint16_t)((20u << 6) | (1u << 3));

  // DRV8301 Control Register 2:
  // Keep the default/safe bring-up settings:
  // - report OT and OC on nOCTW
  // - shunt amp gain 10 V/V
  // - no DC calibration shorting
  // - cycle-by-cycle OC mode
  constexpr uint16_t kControl2Expected = 0x0000u;

  g_drv.ctrl1_ok = false;
  g_drv.ctrl2_ok = false;

  if (!drv8301_spi_init())
  {
    g_drv.spi_ok = false;
    return false;
  }

  // Bring the gate driver out of reset/standby before touching the control registers.
  digitalWrite(kDrvEnable, HIGH);
  HAL_Delay(2u);

  // Clear any latched gate-driver fault once after enable.
  (void)drv8301_write_register(0x02u, (uint16_t)(kControl1Expected | (1u << 2)));
  HAL_Delay(1u);

  if (drv8301_write_register(0x02u, kControl1Expected) != HAL_OK)
  {
    return false;
  }
  if (drv8301_read_register(0x02u, &g_drv.control1) != HAL_OK)
  {
    return false;
  }
  g_drv.ctrl1_ok = (g_drv.control1 == kControl1Expected);

  if (drv8301_write_register(0x03u, kControl2Expected) != HAL_OK)
  {
    return false;
  }
  if (drv8301_read_register(0x03u, &g_drv.control2) != HAL_OK)
  {
    return false;
  }
  g_drv.ctrl2_ok = (g_drv.control2 == kControl2Expected);

  (void)drv8301_read_all();
  return g_drv.spi_ok && g_drv.ctrl1_ok && g_drv.ctrl2_ok;
}

static bool can_prepare_clock_gpio()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();

  RCC_PeriphCLKInitTypeDef PeriphClkInit = {};
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
  PeriphClkInit.FdcanClockSelection = RCC_FDCANCLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    return false;
  }

  __HAL_RCC_FDCAN_CLK_ENABLE();

  GPIO_InitTypeDef GPIO_InitStruct = {};
  GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  return true;
}

static bool can_configure_250k_timing(FDCAN_HandleTypeDef *hfdcan)
{
  constexpr uint32_t kTargetBitrate = 250000u;
  uint32_t fdcan_clock_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_FDCAN);
  uint32_t best_error = 0xFFFFFFFFu;
  uint32_t best_prescaler = 0u;
  uint32_t best_tseg1 = 0u;
  uint32_t best_tseg2 = 0u;

  if (fdcan_clock_hz == 0u)
  {
    fdcan_clock_hz = HAL_RCC_GetPCLK1Freq();
  }

  if (fdcan_clock_hz == 0u)
  {
    return false;
  }

  for (uint32_t total_tq = 10u; total_tq <= 25u; ++total_tq)
  {
    uint32_t prescaler = (fdcan_clock_hz + ((kTargetBitrate * total_tq) / 2u)) / (kTargetBitrate * total_tq);
    if ((prescaler == 0u) || (prescaler > 512u))
    {
      continue;
    }

    uint32_t actual_bitrate = fdcan_clock_hz / (prescaler * total_tq);
    uint32_t error = can_abs_diff_u32(actual_bitrate, kTargetBitrate);
    if (error < best_error)
    {
      best_error = error;
      best_prescaler = prescaler;
      best_tseg2 = total_tq / 5u; // around 80% sample point
      if (best_tseg2 < 2u)
      {
        best_tseg2 = 2u;
      }
      best_tseg1 = total_tq - 1u - best_tseg2;
    }
  }

  if ((best_prescaler == 0u) || (best_tseg1 == 0u) || (best_tseg2 == 0u))
  {
    return false;
  }

  hfdcan->Init.NominalPrescaler = best_prescaler;
  hfdcan->Init.NominalSyncJumpWidth = (best_tseg2 > 4u) ? 4u : best_tseg2;
  hfdcan->Init.NominalTimeSeg1 = best_tseg1;
  hfdcan->Init.NominalTimeSeg2 = best_tseg2;
  return true;
}

static void can_update_run_permission()
{
  const bool drive_enable = (g_can.control_flags & (1u << 0)) != 0u;
  const bool estop = (g_can.control_flags & (1u << 2)) != 0u;
  // Bring-up default: keep SimpleFOC in open-loop unless closed-loop is explicitly requested.
  open_loop_debug = (g_can.control_flags & (1u << 3)) == 0u;
  const bool global_fresh = ((millis() - g_can.last_global_rx_ms) <= kCanTimeoutMs);
  const bool node_fresh = ((millis() - g_can.last_node_rx_ms) <= kCanTimeoutMs);
  const bool phase_test_request = node_fresh && (g_can.node_command >= 3u) && (g_can.node_command <= 5u);
  const bool cmd_enable = !node_fresh || (g_can.node_command == 1u) || ((g_can.node_flags & (1u << 0)) != 0u) || phase_test_request;

  // The bring-up flow is driven from S0_GLOBAL_CONTROL. S0_NODE_CMD remains optional.
  g_can.section_active = !node_fresh || can_section_bit_is_set(g_can.section_mask, kSectionIndex);
  g_can.run_allowed = global_fresh && drive_enable && !estop && cmd_enable && g_can.section_active;

  if (!g_can.run_allowed)
  {
    target_rpm = 0;
    target_velocity = 0.0f;
    phase_test_mode = 0;
    return;
  }

  phase_test_mode = (g_can.node_command >= 3u && g_can.node_command <= 5u) ? (uint8_t)(g_can.node_command - 2u) : 0u;
  const int32_t requested_rpm = (int32_t)g_can.base_rpm + ((int32_t)g_can.trim_rpm_x10 / 10);
  target_rpm = clamp_target_rpm(requested_rpm);
  target_velocity = rpm_to_rad_per_sec(target_rpm) * (open_loop_debug ? kOpenLoopRpmScale : 1.0f);
}

extern "C" void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *hfdcan)
{
  if (hfdcan->Instance != FDCAN1)
  {
    return;
  }

  (void)can_prepare_clock_gpio();
}

static HAL_StatusTypeDef can_send_std_frame(uint16_t can_id, const uint8_t data[8])
{
  FDCAN_TxHeaderTypeDef tx_header = {};
  tx_header.Identifier = can_id;
  tx_header.IdType = FDCAN_STANDARD_ID;
  tx_header.TxFrameType = FDCAN_DATA_FRAME;
  tx_header.DataLength = FDCAN_DLC_BYTES_8;
  tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  tx_header.BitRateSwitch = FDCAN_BRS_OFF;
  tx_header.FDFormat = FDCAN_CLASSIC_CAN;
  tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  tx_header.MessageMarker = 0;
  return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, (uint8_t *)data);
}

static bool can_init()
{
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 1;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 13;
  hfdcan1.Init.NominalTimeSeg2 = 2;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 0;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (!can_prepare_clock_gpio())
    return false;

  if (!can_configure_250k_timing(&hfdcan1))
    return false;

  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
    return false;

  if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_ACCEPT_IN_RX_FIFO0,
                                   FDCAN_REJECT_REMOTE,
                                   FDCAN_REJECT_REMOTE) != HAL_OK)
    return false;

  if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    return false;

  return true;
}

static void can_poll_rx()
{
  FDCAN_RxHeaderTypeDef rx_header = {};
  uint8_t rx_data[8] = {};

  while (HAL_FDCAN_GetRxFifoFillLevel(&hfdcan1, FDCAN_RX_FIFO0) > 0u)
  {
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK)
    {
      break;
    }

    if (rx_header.IdType != FDCAN_STANDARD_ID || rx_header.DataLength != FDCAN_DLC_BYTES_8)
    {
      continue;
    }

    if (rx_header.Identifier == kGlobalControlId)
    {
      g_can.system_mode = rx_data[0];
      g_can.control_flags = rx_data[1];
      g_can.base_rpm = read_u16_le(&rx_data[2]);
      g_can.last_global_rx_ms = millis();
      can_update_run_permission();
      continue;
    }

    if (rx_header.Identifier == kNodeCmdId)
    {
      g_can.node_command = rx_data[0];
      g_can.node_flags = rx_data[1];
      g_can.trim_rpm_x10 = read_i16_le(&rx_data[2]);
      g_can.section_mask = read_u16_le(&rx_data[4]);
      g_can.last_node_rx_ms = millis();
      can_update_run_permission();
    }
  }
}

static void can_send_presence()
{
  static uint32_t last_tx = 0;
  if (millis() - last_tx < 500)
    return;
  last_tx = millis();

  uint8_t payload[8] = {};
  payload[5] = 100u;
  write_u16_le(&payload[6], 0x1000u);
  (void)can_send_std_frame(kPresenceId, payload);
}

static void can_send_status()
{
  static uint32_t last_tx = 0;
  if (millis() - last_tx < 200)
    return;
  last_tx = millis();

  uint8_t payload[8] = {};
  uint8_t status_flags = 0u;
  if (g_can.run_allowed)
    status_flags |= 0x01u;
  if (g_can.section_active)
    status_flags |= 0x02u;
  if (digitalRead(kDrvFault) != LOW)
    status_flags |= 0x20u;
  status_flags |= 0x40u;
  if (((millis() - g_can.last_global_rx_ms) <= kCanTimeoutMs) ||
      ((millis() - g_can.last_node_rx_ms) <= kCanTimeoutMs))
    status_flags |= 0x80u;

  payload[0] = status_flags;
  payload[1] = 0u;
  write_u16_le(&payload[2], (uint16_t)fabsf(motor.shaft_velocity * 9.54929659f));
  write_u16_le(&payload[4], (uint16_t)(fmodf(motor.shaft_angle, _2PI) * (65535.0f / _2PI)));
  payload[6] = g_can.alive_counter++;
  payload[7] = 0u;
  (void)can_send_std_frame(kStatusId, payload);
}

static void can_send_motor_debug()
{
  static uint32_t last_tx = 0;
  if (millis() - last_tx < 200)
    return;
  last_tx = millis();

  const bool fault_active = digitalRead(kDrvFault) == LOW;
  const bool global_fresh = ((millis() - g_can.last_global_rx_ms) <= kCanTimeoutMs);
  const bool node_fresh = ((millis() - g_can.last_node_rx_ms) <= kCanTimeoutMs);
  uint8_t motor_flags = 0u;
  if (g_can.run_allowed)
    motor_flags |= 0x01u;
  if (fault_active)
    motor_flags |= 0x02u;
  if (global_fresh)
    motor_flags |= 0x04u;
  if (node_fresh)
    motor_flags |= 0x08u;
  if (g_can.section_active)
    motor_flags |= 0x10u;
  if (open_loop_debug)
    motor_flags |= 0x20u;
  if (phase_test_mode != 0u)
    motor_flags |= 0x40u;

  uint8_t payload[8] = {};
  write_u16_le(&payload[0], target_rpm);
  write_u16_le(&payload[2], (uint16_t)clamp_i16((int32_t)(motor.shaft_velocity * 9.54929659f)));
  write_u16_le(&payload[4], (uint16_t)clamp_i16((int32_t)(target_velocity * 10.0f)));
  payload[6] = (uint8_t)((digitalRead(kHallC) << 2) | (digitalRead(kHallB) << 1) | digitalRead(kHallA));
  payload[7] = motor_flags | ((phase_test_mode & 0x03u) << 6);
  (void)can_send_std_frame(kMotorDebugId, payload);
}

static void can_send_drv_debug()
{
  static uint32_t last_tx = 0;
  if (millis() - last_tx < 400)
    return;
  last_tx = millis();

  (void)drv8301_read_all();

  uint8_t payload[8] = {};
  payload[0] = (uint8_t)(g_drv.status1 & 0xFFu);
  payload[1] = (uint8_t)(g_drv.status2 & 0xFFu);
  payload[2] = (uint8_t)(g_drv.control1 & 0xFFu);
  payload[3] = (uint8_t)((g_drv.control1 >> 8) & 0xFFu);
  payload[4] = (uint8_t)(g_drv.control2 & 0xFFu);
  payload[5] = (uint8_t)((g_drv.control2 >> 8) & 0xFFu);
  payload[6] = (g_drv.ctrl1_ok ? 0x01u : 0x00u) |
               (g_drv.ctrl2_ok ? 0x02u : 0x00u) |
               (g_drv.spi_ok ? 0x04u : 0x00u) |
               (digitalRead(kDrvEnable) == HIGH ? 0x08u : 0x00u) |
               (digitalRead(kDrvFault) == LOW ? 0x10u : 0x00u);
  payload[7] = 0xA5u;
  (void)can_send_std_frame(kDrvDebugId, payload);
}

static void setup_driver_pins()
{
  pinMode(kDrvEnable, OUTPUT);
  digitalWrite(kDrvEnable, LOW);

  pinMode(kDrvFault, INPUT_PULLUP);
  pinMode(kHeartbeatLed, OUTPUT);
  digitalWrite(kHeartbeatLed, LOW);
}

void setup()
{
  setup_driver_pins();

  Serial.begin(115200);
  delay(500);
  Serial.println("SimpleFOC STM32G431 DRV8301 Hall POC");

  sensor.pullup = Pullup::USE_EXTERN;
  sensor.init();
  sensor.direction = Direction::CW;
  sensor.enableInterrupts(doA, doB, doC);

  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = 12.0f;
  driver.voltage_limit = 0.35f;
  driver.init();
  driver.disable();

  motor.linkDriver(&driver);
  motor.linkSensor(&sensor);

  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = 0.35f;
  motor.velocity_limit = rpm_to_rad_per_sec(kBringupMaxRpm);
  motor.PID_velocity.P = 0.03f;
  motor.PID_velocity.I = 0.05f;
  motor.PID_velocity.D = 0.0f;
  motor.PID_velocity.output_ramp = 50.0f;
  motor.LPF_velocity.Tf = 0.20f;

  motor.useMonitoring(Serial);
  motor.monitor_downsample = 100;

  if (digitalRead(kDrvFault) == LOW)
  {
    Serial.println("DRV fault pin active before initFOC()");
  }

  const bool drv_config_ok = drv8301_configure();
  delay(2);
  if (!drv_config_ok)
  {
    Serial.println("DRV8301 SPI configure failed");
  }

  if (!can_init())
  {
    Serial.println("FDCAN init failed");
  }

  if (!kDrvOnlyBringup)
  {
    motor.init();
    motor.initFOC();
  }

  command.add('M', onMotor, "motor");
  command.add('T', onTarget, "target velocity rad/s");

  Serial.println(kDrvOnlyBringup ? "Ready - DRV only bringup" : "Ready");
  Serial.println("Commands:");
  Serial.println("  T<value>   target velocity in rad/s");
  Serial.println("  M?         inspect motor params");
}

void loop()
{
  can_poll_rx();
  can_update_run_permission();
  if (kDrvOnlyBringup)
  {
    target_rpm = 0;
    target_velocity = 0.0f;
    phase_test_mode = 0;
    driver.disable();
    driver.setPwm(0.0f, 0.0f, 0.0f);
  }
  else if (phase_test_mode != 0u)
  {
    motor.disable();
    driver.enable();
    const float test_voltage = 0.35f;
    driver.setPwm(phase_test_mode == 1u ? test_voltage : 0.0f,
                  phase_test_mode == 2u ? test_voltage : 0.0f,
                  phase_test_mode == 3u ? test_voltage : 0.0f);
  }
  else
  {
    motor.enable();
    motor.controller = open_loop_debug ? MotionControlType::velocity_openloop : MotionControlType::velocity;
    if (!open_loop_debug)
    {
      motor.loopFOC();
    }
    motor.move(target_velocity);
  }
  command.run();
  switch (debug_tx_slot++ & 0x03u)
  {
  case 0:
    can_send_status();
    break;
  case 1:
    can_send_motor_debug();
    break;
  case 2:
    can_send_drv_debug();
    break;
  default:
    can_send_presence();
    break;
  }

  static uint32_t last_led = 0;
  static bool led_state = false;
  const bool fault_active = digitalRead(kDrvFault) == LOW;
  const uint32_t blink_period_ms = fault_active ? 100 : 250;
  if (millis() - last_led > blink_period_ms)
  {
    last_led = millis();
    led_state = !led_state;
    digitalWrite(kHeartbeatLed, led_state ? HIGH : LOW);
  }

  static uint32_t last_print = 0;
  if (millis() - last_print > 200)
  {
    last_print = millis();
    Serial.print("target=");
    Serial.print(target_velocity);
    Serial.print(" shaft_vel=");
    Serial.print(motor.shaft_velocity);
    Serial.print(" shaft_angle=");
    Serial.print(motor.shaft_angle);
    Serial.print(" hall_raw=");
    Serial.print((digitalRead(kHallC) << 2) | (digitalRead(kHallB) << 1) | digitalRead(kHallA));
    Serial.print(" fault=");
    Serial.println(fault_active ? "ACTIVE" : "OK");
  }
}
