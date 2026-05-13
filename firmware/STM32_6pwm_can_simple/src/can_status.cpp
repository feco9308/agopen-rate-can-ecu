#include "can_status.h"

#include "config.h"

extern "C" {
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"
}

namespace can_status {
namespace {

FDCAN_HandleTypeDef g_hfdcan1 = {};
bool g_ready = false;
uint32_t g_last_tx_ms = 0;

void gpioInit() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_FDCAN_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_11 | GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF9_FDCAN1;
  HAL_GPIO_Init(GPIOA, &gpio);
}

int16_t scaledI16(float value, float scale) {
  const float scaled = value * scale;
  if (scaled > 32767.0f) {
    return 32767;
  }
  if (scaled < -32768.0f) {
    return -32768;
  }
  return static_cast<int16_t>(scaled);
}

void putI16(uint8_t* data, uint8_t offset, int16_t value) {
  data[offset] = static_cast<uint8_t>(value & 0xFF);
  data[offset + 1] = static_cast<uint8_t>((static_cast<uint16_t>(value) >> 8) & 0xFF);
}

}  // namespace

bool begin() {
  gpioInit();

  g_hfdcan1.Instance = FDCAN1;
  g_hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  g_hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  g_hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  g_hfdcan1.Init.AutoRetransmission = ENABLE;
  g_hfdcan1.Init.TransmitPause = DISABLE;
  g_hfdcan1.Init.ProtocolException = DISABLE;
  g_hfdcan1.Init.NominalPrescaler = 20;
  g_hfdcan1.Init.NominalSyncJumpWidth = 3;
  g_hfdcan1.Init.NominalTimeSeg1 = 13;
  g_hfdcan1.Init.NominalTimeSeg2 = 3;
  g_hfdcan1.Init.DataPrescaler = 20;
  g_hfdcan1.Init.DataSyncJumpWidth = 3;
  g_hfdcan1.Init.DataTimeSeg1 = 13;
  g_hfdcan1.Init.DataTimeSeg2 = 3;
  g_hfdcan1.Init.StdFiltersNbr = 1;
  g_hfdcan1.Init.ExtFiltersNbr = 0;
  g_hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

  if (HAL_FDCAN_Init(&g_hfdcan1) != HAL_OK) {
    g_ready = false;
    return false;
  }

  FDCAN_FilterTypeDef filter = {};
  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = 0;
  filter.FilterID2 = 0;
  (void)HAL_FDCAN_ConfigFilter(&g_hfdcan1, &filter);
  (void)HAL_FDCAN_ConfigGlobalFilter(&g_hfdcan1, FDCAN_ACCEPT_IN_RX_FIFO0,
                                     FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE);

  g_ready = (HAL_FDCAN_Start(&g_hfdcan1) == HAL_OK);
  return g_ready;
}

void tick(const AppStatus& status, uint32_t now) {
  if (!g_ready || (now - g_last_tx_ms) < config::kCanStatusIntervalMs) {
    return;
  }
  if (HAL_FDCAN_GetTxFifoFreeLevel(&g_hfdcan1) == 0u) {
    return;
  }

  FDCAN_TxHeaderTypeDef header = {};
  header.Identifier = config::kCanStatusId;
  header.IdType = FDCAN_STANDARD_ID;
  header.TxFrameType = FDCAN_DATA_FRAME;
  header.DataLength = FDCAN_DLC_BYTES_8;
  header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  header.BitRateSwitch = FDCAN_BRS_OFF;
  header.FDFormat = FDCAN_CLASSIC_CAN;
  header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  header.MessageMarker = 0;

  uint8_t data[8] = {};
  data[0] = static_cast<uint8_t>((status.driver_enabled ? 0x01u : 0u) |
                                 (status.driver_ready ? 0x02u : 0u) |
                                 (status.fault_active ? 0x04u : 0u) |
                                 (status.closed_loop ? 0x08u : 0u) |
                                 (status.startup_assist ? 0x10u : 0u));
  data[1] = status.hall_state & 0x07u;
  putI16(data, 2, scaledI16(status.target_rad_s, 100.0f));
  putI16(data, 4, scaledI16(status.measured_rad_s, 100.0f));
  putI16(data, 6, scaledI16(config::kBusVoltage, 100.0f));

  if (HAL_FDCAN_AddMessageToTxFifoQ(&g_hfdcan1, &header, data) == HAL_OK) {
    g_last_tx_ms = now;
  }
}

bool ready() {
  return g_ready;
}

}  // namespace can_status
