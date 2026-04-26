#include "driver_test.h"

#include <Arduino.h>

#include "config.h"

extern "C" {
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
}

namespace driver_test {

namespace {

SPI_HandleTypeDef g_hspi1 = {};

struct DrvRegisters {
  uint16_t status1 = 0;
  uint16_t status2 = 0;
  uint16_t control1 = 0;
  uint16_t control2 = 0;
  bool spi_ok = false;
  bool control1_ok = false;
  bool control2_ok = false;
} g_drv;

bool g_driver_enabled = false;
bool g_fault_latched = false;
uint32_t g_boot_ms = 0;
uint32_t g_last_print_ms = 0;
uint32_t g_last_spi_read_ms = 0;

constexpr uint8_t kDrvRegStatus1 = 0x00u;
constexpr uint8_t kDrvRegStatus2 = 0x01u;
constexpr uint8_t kDrvRegControl1 = 0x02u;
constexpr uint8_t kDrvRegControl2 = 0x03u;
constexpr uint16_t kControl1Expected = 0x0040u;
constexpr uint16_t kControl2Expected = 0x0000u;

uint16_t readU16Be(const uint8_t* data) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

bool faultActive() {
  return digitalRead(config::kDrvFaultPin) == LOW;
}

void forceDisable() {
  digitalWrite(config::kDrvEnablePin, LOW);
  g_driver_enabled = false;
}

bool drvSpiGpioInit() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  gpio.Pin = GPIO_PIN_4;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = 0;
  HAL_GPIO_Init(GPIOA, &gpio);
  return true;
}

extern "C" void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {
  if (hspi->Instance == SPI1) {
    (void)drvSpiGpioInit();
  }
}

bool drvSpiInit() {
  if (!drvSpiGpioInit()) {
    return false;
  }

  g_hspi1.Instance = SPI1;
  g_hspi1.Init.Mode = SPI_MODE_MASTER;
  g_hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  g_hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  g_hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  g_hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  g_hspi1.Init.NSS = SPI_NSS_SOFT;
  g_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  g_hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  g_hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  g_hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  g_hspi1.Init.CRCPolynomial = 7;
  g_hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  g_hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  const bool ok = HAL_SPI_Init(&g_hspi1) == HAL_OK;
  g_drv.spi_ok = ok;
  return ok;
}

HAL_StatusTypeDef drvTransfer16(uint16_t tx_word, uint16_t* rx_word) {
  uint8_t tx_buf[2] = {static_cast<uint8_t>(tx_word >> 8), static_cast<uint8_t>(tx_word & 0xFFu)};
  uint8_t rx_buf[2] = {};

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  const HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(&g_hspi1, tx_buf, rx_buf, 2u, 20u);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

  if (status == HAL_OK && rx_word != nullptr) {
    *rx_word = readU16Be(rx_buf);
  }
  return status;
}

HAL_StatusTypeDef drvReadRegister(uint8_t reg, uint16_t* value) {
  uint16_t rx_word = 0;
  const uint16_t cmd = static_cast<uint16_t>(0x8000u | ((static_cast<uint16_t>(reg) & 0x0Fu) << 11));
  HAL_StatusTypeDef status = drvTransfer16(cmd, &rx_word);
  if (status != HAL_OK) {
    g_drv.spi_ok = false;
    return status;
  }

  status = drvTransfer16(0u, &rx_word);
  if (status != HAL_OK) {
    g_drv.spi_ok = false;
    return status;
  }

  if (value != nullptr) {
    *value = static_cast<uint16_t>(rx_word & 0x07FFu);
  }

  g_drv.spi_ok = true;
  return HAL_OK;
}

HAL_StatusTypeDef drvWriteRegister(uint8_t reg, uint16_t value) {
  uint16_t rx_word = 0;
  const uint16_t cmd =
      static_cast<uint16_t>(((static_cast<uint16_t>(reg) & 0x0Fu) << 11) | (value & 0x07FFu));
  const HAL_StatusTypeDef status = drvTransfer16(cmd, &rx_word);
  g_drv.spi_ok = (status == HAL_OK);
  return status;
}

bool drvReadAll() {
  bool ok = true;
  ok = (drvReadRegister(kDrvRegStatus1, &g_drv.status1) == HAL_OK) && ok;
  ok = (drvReadRegister(kDrvRegStatus2, &g_drv.status2) == HAL_OK) && ok;
  ok = (drvReadRegister(kDrvRegControl1, &g_drv.control1) == HAL_OK) && ok;
  ok = (drvReadRegister(kDrvRegControl2, &g_drv.control2) == HAL_OK) && ok;
  g_drv.control1_ok = (g_drv.control1 == kControl1Expected);
  g_drv.control2_ok = (g_drv.control2 == kControl2Expected);
  g_drv.spi_ok = ok;
  return ok;
}

bool drvConfigure() {
  g_drv.control1_ok = false;
  g_drv.control2_ok = false;

  if (drvWriteRegister(kDrvRegControl1, kControl1Expected) != HAL_OK) {
    return false;
  }
  if (drvWriteRegister(kDrvRegControl2, kControl2Expected) != HAL_OK) {
    return false;
  }
  return drvReadAll();
}

void printRegisters(Stream& serial) {
  serial.print(" spi_ok=");
  serial.print(g_drv.spi_ok ? "yes" : "no");
  serial.print(" status1=0x");
  serial.print(g_drv.status1, HEX);
  serial.print(" status2=0x");
  serial.print(g_drv.status2, HEX);
  serial.print(" ctrl1=0x");
  serial.print(g_drv.control1, HEX);
  serial.print(" ctrl2=0x");
  serial.print(g_drv.control2, HEX);
  serial.print(" ctrl_match=");
  serial.print((g_drv.control1_ok && g_drv.control2_ok) ? "yes" : "no");
}

void printStatus(Stream& serial, const char* reason) {
  serial.print("driver_test reason=");
  serial.print(reason);
  serial.print(" en=");
  serial.print(g_driver_enabled ? "HIGH" : "LOW");
  serial.print(" fault=");
  serial.print(faultActive() ? "ACTIVE" : "inactive");
  serial.print(" auto_start=");
  serial.print(config::kDriverEnableAutoStart ? "on" : "off");
  printRegisters(serial);
  serial.println();
}

}  // namespace

void begin(Stream& serial) {
  pinMode(config::kDrvEnablePin, OUTPUT);
  forceDisable();
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  g_fault_latched = false;
  g_boot_ms = millis();
  g_last_print_ms = 0;
  g_last_spi_read_ms = 0;
  g_drv = {};

  const bool spi_ok = drvSpiInit();

  serial.println("Driver enable test");
  serial.println("PWM outputs remain inactive.");
  serial.print("SPI init=");
  serial.println(spi_ok ? "ok" : "failed");
  if (spi_ok) {
    (void)drvReadAll();
    printStatus(serial, "spi_snapshot_boot");
  }
  serial.print("Driver will ");
  if (config::kDriverEnableAutoStart) {
    serial.print("auto-enable after ");
    serial.print(config::kDriverEnableDelayMs);
    serial.println(" ms if no fault is active.");
  } else {
    serial.println("stay disabled until config changes.");
  }
  printStatus(serial, "boot");
}

void update(Stream& serial) {
  const uint32_t now = millis();
  const bool fault_active = faultActive();

  if ((now - g_last_spi_read_ms) >= config::kDriverSpiReadbackIntervalMs) {
    g_last_spi_read_ms = now;
    (void)drvReadAll();
  }

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      (void)drvReadAll();
      forceDisable();
      g_fault_latched = true;
      printStatus(serial, "fault_shutdown");
    }
  } else if (g_fault_latched) {
    (void)drvReadAll();
    g_fault_latched = false;
    printStatus(serial, "fault_cleared");
  }

  if (!g_driver_enabled && !fault_active && config::kDriverEnableAutoStart &&
      (now - g_boot_ms) >= config::kDriverEnableDelayMs) {
    digitalWrite(config::kDrvEnablePin, HIGH);
    g_driver_enabled = true;
    delay(2);
    if (config::kDriverSpiAutoConfigure) {
      (void)drvConfigure();
    } else {
      (void)drvReadAll();
    }
    printStatus(serial, "enable_applied");
  }

  if ((now - g_last_print_ms) >= config::kStatusPrintIntervalMs) {
    g_last_print_ms = now;
    printStatus(serial, "periodic");
  }
}

bool driverEnabled() {
  return g_driver_enabled;
}

}  // namespace driver_test
