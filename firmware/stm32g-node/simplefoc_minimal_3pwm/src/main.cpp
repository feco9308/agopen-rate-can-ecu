#include <Arduino.h>
#include <SimpleFOC.h>

extern "C" {
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
}

static constexpr int kPolePairs = 2;

static constexpr int kPwmA = PA8;
static constexpr int kPwmB = PA9;
static constexpr int kPwmC = PA10;
static constexpr int kPwmLowA = PB13;
static constexpr int kPwmLowB = PB14;
static constexpr int kPwmLowC = PB15;

static constexpr int kDrvEnable = PB12;
static constexpr int kDrvFault = PB0;
static constexpr int kHeartbeatLed = PC6;

static constexpr int kHallU = PB6;
static constexpr int kHallV = PB7;
static constexpr int kHallW = PB11;

static constexpr int kDebugTx = PA2;
static constexpr int kDebugRx = PA3;

static constexpr uint16_t kDrvCsPin = GPIO_PIN_4;
static GPIO_TypeDef *const kDrvCsPort = GPIOA;

static constexpr float kBusVoltage = 12.0f;
static constexpr float kVoltageLimit = 0.5f;
static constexpr float kTargetRpm = 25.0f;
static constexpr uint32_t kStartupAlignMs = 150u;

BLDCMotor motor(kPolePairs);
BLDCDriver3PWM driver(kPwmA, kPwmB, kPwmC);
SPI_HandleTypeDef hspi1;
HardwareSerial DebugSerial(kDebugRx, kDebugTx);

struct DrvState
{
  uint16_t status1 = 0;
  uint16_t status2 = 0;
  uint16_t control1 = 0;
  uint16_t control2 = 0;
  bool spi_ok = false;
  bool ctrl1_ok = false;
  bool ctrl2_ok = false;
} g_drv;

static float rpm_to_rad_per_sec(float rpm)
{
  return rpm * 0.104719755f;
}

static uint16_t read_u16_be(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void setup_driver_pins()
{
  pinMode(kDrvEnable, OUTPUT);
  digitalWrite(kDrvEnable, LOW);

  pinMode(kDrvFault, INPUT_PULLUP);
  pinMode(kHeartbeatLed, OUTPUT);
  digitalWrite(kHeartbeatLed, LOW);

  // Force the legacy low-side PWM pins to a passive state during 3PWM bringup.
  pinMode(kPwmLowA, OUTPUT);
  pinMode(kPwmLowB, OUTPUT);
  pinMode(kPwmLowC, OUTPUT);
  digitalWrite(kPwmLowA, LOW);
  digitalWrite(kPwmLowB, LOW);
  digitalWrite(kPwmLowC, LOW);

  pinMode(kHallU, INPUT_PULLUP);
  pinMode(kHallV, INPUT_PULLUP);
  pinMode(kHallW, INPUT_PULLUP);

}

static bool drv8301_spi_gpio_init()
{
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_SPI1_CLK_ENABLE();

  GPIO_InitTypeDef gpio = {};
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &gpio);

  HAL_GPIO_WritePin(kDrvCsPort, kDrvCsPin, GPIO_PIN_SET);
  gpio.Pin = kDrvCsPin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(kDrvCsPort, &gpio);
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

  HAL_GPIO_WritePin(kDrvCsPort, kDrvCsPin, GPIO_PIN_RESET);
  HAL_StatusTypeDef status = HAL_SPI_TransmitReceive(&hspi1, tx_buf, rx_buf, 2u, 20u);
  HAL_GPIO_WritePin(kDrvCsPort, kDrvCsPin, GPIO_PIN_SET);

  if (status == HAL_OK && rx_word != nullptr)
  {
    *rx_word = read_u16_be(rx_buf);
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

  if (value != nullptr)
  {
    *value = (uint16_t)(rx_word & 0x07FFu);
  }
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
  constexpr uint16_t kControl1Expected = (uint16_t)((20u << 6) | (1u << 3));
  constexpr uint16_t kControl2Expected = 0x0000u;

  g_drv.ctrl1_ok = false;
  g_drv.ctrl2_ok = false;

  if (!drv8301_spi_init())
  {
    g_drv.spi_ok = false;
    return false;
  }

  digitalWrite(kDrvEnable, HIGH);
  delay(2);

  (void)drv8301_write_register(0x02u, (uint16_t)(kControl1Expected | (1u << 2)));
  delay(1);

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

static int read_hall_raw()
{
  return (digitalRead(kHallW) << 2) | (digitalRead(kHallV) << 1) | digitalRead(kHallU);
}

void setup()
{
  setup_driver_pins();

  DebugSerial.begin(115200);
  delay(500);
  DebugSerial.println("SimpleFOC minimal 3PWM bringup");

  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = kBusVoltage;
  driver.voltage_limit = kVoltageLimit;
  driver.init();

  motor.linkDriver(&driver);
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::velocity_openloop;
  motor.voltage_limit = kVoltageLimit;
  motor.velocity_limit = rpm_to_rad_per_sec(kTargetRpm);

  const bool drv_ok = drv8301_configure();
  if (!drv_ok)
  {
    DebugSerial.println("DRV8301 configure failed");
  }

  motor.init();

  DebugSerial.print("DRV status1=0x");
  DebugSerial.println(g_drv.status1, HEX);
  DebugSerial.print("DRV status2=0x");
  DebugSerial.println(g_drv.status2, HEX);
  DebugSerial.print("DRV control1=0x");
  DebugSerial.println(g_drv.control1, HEX);
  DebugSerial.print("DRV control2=0x");
  DebugSerial.println(g_drv.control2, HEX);
  DebugSerial.println("Starting open-loop ramp after 1s");
}

void loop()
{
  static uint32_t boot_ms = millis();
  static uint32_t last_print = 0;
  static uint32_t last_led = 0;
  static bool led_state = false;

  const bool fault_active = digitalRead(kDrvFault) == LOW;
  const uint32_t now = millis();

  if (now - last_led > (fault_active ? 100u : 250u))
  {
    last_led = now;
    led_state = !led_state;
    digitalWrite(kHeartbeatLed, led_state ? HIGH : LOW);
  }

  if (fault_active)
  {
    motor.disable();
    driver.disable();
  }
  else
  {
    motor.enable();
    if (now < boot_ms + 1000u)
    {
      motor.move(0.0f);
    }
    else if (now < boot_ms + 1000u + kStartupAlignMs)
    {
      // Short static phase hold to overcome stiction and make the initial electrical angle deterministic.
      driver.setPwm(0.35f, 0.0f, 0.0f);
    }
    else
    {
      motor.move(rpm_to_rad_per_sec(kTargetRpm));
    }
  }

  if (now - last_print > 200u)
  {
    last_print = now;
    (void)drv8301_read_all();
    DebugSerial.print("fault=");
    DebugSerial.print(fault_active ? 1 : 0);
    DebugSerial.print(" hall=");
    DebugSerial.print(read_hall_raw());
    DebugSerial.print(" status1=0x");
    DebugSerial.print(g_drv.status1, HEX);
    DebugSerial.print(" status2=0x");
    DebugSerial.print(g_drv.status2, HEX);
    DebugSerial.print(" ctrl1=0x");
    DebugSerial.print(g_drv.control1, HEX);
    DebugSerial.print(" ctrl2=0x");
    DebugSerial.println(g_drv.control2, HEX);
  }
}
