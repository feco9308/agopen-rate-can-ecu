#include "motor_test.h"

#include <Arduino.h>
#include <SimpleFOC.h>

#include "config.h"

extern "C" {
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
}

namespace motor_test {

namespace {

float rpmToRadPerSec(float rpm) {
  return rpm * 0.104719755f;
}

uint16_t readU16Be(const uint8_t* data) {
  return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

struct DrvRegisters {
  uint16_t status1 = 0;
  uint16_t status2 = 0;
  uint16_t control1 = 0;
  uint16_t control2 = 0;
  bool frame_fault = false;
  bool spi_ok = false;
  bool control_match = false;
};

SPI_HandleTypeDef g_hspi1 = {};
DrvRegisters g_drv = {};
BLDCMotor g_motor(config::kConfiguredPolePairs);
BLDCDriver3PWM g_driver(config::kPwmAPin, config::kPwmBPin, config::kPwmCPin);
HallSensor g_hall_sensor(config::kHallAPin, config::kHallBPin, config::kHallCPin,
                         config::kConfiguredPolePairs);

struct StepPattern {
  uint8_t pwm_a;
  uint8_t pwm_b;
  uint8_t pwm_c;
  const char* label;
};

constexpr StepPattern kForwardSteps[6] = {
    {config::kSixStepPwmDuty, 0, 0, "A+"},
    {config::kSixStepPwmDuty, config::kSixStepPwmDuty, 0, "A+B+"},
    {0, config::kSixStepPwmDuty, 0, "B+"},
    {0, config::kSixStepPwmDuty, config::kSixStepPwmDuty, "B+C+"},
    {0, 0, config::kSixStepPwmDuty, "C+"},
    {config::kSixStepPwmDuty, 0, config::kSixStepPwmDuty, "A+C+"},
};

constexpr uint8_t kHallSequence[6] = {0b001, 0b101, 0b100, 0b110, 0b010, 0b011};
constexpr uint8_t kDrvRegStatus1 = 0x00u;
constexpr uint8_t kDrvRegStatus2 = 0x01u;
constexpr uint8_t kDrvRegControl1 = 0x02u;
constexpr uint8_t kDrvRegControl2 = 0x03u;

enum Drv8301Control1Bits : uint16_t {
  kCtrl1OcAdjMask = 0x001Fu,
  kCtrl1OcpModeShift = 5,
  kCtrl1PwmModeBit = 7,
  kCtrl1GateResetBit = 8,
  kCtrl1GateCurrentShift = 9,
};

enum Drv8301Control2Bits : uint16_t {
  kCtrl2OcToffBit = 0,
  kCtrl2DcCalCh2Bit = 1,
  kCtrl2DcCalCh1Bit = 2,
  kCtrl2GainShift = 3,
  kCtrl2OctwModeShift = 5,
};

bool g_driver_enabled = false;
bool g_fault_latched = false;
bool g_openloop_started = false;
bool g_closedloop_started = false;
bool g_closedloop_startup_active = false;
bool g_drv_spi_stage_pwm = false;
uint8_t g_step_index = 0;
uint8_t g_last_hall_state = 0xFF;
uint32_t g_boot_ms = 0;
uint32_t g_last_print_ms = 0;
uint32_t g_last_phase_change_ms = 0;
uint32_t g_closedloop_enable_ms = 0;

bool faultActive() {
  return digitalRead(config::kDrvFaultPin) == LOW;
}

void doHallA() {
  g_hall_sensor.handleA();
}

void doHallB() {
  g_hall_sensor.handleB();
}

void doHallC() {
  g_hall_sensor.handleC();
}

uint16_t buildDrvControl1(bool gate_reset) {
  uint16_t value = 0;
  value |= static_cast<uint16_t>(config::kDrvOcpAdjustCode & 0x1Fu);
  value |= static_cast<uint16_t>((config::kDrvOcpModeCode & 0x03u) << kCtrl1OcpModeShift);
  if (config::kDrvUse3PwmMode) {
    value |= static_cast<uint16_t>(1u << kCtrl1PwmModeBit);
  }
  if (gate_reset) {
    value |= static_cast<uint16_t>(1u << kCtrl1GateResetBit);
  }
  value |= static_cast<uint16_t>((config::kDrvGateCurrentCode & 0x03u) << kCtrl1GateCurrentShift);
  return value;
}

uint16_t buildDrvControl2() {
  uint16_t value = 0;
  if (config::kDrvOcToff) {
    value |= static_cast<uint16_t>(1u << kCtrl2OcToffBit);
  }
  if (config::kDrvDcCalCh2) {
    value |= static_cast<uint16_t>(1u << kCtrl2DcCalCh2Bit);
  }
  if (config::kDrvDcCalCh1) {
    value |= static_cast<uint16_t>(1u << kCtrl2DcCalCh1Bit);
  }
  value |= static_cast<uint16_t>((config::kDrvShuntGainCode & 0x03u) << kCtrl2GainShift);
  value |= static_cast<uint16_t>((config::kDrvOctwModeCode & 0x03u) << kCtrl2OctwModeShift);
  return value;
}

uint8_t readHallState() {
  const uint8_t a = static_cast<uint8_t>(digitalRead(config::kHallAPin) == HIGH);
  const uint8_t b = static_cast<uint8_t>(digitalRead(config::kHallBPin) == HIGH);
  const uint8_t c = static_cast<uint8_t>(digitalRead(config::kHallCPin) == HIGH);
  return static_cast<uint8_t>((a << 2) | (b << 1) | c);
}

const StepPattern& activePattern() {
  return kForwardSteps[g_step_index % 6u];
}

void applyPattern(const StepPattern& pattern) {
  analogWrite(config::kPwmAPin, pattern.pwm_a);
  analogWrite(config::kPwmBPin, pattern.pwm_b);
  analogWrite(config::kPwmCPin, pattern.pwm_c);
}

void applySinglePhase(uint8_t phase_index) {
  analogWrite(config::kPwmAPin, phase_index == 0 ? config::kPhaseCheckPwmDuty : 0);
  analogWrite(config::kPwmBPin, phase_index == 1 ? config::kPhaseCheckPwmDuty : 0);
  analogWrite(config::kPwmCPin, phase_index == 2 ? config::kPhaseCheckPwmDuty : 0);
}

void applySinglePhaseStability(uint8_t phase_index) {
  analogWrite(config::kPwmAPin, phase_index == 0 ? config::kDrvSpiStabilityPwmDuty : 0);
  analogWrite(config::kPwmBPin, phase_index == 1 ? config::kDrvSpiStabilityPwmDuty : 0);
  analogWrite(config::kPwmCPin, phase_index == 2 ? config::kDrvSpiStabilityPwmDuty : 0);
}

void disableOutputs() {
  analogWrite(config::kPwmAPin, 0);
  analogWrite(config::kPwmBPin, 0);
  analogWrite(config::kPwmCPin, 0);
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

bool drvSpiInit() {
  if (!drvSpiGpioInit()) {
    return false;
  }

  g_hspi1.Instance = SPI1;
  g_hspi1.Init.Mode = SPI_MODE_MASTER;
  g_hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  g_hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  g_hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  g_hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  g_hspi1.Init.NSS = SPI_NSS_SOFT;
  g_hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  g_hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  g_hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  g_hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  g_hspi1.Init.CRCPolynomial = 7;
  g_hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  g_hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;

  g_drv.spi_ok = (HAL_SPI_Init(&g_hspi1) == HAL_OK);
  return g_drv.spi_ok;
}

HAL_StatusTypeDef drvTransfer16(uint16_t tx_word, uint16_t* rx_word) {
  uint16_t tx_buf = tx_word;
  uint16_t rx_buf = 0;
  delayMicroseconds(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
  delayMicroseconds(2);
  const HAL_StatusTypeDef status =
      HAL_SPI_TransmitReceive(&g_hspi1, reinterpret_cast<uint8_t*>(&tx_buf),
                              reinterpret_cast<uint8_t*>(&rx_buf), 1u, 20u);
  delayMicroseconds(2);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  delayMicroseconds(2);

  if (status == HAL_OK && rx_word != nullptr) {
    *rx_word = rx_buf;
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

  g_drv.frame_fault = ((rx_word & 0x8000u) != 0u);

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
  const uint16_t expected_ctrl1 = buildDrvControl1(false);
  const uint16_t expected_ctrl2 = buildDrvControl2();
  g_drv.control_match =
      (g_drv.control1 == expected_ctrl1) && (g_drv.control2 == expected_ctrl2);
  g_drv.spi_ok = ok;
  return ok;
}

bool drvConfigure3Pwm() {
  const uint16_t expected_ctrl1 = buildDrvControl1(false);
  const uint16_t reset_ctrl1 = buildDrvControl1(true);
  const uint16_t expected_ctrl2 = buildDrvControl2();

  (void)drvWriteRegister(kDrvRegControl1, reset_ctrl1);
  delay(1);

  if (drvWriteRegister(kDrvRegControl1, expected_ctrl1) != HAL_OK) {
    return false;
  }
  if (drvWriteRegister(kDrvRegControl2, expected_ctrl2) != HAL_OK) {
    return false;
  }
  return drvReadAll();
}

bool isValidHallState(uint8_t hall) {
  return hall != 0b000 && hall != 0b111;
}

int8_t hallIndex(uint8_t hall) {
  for (uint8_t index = 0; index < 6; ++index) {
    if (kHallSequence[index] == hall) {
      return static_cast<int8_t>(index);
    }
  }
  return -1;
}

void printHallBits(Stream& serial, uint8_t hall) {
  serial.print((hall >> 2) & 0x01);
  serial.print((hall >> 1) & 0x01);
  serial.print(hall & 0x01);
}

uint8_t commutationIndexFromHall(uint8_t hall) {
  const int8_t hall_index = hallIndex(hall);
  if (hall_index < 0) {
    return 0;
  }

  int8_t target_index = static_cast<int8_t>(hall_index + config::kSixStepHallAdvance);
  if (config::kSixStepReverseDirection) {
    target_index = static_cast<int8_t>(hall_index - config::kSixStepHallAdvance);
  }

  while (target_index < 0) {
    target_index += 6;
  }
  while (target_index >= 6) {
    target_index -= 6;
  }
  return static_cast<uint8_t>(target_index);
}

bool applyCommutationForHall(uint8_t hall) {
  if (!isValidHallState(hall)) {
    disableOutputs();
    return false;
  }

  g_step_index = commutationIndexFromHall(hall);
  applyPattern(activePattern());
  return true;
}

void printDrvRegisters(Stream& serial) {
  serial.print(" spi_ok=");
  serial.print(g_drv.spi_ok ? "yes" : "no");
  serial.print(" frame_fault=");
  serial.print(g_drv.frame_fault ? "yes" : "no");
  serial.print(" status1=0x");
  serial.print(g_drv.status1, HEX);
  serial.print(" status2=0x");
  serial.print(g_drv.status2, HEX);
  serial.print(" ctrl1=0x");
  serial.print(g_drv.control1, HEX);
  serial.print(" ctrl2=0x");
  serial.print(g_drv.control2, HEX);
  serial.print(" ctrl_match=");
  serial.print(g_drv.control_match ? "yes" : "no");
}

void printState(Stream& serial, const char* reason) {
  if (config::kTestMode == config::TEST_HALL_SENSOR_CHECK) {
    const uint8_t hall = readHallState();
    g_hall_sensor.update();
    serial.print("motor_test reason=");
    serial.print(reason);
    serial.print(" raw_hall=");
    printHallBits(serial, hall);
    serial.print(" raw_valid=");
    serial.print(isValidHallState(hall) ? "yes" : "no");
    serial.print(" hall_idx=");
    serial.print(hallIndex(hall));
    serial.print(" sensor_angle=");
    serial.print(g_hall_sensor.getAngle(), 4);
    serial.print(" sensor_vel=");
    serial.print(g_hall_sensor.getVelocity(), 4);
    serial.println();
    return;
  }

  if (config::kTestMode == config::TEST_DRIVER_SPI_STABILITY) {
    const uint8_t hall = readHallState();
    static const char* kPhaseLabels[3] = {"A", "B", "C"};
    serial.print("motor_test reason=");
    serial.print(reason);
    serial.print(" en=");
    serial.print(g_driver_enabled ? "HIGH" : "LOW");
    serial.print(" fault=");
    serial.print(faultActive() ? "ACTIVE" : "inactive");
    serial.print(" stage=");
    serial.print(g_drv_spi_stage_pwm ? "pwm_stress" : "no_pwm");
    serial.print(" phase=");
    serial.print(kPhaseLabels[g_step_index % 3u]);
    serial.print(" pwm=");
    serial.print(g_drv_spi_stage_pwm ? config::kDrvSpiStabilityPwmDuty : 0);
    serial.print(" hall=");
    printHallBits(serial, hall);
    serial.print(" valid=");
    serial.print(isValidHallState(hall) ? "yes" : "no");
    printDrvRegisters(serial);
    serial.println();
    return;
  }

  if (config::kTestMode == config::TEST_SIMPLEFOC_CLOSEDLOOP) {
    const uint8_t hall = readHallState();
    g_hall_sensor.update();
    serial.print("motor_test reason=");
    serial.print(reason);
    serial.print(" en=");
    serial.print(g_driver_enabled ? "HIGH" : "LOW");
    serial.print(" fault=");
    serial.print(faultActive() ? "ACTIVE" : "inactive");
    serial.print(" mode=");
    serial.print(g_closedloop_startup_active ? "startup" : "closed");
    serial.print(" target_rpm=");
    serial.print(config::kSimpleFocClosedloopTargetRpm, 1);
    serial.print(" shaft_vel=");
    serial.print(g_hall_sensor.getVelocity(), 3);
    serial.print(" shaft_angle=");
    serial.print(g_hall_sensor.getAngle(), 3);
    serial.print(" hall=");
    printHallBits(serial, hall);
    serial.print(" valid=");
    serial.print(isValidHallState(hall) ? "yes" : "no");
    printDrvRegisters(serial);
    serial.println();
    return;
  }

  if (config::kTestMode == config::TEST_PHASE_CHECK) {
    const uint8_t hall = readHallState();
    static const char* kPhaseLabels[3] = {"A", "B", "C"};
    serial.print("motor_test reason=");
    serial.print(reason);
    serial.print(" en=");
    serial.print(g_driver_enabled ? "HIGH" : "LOW");
    serial.print(" fault=");
    serial.print(faultActive() ? "ACTIVE" : "inactive");
    serial.print(" phase=");
    serial.print(kPhaseLabels[g_step_index % 3u]);
    serial.print(" pwm=");
    serial.print(config::kPhaseCheckPwmDuty);
    serial.print(" hall=");
    printHallBits(serial, hall);
    serial.print(" valid=");
    serial.print(isValidHallState(hall) ? "yes" : "no");
    printDrvRegisters(serial);
    serial.println();
    return;
  }

  if (config::kTestMode == config::TEST_SIMPLEFOC_OPENLOOP) {
    const uint8_t hall = readHallState();
    serial.print("motor_test reason=");
    serial.print(reason);
    serial.print(" en=");
    serial.print(g_driver_enabled ? "HIGH" : "LOW");
    serial.print(" fault=");
    serial.print(faultActive() ? "ACTIVE" : "inactive");
    serial.print(" target_rpm=");
    serial.print(config::kSimpleFocOpenloopTargetRpm, 1);
    serial.print(" hall=");
    printHallBits(serial, hall);
    serial.print(" valid=");
    serial.print(isValidHallState(hall) ? "yes" : "no");
    printDrvRegisters(serial);
    serial.println();
    return;
  }

  const StepPattern& pattern = activePattern();
  const uint8_t hall = readHallState();
  serial.print("motor_test reason=");
  serial.print(reason);
  serial.print(" en=");
  serial.print(g_driver_enabled ? "HIGH" : "LOW");
  serial.print(" fault=");
  serial.print(faultActive() ? "ACTIVE" : "inactive");
  serial.print(" step=");
  serial.print(g_step_index % 6u);
  serial.print(" pattern=");
  serial.print(pattern.label);
  serial.print(" hall_idx=");
  serial.print(hallIndex(hall));
  serial.print(" pwm=[");
  serial.print(pattern.pwm_a);
  serial.print(",");
  serial.print(pattern.pwm_b);
  serial.print(",");
  serial.print(pattern.pwm_c);
  serial.print("] hall=");
  serial.print((hall >> 2) & 0x01);
  serial.print((hall >> 1) & 0x01);
  serial.print(hall & 0x01);
  serial.print(" valid=");
  serial.println(isValidHallState(hall) ? "yes" : "no");
}

void beginSimpleFocOpenloop(Stream& serial) {
  pinMode(config::kDrvEnablePin, OUTPUT);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);
  digitalWrite(config::kDrvEnablePin, LOW);

  g_fault_latched = false;
  g_openloop_started = false;
  g_boot_ms = millis();
  g_last_print_ms = 0;
  g_drv = {};

  const bool spi_ok = drvSpiInit();
  if (spi_ok) {
    (void)drvReadAll();
  }

  g_driver.pwm_frequency = 20000;
  g_driver.voltage_power_supply = config::kBusVoltage;
  g_driver.voltage_limit = config::kSimpleFocOpenloopVoltageLimit;
  g_driver.init();

  g_motor.linkDriver(&g_driver);
  g_motor.torque_controller = TorqueControlType::voltage;
  g_motor.controller = MotionControlType::velocity_openloop;
  g_motor.voltage_limit = config::kSimpleFocOpenloopVoltageLimit;
  g_motor.velocity_limit = rpmToRadPerSec(config::kSimpleFocOpenloopTargetRpm);
  g_motor.init();

  serial.println("SimpleFOC 3PWM open-loop test");
  serial.println("Driver starts disabled, then DRV8301 is configured over SPI.");
  serial.print("Enable delay ms=");
  serial.print(config::kSimpleFocEnableDelayMs);
  serial.print(" target_rpm=");
  serial.print(config::kSimpleFocOpenloopTargetRpm, 1);
  serial.print(" voltage_limit=");
  serial.println(config::kSimpleFocOpenloopVoltageLimit, 2);
  serial.print("SPI init=");
  serial.println(spi_ok ? "ok" : "failed");
  printState(serial, "boot");
}

void beginSimpleFocClosedloop(Stream& serial) {
  pinMode(config::kDrvEnablePin, OUTPUT);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);
  digitalWrite(config::kDrvEnablePin, LOW);

  g_fault_latched = false;
  g_closedloop_started = false;
  g_closedloop_startup_active = false;
  g_boot_ms = millis();
  g_closedloop_enable_ms = 0;
  g_last_print_ms = 0;
  g_last_hall_state = readHallState();
  g_drv = {};

  const bool spi_ok = drvSpiInit();
  if (spi_ok) {
    (void)drvReadAll();
  }

  g_hall_sensor.pullup = Pullup::USE_INTERN;
  g_hall_sensor.init();
  g_hall_sensor.enableInterrupts(doHallA, doHallB, doHallC);

  g_driver.pwm_frequency = 20000;
  g_driver.voltage_power_supply = config::kBusVoltage;
  g_driver.voltage_limit = config::kSimpleFocClosedloopVoltageLimit;
  g_driver.init();

  g_motor.linkDriver(&g_driver);
  g_motor.linkSensor(&g_hall_sensor);
  g_motor.torque_controller = TorqueControlType::voltage;
  g_motor.controller = MotionControlType::velocity;
  g_motor.voltage_limit = config::kSimpleFocClosedloopVoltageLimit;
  g_motor.velocity_limit = rpmToRadPerSec(config::kSimpleFocClosedloopVelocityLimit);
  g_motor.PID_velocity.P = config::kSimpleFocClosedloopVelP;
  g_motor.PID_velocity.I = config::kSimpleFocClosedloopVelI;
  g_motor.PID_velocity.D = config::kSimpleFocClosedloopVelD;
  g_motor.LPF_velocity.Tf = config::kSimpleFocClosedloopVelLpfTf;
  g_motor.init();

  serial.println("SimpleFOC 3PWM closed-loop hall test");
  serial.println("Driver starts disabled, then DRV8301 is configured over SPI.");
  serial.print("Enable delay ms=");
  serial.print(config::kSimpleFocEnableDelayMs);
  serial.print(" target_rpm=");
  serial.print(config::kSimpleFocClosedloopTargetRpm, 1);
  serial.print(" voltage_limit=");
  serial.print(config::kSimpleFocClosedloopVoltageLimit, 2);
  serial.print(" velocity_limit_rpm=");
  serial.print(config::kSimpleFocClosedloopVelocityLimit, 1);
  serial.print(" startup_rpm=");
  serial.print(config::kSimpleFocClosedloopStartupRpm, 1);
  serial.print(" startup_ms=");
  serial.println(config::kSimpleFocClosedloopStartupMs);
  serial.print("SPI init=");
  serial.println(spi_ok ? "ok" : "failed");
  printState(serial, "boot");
}

void beginHallSensorCheck(Stream& serial) {
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);

  g_last_hall_state = readHallState();
  g_last_print_ms = 0;

  g_hall_sensor.pullup = Pullup::USE_INTERN;
  g_hall_sensor.init();
  g_hall_sensor.enableInterrupts(doHallA, doHallB, doHallC);

  serial.println("HallSensor integration check");
  serial.println("Rotate motor by hand. Compare raw_hall and sensor angle/velocity.");
  printState(serial, "boot");
}

void beginPhaseCheck(Stream& serial) {
  pinMode(config::kDrvEnablePin, OUTPUT);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);
  pinMode(config::kPwmAPin, OUTPUT);
  pinMode(config::kPwmBPin, OUTPUT);
  pinMode(config::kPwmCPin, OUTPUT);
  disableOutputs();

  g_fault_latched = false;
  g_driver_enabled = false;
  g_step_index = 0;
  g_last_hall_state = readHallState();
  g_boot_ms = millis();
  g_last_print_ms = 0;
  g_last_phase_change_ms = 0;
  g_drv = {};

  const bool spi_ok = drvSpiInit();
  if (spi_ok) {
    (void)drvReadAll();
  }

  serial.println("Phase check test");
  serial.println("One phase at a time with low PWM.");
  serial.print("Enable delay ms=");
  serial.print(config::kPhaseCheckEnableDelayMs);
  serial.print(" hold ms=");
  serial.print(config::kPhaseCheckHoldMs);
  serial.print(" pwm=");
  serial.println(config::kPhaseCheckPwmDuty);
  serial.print("SPI init=");
  serial.println(spi_ok ? "ok" : "failed");
  serial.println("Expect similar rotor holding feel on A, B and C.");
  printState(serial, "boot");
}

void beginDrvSpiStability(Stream& serial) {
  pinMode(config::kDrvEnablePin, OUTPUT);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);
  pinMode(config::kPwmAPin, OUTPUT);
  pinMode(config::kPwmBPin, OUTPUT);
  pinMode(config::kPwmCPin, OUTPUT);
  disableOutputs();

  g_fault_latched = false;
  g_driver_enabled = false;
  g_drv_spi_stage_pwm = false;
  g_step_index = 0;
  g_last_hall_state = readHallState();
  g_boot_ms = millis();
  g_last_print_ms = 0;
  g_last_phase_change_ms = 0;
  g_drv = {};

  const bool spi_ok = drvSpiInit();
  if (spi_ok) {
    (void)drvReadAll();
  }

  serial.println("DRV SPI stability test");
  serial.println("Stage 1: driver enabled, no PWM. Stage 2: low-PWM phase stress.");
  serial.print("Enable delay ms=");
  serial.print(config::kDrvSpiStabilityEnableDelayMs);
  serial.print(" no_pwm_ms=");
  serial.print(config::kDrvSpiStabilityNoPwmMs);
  serial.print(" step_ms=");
  serial.print(config::kDrvSpiStabilityStepMs);
  serial.print(" pwm=");
  serial.println(config::kDrvSpiStabilityPwmDuty);
  serial.print("SPI init=");
  serial.println(spi_ok ? "ok" : "failed");
  printState(serial, "boot");
}

void updatePhaseCheck(Stream& serial) {
  const uint32_t now = millis();
  const bool fault_active = faultActive();

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      disableOutputs();
      g_fault_latched = true;
      (void)drvReadAll();
      printState(serial, "fault_shutdown");
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    (void)drvReadAll();
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kPhaseCheckEnableDelayMs) {
    digitalWrite(config::kDrvEnablePin, HIGH);
    delay(2);
    g_driver_enabled = true;
    (void)drvConfigure3Pwm();
    applySinglePhase(g_step_index % 3u);
    g_last_phase_change_ms = now;
    printState(serial, "enable_applied");
  }

  if (g_driver_enabled && (now - g_last_phase_change_ms) >= config::kPhaseCheckHoldMs) {
    g_step_index = static_cast<uint8_t>((g_step_index + 1u) % 3u);
    applySinglePhase(g_step_index % 3u);
    g_last_phase_change_ms = now;
    printState(serial, "phase_advance");
  }

  if ((now - g_last_print_ms) >= config::kPhaseCheckStatusIntervalMs) {
    g_last_print_ms = now;
    (void)drvReadAll();
    printState(serial, "periodic");
  }
}

void updateDrvSpiStability(Stream& serial) {
  const uint32_t now = millis();
  const bool fault_active = faultActive();
  const uint16_t prev_ctrl1 = g_drv.control1;
  const uint16_t prev_ctrl2 = g_drv.control2;
  const bool prev_match = g_drv.control_match;

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      disableOutputs();
      g_fault_latched = true;
      (void)drvReadAll();
      printState(serial, "fault_shutdown");
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    (void)drvReadAll();
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kDrvSpiStabilityEnableDelayMs) {
    digitalWrite(config::kDrvEnablePin, HIGH);
    delay(2);
    g_driver_enabled = true;
    (void)drvConfigure3Pwm();
    g_last_phase_change_ms = now;
    printState(serial, "enable_applied");
  }

  if (g_driver_enabled && !g_drv_spi_stage_pwm &&
      (now - g_last_phase_change_ms) >= config::kDrvSpiStabilityNoPwmMs) {
    g_drv_spi_stage_pwm = true;
    g_step_index = 0;
    applySinglePhaseStability(g_step_index % 3u);
    g_last_phase_change_ms = now;
    printState(serial, "pwm_stress_start");
  }

  if (g_driver_enabled && g_drv_spi_stage_pwm &&
      (now - g_last_phase_change_ms) >= config::kDrvSpiStabilityStepMs) {
    g_step_index = static_cast<uint8_t>((g_step_index + 1u) % 3u);
    applySinglePhaseStability(g_step_index % 3u);
    g_last_phase_change_ms = now;
    printState(serial, "phase_advance");
  }

  if ((now - g_last_print_ms) >= config::kDrvSpiStabilityStatusIntervalMs) {
    g_last_print_ms = now;
    (void)drvReadAll();
    if (g_drv.control1 != prev_ctrl1 || g_drv.control2 != prev_ctrl2 || g_drv.control_match != prev_match) {
      printState(serial, "reg_change");
    } else {
      printState(serial, "periodic");
    }
  }
}

void updateSimpleFocOpenloop(Stream& serial) {
  const uint32_t now = millis();
  const bool fault_active = faultActive();
  const uint8_t hall = readHallState();

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      g_motor.disable();
      g_driver.disable();
      digitalWrite(config::kDrvEnablePin, LOW);
      g_driver_enabled = false;
      g_openloop_started = false;
      g_fault_latched = true;
      (void)drvReadAll();
      printState(serial, "fault_shutdown");
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    (void)drvReadAll();
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kSimpleFocEnableDelayMs) {
    digitalWrite(config::kDrvEnablePin, HIGH);
    delay(2);
    g_driver_enabled = true;
    (void)drvConfigure3Pwm();
    g_driver.enable();
    g_motor.enable();
    g_openloop_started = true;
    g_last_hall_state = hall;
    printState(serial, "enable_applied");
  }

  if (g_openloop_started) {
    g_motor.move(rpmToRadPerSec(config::kSimpleFocOpenloopTargetRpm));
  } else {
    g_motor.move(0.0f);
  }

  if ((now - g_last_print_ms) >= config::kSimpleFocStatusIntervalMs) {
    g_last_print_ms = now;
    (void)drvReadAll();
    printState(serial, "periodic");
  }

  if (config::kSimpleFocLogHallChanges && g_driver_enabled && hall != g_last_hall_state) {
    serial.print("motor_test reason=hall_change prev=");
    printHallBits(serial, g_last_hall_state);
    serial.print(" next=");
    printHallBits(serial, hall);
    serial.print(" prev_idx=");
    serial.print(hallIndex(g_last_hall_state));
    serial.print(" next_idx=");
    serial.print(hallIndex(hall));
    serial.print(" valid=");
    serial.print(isValidHallState(hall) ? "yes" : "no");
    serial.print(" target_rpm=");
    serial.print(config::kSimpleFocOpenloopTargetRpm, 1);
    printDrvRegisters(serial);
    serial.println();
    g_last_hall_state = hall;
  }
}

void updateSimpleFocClosedloop(Stream& serial) {
  const uint32_t now = millis();
  const bool fault_active = faultActive();
  const uint8_t hall = readHallState();

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      g_motor.disable();
      g_driver.disable();
      digitalWrite(config::kDrvEnablePin, LOW);
      g_driver_enabled = false;
      g_closedloop_started = false;
      g_fault_latched = true;
      (void)drvReadAll();
      printState(serial, "fault_shutdown");
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    (void)drvReadAll();
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kSimpleFocEnableDelayMs) {
    digitalWrite(config::kDrvEnablePin, HIGH);
    delay(2);
    g_driver_enabled = true;
    (void)drvConfigure3Pwm();
    g_driver.enable();
    g_motor.enable();
    g_motor.initFOC();
    g_closedloop_started = true;
    g_closedloop_startup_active = config::kSimpleFocClosedloopUseStartupAssist;
    g_closedloop_enable_ms = now;
    g_motor.controller = g_closedloop_startup_active ? MotionControlType::velocity_openloop
                                                     : MotionControlType::velocity;
    g_last_hall_state = hall;
    printState(serial, "enable_applied");
  }

  if (g_closedloop_started) {
    const float target_sign = config::kSimpleFocClosedloopReverse ? -1.0f : 1.0f;
    if (g_closedloop_startup_active) {
      const uint32_t elapsed_ms = now - g_closedloop_enable_ms;
      if (elapsed_ms >= config::kSimpleFocClosedloopStartupMs) {
        g_closedloop_startup_active = false;
        g_motor.controller = MotionControlType::velocity;
        printState(serial, "startup_handoff");
      } else {
        g_motor.move(rpmToRadPerSec(config::kSimpleFocClosedloopStartupRpm) * target_sign);
      }
    }

    if (!g_closedloop_startup_active) {
      g_motor.loopFOC();
      g_motor.move(rpmToRadPerSec(config::kSimpleFocClosedloopTargetRpm) * target_sign);
    }
  } else {
    g_motor.move(0.0f);
  }

  if ((now - g_last_print_ms) >= config::kSimpleFocStatusIntervalMs) {
    g_last_print_ms = now;
    (void)drvReadAll();
    printState(serial, "periodic");
  }

  if (config::kSimpleFocLogHallChanges && g_driver_enabled && hall != g_last_hall_state) {
    serial.print("motor_test reason=hall_change prev=");
    printHallBits(serial, g_last_hall_state);
    serial.print(" next=");
    printHallBits(serial, hall);
    serial.print(" prev_idx=");
    serial.print(hallIndex(g_last_hall_state));
    serial.print(" next_idx=");
    serial.print(hallIndex(hall));
    serial.print(" valid=");
    serial.print(isValidHallState(hall) ? "yes" : "no");
    serial.print(" target_rpm=");
    serial.print(config::kSimpleFocClosedloopTargetRpm, 1);
    printDrvRegisters(serial);
    serial.println();
    g_last_hall_state = hall;
  }
}

void updateHallSensorCheck(Stream& serial) {
  const uint32_t now = millis();
  const uint8_t hall = readHallState();
  g_hall_sensor.update();

  if (hall != g_last_hall_state) {
    g_last_hall_state = hall;
    printState(serial, "hall_change");
  }

  if ((now - g_last_print_ms) >= config::kHallSensorCheckStatusIntervalMs) {
    g_last_print_ms = now;
    printState(serial, "periodic");
  }
}

}  // namespace

void begin(Stream& serial) {
  if (config::kTestMode == config::TEST_HALL_SENSOR_CHECK) {
    beginHallSensorCheck(serial);
    return;
  }

  if (config::kTestMode == config::TEST_DRIVER_SPI_STABILITY) {
    beginDrvSpiStability(serial);
    return;
  }

  if (config::kTestMode == config::TEST_SIMPLEFOC_CLOSEDLOOP) {
    beginSimpleFocClosedloop(serial);
    return;
  }

  if (config::kTestMode == config::TEST_PHASE_CHECK) {
    beginPhaseCheck(serial);
    return;
  }

  if (config::kTestMode == config::TEST_SIMPLEFOC_OPENLOOP) {
    beginSimpleFocOpenloop(serial);
    return;
  }

  pinMode(config::kDrvEnablePin, OUTPUT);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);
  pinMode(config::kPwmAPin, OUTPUT);
  pinMode(config::kPwmBPin, OUTPUT);
  pinMode(config::kPwmCPin, OUTPUT);
  disableOutputs();

  g_fault_latched = false;
  g_step_index = 0;
  g_last_hall_state = readHallState();
  g_boot_ms = millis();
  g_last_print_ms = 0;

  serial.println("6-step hall commutation test");
  serial.println("Driver starts disabled and outputs are zero.");
  serial.print("Enable delay ms=");
  serial.print(config::kSixStepEnableDelayMs);
  serial.print(" pwm duty=");
  serial.print(config::kSixStepPwmDuty);
  serial.print(" hall advance=");
  serial.print(config::kSixStepHallAdvance);
  serial.print(" reverse=");
  serial.println(config::kSixStepReverseDirection ? "yes" : "no");
  serial.println("Keep motor unloaded. Stop immediately on harsh vibration.");
  printState(serial, "boot");
}

void update(Stream& serial) {
  if (config::kTestMode == config::TEST_HALL_SENSOR_CHECK) {
    updateHallSensorCheck(serial);
    return;
  }

  if (config::kTestMode == config::TEST_DRIVER_SPI_STABILITY) {
    updateDrvSpiStability(serial);
    return;
  }

  if (config::kTestMode == config::TEST_SIMPLEFOC_CLOSEDLOOP) {
    updateSimpleFocClosedloop(serial);
    return;
  }

  if (config::kTestMode == config::TEST_PHASE_CHECK) {
    updatePhaseCheck(serial);
    return;
  }

  if (config::kTestMode == config::TEST_SIMPLEFOC_OPENLOOP) {
    updateSimpleFocOpenloop(serial);
    return;
  }

  const uint32_t now = millis();
  const bool fault_active = faultActive();

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      disableOutputs();
      g_fault_latched = true;
      printState(serial, "fault_shutdown");
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kSixStepEnableDelayMs) {
    digitalWrite(config::kDrvEnablePin, HIGH);
    g_driver_enabled = true;
    const bool commutation_ok = applyCommutationForHall(readHallState());
    printState(serial, commutation_ok ? "enable_applied" : "enable_invalid_hall");
  }

  const uint8_t hall = readHallState();
  if (g_driver_enabled && hall != g_last_hall_state) {
    g_last_hall_state = hall;
    const bool commutation_ok = applyCommutationForHall(hall);
    printState(serial, commutation_ok ? "hall_update" : "invalid_hall");
  }

  if ((now - g_last_print_ms) >= config::kSixStepStatusIntervalMs) {
    g_last_print_ms = now;
    printState(serial, "periodic");
  }
}

bool driverEnabled() {
  return g_driver_enabled;
}

}  // namespace motor_test
