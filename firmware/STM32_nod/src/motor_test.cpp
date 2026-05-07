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
BLDCDriver3PWM g_driver3(config::kPwmAPin, config::kPwmBPin, config::kPwmCPin);
BLDCDriver6PWM g_driver6(config::kPwmAPin, config::kPwmALowPin, config::kPwmBPin,
                         config::kPwmBLowPin, config::kPwmCPin, config::kPwmCLowPin);
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
bool g_drv_configured = false;
bool g_fault_latched = false;
bool g_openloop_started = false;
bool g_openloop_debug_started = false;
bool g_closedloop_started = false;
bool g_closedloop_startup_active = false;
bool g_drv_spi_stage_pwm = false;
bool g_sixstep_startup_active = false;
uint8_t g_step_index = 0;
uint8_t g_last_hall_state = 0xFF;
uint32_t g_boot_ms = 0;
uint32_t g_last_print_ms = 0;
uint32_t g_last_phase_change_ms = 0;
uint32_t g_sixstep_enable_ms = 0;
uint32_t g_sixstep_last_kick_ms = 0;
uint32_t g_closedloop_enable_ms = 0;
uint32_t g_openloop_enable_ms = 0;
uint32_t g_openloop_debug_step_ms = 0;
uint32_t g_openloop_step_interval_ms = 0;
float g_openloop_debug_angle_el = 0.0f;
float g_openloop_debug_voltage = 0.0f;
float g_openloop_target_rad_s = 0.0f;
bool g_openloop_move_called = false;
uint8_t g_sixstep_current_duty = config::kSixStepPwmDuty;
bool g_closedloop_loopfoc_called = false;
bool g_closedloop_move_called = false;
bool g_closedloop_initfoc_started = false;
bool g_closedloop_initfoc_done = false;
bool g_closedloop_handoff_allowed = false;
bool g_closedloop_handoff_evaluated = false;
float g_closedloop_startup_target_rad_s = 0.0f;
float g_closedloop_measured_rad_s_at_handoff = 0.0f;
int32_t g_closedloop_hall_forward_count = 0;
int32_t g_closedloop_hall_reverse_count = 0;
int32_t g_closedloop_hall_jump_count = 0;
int32_t g_closedloop_hall_invalid_count = 0;
int8_t g_closedloop_last_hall_idx = -1;
int8_t g_closedloop_hall_delta = 0;
volatile uint32_t hall_a_irq_count = 0;
volatile uint32_t hall_b_irq_count = 0;
volatile uint32_t hall_c_irq_count = 0;
uint32_t g_last_hall_a_irq_count = 0;
uint32_t g_last_hall_b_irq_count = 0;
uint32_t g_last_hall_c_irq_count = 0;
uint32_t g_hall_a_irq_delta = 0;
uint32_t g_hall_b_irq_delta = 0;
uint32_t g_hall_c_irq_delta = 0;
float g_last_sensor_angle = 0.0f;
float g_last_sensor_velocity = 0.0f;
bool g_sensor_angle_changes = false;
bool g_sensor_velocity_valid = false;
bool g_hall_irq_counts_increasing = false;

void printState(Stream& serial, const char* reason);
bool isValidHallState(uint8_t hall);
int8_t hallIndex(uint8_t hall);

bool isClosedloop6PwmMode() {
  return config::kTestMode == config::TEST_SIMPLEFOC_CLOSEDLOOP_6PWM;
}

float closedloopMeasuredRadS() {
  return g_hall_sensor.getVelocity();
}

float closedloopTargetRadS() {
  return rpmToRadPerSec(config::kSimpleFocClosedloopTargetRpm) *
         (config::kSimpleFocClosedloopReverse ? -1.0f : 1.0f);
}

void recordClosedloopHallTransition(uint8_t prev_hall, uint8_t next_hall) {
  const int8_t prev_idx = hallIndex(prev_hall);
  const int8_t next_idx = hallIndex(next_hall);
  g_closedloop_last_hall_idx = next_idx;

  if (!isValidHallState(next_hall) || prev_idx < 0 || next_idx < 0) {
    ++g_closedloop_hall_invalid_count;
    g_closedloop_hall_delta = 0;
    return;
  }

  const int8_t forward_delta = static_cast<int8_t>((next_idx - prev_idx + 6) % 6);
  if (forward_delta == 1) {
    ++g_closedloop_hall_forward_count;
    g_closedloop_hall_delta = 1;
  } else if (forward_delta == 5) {
    ++g_closedloop_hall_reverse_count;
    g_closedloop_hall_delta = -1;
  } else {
    ++g_closedloop_hall_jump_count;
    g_closedloop_hall_delta = forward_delta;
  }
}

void advanceOpenloopAngleStep() {
  g_openloop_debug_angle_el += config::kSimpleFocOpenloopDebugStepDeg * DEG_TO_RAD;
  if (g_openloop_debug_angle_el >= _2PI) {
    g_openloop_debug_angle_el -= _2PI;
  }
}

uint32_t openloopStepIntervalMsForRpm(float rpm) {
  const float clamped_rpm = max(1.0f, rpm);
  const float step_rad = config::kSimpleFocOpenloopDebugStepDeg * DEG_TO_RAD;
  const float electrical_rad_per_sec =
      rpmToRadPerSec(clamped_rpm) * static_cast<float>(config::kConfiguredPolePairs);
  const float interval_ms = (step_rad / electrical_rad_per_sec) * 1000.0f;
  return static_cast<uint32_t>(max(20.0f, min(200.0f, interval_ms)));
}

bool faultActive() {
  return digitalRead(config::kDrvFaultPin) == LOW;
}

void doHallA() {
  ++hall_a_irq_count;
  g_hall_sensor.handleA();
}

void doHallB() {
  ++hall_b_irq_count;
  g_hall_sensor.handleB();
}

void doHallC() {
  ++hall_c_irq_count;
  g_hall_sensor.handleC();
}

uint16_t buildDrvControl1(bool gate_reset) {
  uint16_t value = 0;
  value |= static_cast<uint16_t>(config::kDrvOcpAdjustCode & 0x1Fu);
  value |= static_cast<uint16_t>((config::kDrvOcpModeCode & 0x03u) << kCtrl1OcpModeShift);
  if (!isClosedloop6PwmMode() && config::kDrvUse3PwmMode) {
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

void applyPatternRaw(const StepPattern& pattern) {
  analogWrite(config::kPwmAPin, pattern.pwm_a);
  analogWrite(config::kPwmBPin, pattern.pwm_b);
  analogWrite(config::kPwmCPin, pattern.pwm_c);
  analogWrite(config::kPwmALowPin, 0);
  analogWrite(config::kPwmBLowPin, 0);
  analogWrite(config::kPwmCLowPin, 0);
}

void pwmOff() {
  analogWrite(config::kPwmAPin, 0);
  analogWrite(config::kPwmBPin, 0);
  analogWrite(config::kPwmCPin, 0);
  analogWrite(config::kPwmALowPin, 0);
  analogWrite(config::kPwmBLowPin, 0);
  analogWrite(config::kPwmCLowPin, 0);
}

void drvDisable() {
  pwmOff();
  digitalWrite(config::kDrvEnablePin, LOW);
  g_driver_enabled = false;
  g_drv_configured = false;
}

bool drvStatus2GvddOv() {
  return (g_drv.status2 & 0x80u) != 0u;
}

uint8_t drvStatus2DeviceId() {
  return static_cast<uint8_t>(g_drv.status2 & 0x0Fu);
}

bool drvReadyForPwm() {
  return g_driver_enabled && g_drv_configured && g_drv.control_match && !g_drv.frame_fault &&
         !faultActive() && g_drv.status1 == 0u && !drvStatus2GvddOv();
}

bool applyPattern(const StepPattern& pattern) {
  if (!drvReadyForPwm()) {
    pwmOff();
    return false;
  }
  applyPatternRaw(pattern);
  return true;
}

void applyStepIndexWithDuty(uint8_t step_index, uint8_t duty) {
  g_step_index = static_cast<uint8_t>(step_index % 6u);
  const StepPattern& base = activePattern();
  g_sixstep_current_duty = duty;
  if (!drvReadyForPwm()) {
    pwmOff();
    return;
  }
  analogWrite(config::kPwmAPin, base.pwm_a ? duty : 0);
  analogWrite(config::kPwmBPin, base.pwm_b ? duty : 0);
  analogWrite(config::kPwmCPin, base.pwm_c ? duty : 0);
  analogWrite(config::kPwmALowPin, 0);
  analogWrite(config::kPwmBLowPin, 0);
  analogWrite(config::kPwmCLowPin, 0);
}

void applySinglePhase(uint8_t phase_index) {
  if (!drvReadyForPwm()) {
    pwmOff();
    return;
  }
  analogWrite(config::kPwmAPin, phase_index == 0 ? config::kPhaseCheckPwmDuty : 0);
  analogWrite(config::kPwmBPin, phase_index == 1 ? config::kPhaseCheckPwmDuty : 0);
  analogWrite(config::kPwmCPin, phase_index == 2 ? config::kPhaseCheckPwmDuty : 0);
  analogWrite(config::kPwmALowPin, 0);
  analogWrite(config::kPwmBLowPin, 0);
  analogWrite(config::kPwmCLowPin, 0);
}

void applySinglePhaseStability(uint8_t phase_index) {
  if (!drvReadyForPwm()) {
    pwmOff();
    return;
  }
  analogWrite(config::kPwmAPin, phase_index == 0 ? config::kDrvSpiStabilityPwmDuty : 0);
  analogWrite(config::kPwmBPin, phase_index == 1 ? config::kDrvSpiStabilityPwmDuty : 0);
  analogWrite(config::kPwmCPin, phase_index == 2 ? config::kDrvSpiStabilityPwmDuty : 0);
  analogWrite(config::kPwmALowPin, 0);
  analogWrite(config::kPwmBLowPin, 0);
  analogWrite(config::kPwmCLowPin, 0);
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
  if (!g_driver_enabled) {
    g_drv.spi_ok = false;
    g_drv.frame_fault = false;
    g_drv.control_match = false;
    return false;
  }
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
  if (!g_driver_enabled) {
    g_drv_configured = false;
    return false;
  }
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
  g_drv_configured = drvReadAll();
  return g_drv_configured;
}

bool drvEnableAndConfigure(Stream& serial) {
  pwmOff();
  digitalWrite(config::kDrvEnablePin, HIGH);
  delay(50);
  g_driver_enabled = true;
  g_drv_configured = false;

  if (!drvConfigure3Pwm()) {
    printState(serial, "drv_config_failed");
    pwmOff();
    g_driver_enabled = false;
    g_drv_configured = false;
    return false;
  }

  (void)drvReadAll();
  if (!faultActive() && !g_drv.frame_fault && g_drv.control_match && g_drv.status1 == 0u &&
      !drvStatus2GvddOv()) {
    g_driver_enabled = true;
    g_drv_configured = true;
    return true;
  }

  printState(serial, "drv_config_invalid");
  pwmOff();
  g_driver_enabled = false;
  g_drv_configured = false;
  return false;
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
    pwmOff();
    return false;
  }

  applyStepIndexWithDuty(commutationIndexFromHall(hall), config::kSixStepPwmDuty);
  return true;
}

void printDrvRegisters(Stream& serial) {
  serial.print(" spi_ok=");
  serial.print(g_drv.spi_ok ? "yes" : "no");
  serial.print(" frame_fault=");
  serial.print(g_drv.frame_fault ? "yes" : "no");
  serial.print(" status1=0x");
  serial.print(g_drv.status1, HEX);
  serial.print(" status2_raw=0x");
  serial.print(g_drv.status2, HEX);
  serial.print(" status2_device_id=0x");
  serial.print(drvStatus2DeviceId(), HEX);
  serial.print(" status2_gvdd_ov=");
  serial.print(drvStatus2GvddOv() ? "yes" : "no");
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

  if (config::kTestMode == config::TEST_SIMPLEFOC_CLOSEDLOOP ||
      config::kTestMode == config::TEST_SIMPLEFOC_CLOSEDLOOP_6PWM) {
    const uint8_t hall = readHallState();
    const float measured_rad_s = closedloopMeasuredRadS();
    const float target_rad_s =
        g_closedloop_startup_active ? g_closedloop_startup_target_rad_s : closedloopTargetRadS();
    const float sensor_angle = g_hall_sensor.getAngle();
    const float sensor_velocity = g_hall_sensor.getVelocity();
    const float motor_shaft_angle = g_motor.shaft_angle;
    const float motor_shaft_velocity = g_motor.shaft_velocity;
    const uint32_t irq_a = hall_a_irq_count;
    const uint32_t irq_b = hall_b_irq_count;
    const uint32_t irq_c = hall_c_irq_count;
    serial.print("motor_test reason=");
    serial.print(reason);
    serial.print(" en=");
    serial.print(g_driver_enabled ? "HIGH" : "LOW");
    serial.print(" fault=");
    serial.print(faultActive() ? "ACTIVE" : "inactive");
    serial.print(" mode=");
    serial.print(g_closedloop_startup_active ? "startup" : "closed");
    serial.print(" pwm_mode=");
    serial.print(isClosedloop6PwmMode() ? "6PWM" : "3PWM");
    serial.print(" target_rpm=");
    serial.print(config::kSimpleFocClosedloopTargetRpm, 1);
    serial.print(" target_rad_s=");
    serial.print(target_rad_s, 3);
    serial.print(" measured_rad_s=");
    serial.print(measured_rad_s, 3);
    serial.print(" velocity_error=");
    serial.print(target_rad_s - measured_rad_s, 3);
    serial.print(" startup_target_rad_s=");
    serial.print(g_closedloop_startup_target_rad_s, 3);
    serial.print(" measured_rad_s_at_handoff=");
    serial.print(g_closedloop_measured_rad_s_at_handoff, 3);
    serial.print(" handoff_allowed=");
    serial.print(g_closedloop_handoff_allowed ? "yes" : "no");
    serial.print(" voltage_limit=");
    serial.print(g_motor.voltage_limit, 2);
    serial.print(" pid_p=");
    serial.print(g_motor.PID_velocity.P, 4);
    serial.print(" pid_i=");
    serial.print(g_motor.PID_velocity.I, 4);
    serial.print(" pid_d=");
    serial.print(g_motor.PID_velocity.D, 4);
    serial.print(" lpf_tf=");
    serial.print(g_motor.LPF_velocity.Tf, 3);
    serial.print(" closedloop_loopfoc_called=");
    serial.print(g_closedloop_loopfoc_called ? "yes" : "no");
    serial.print(" closedloop_move_called=");
    serial.print(g_closedloop_move_called ? "yes" : "no");
    serial.print(" initFOC_start=");
    serial.print(g_closedloop_initfoc_started ? "yes" : "no");
    serial.print(" initFOC_done=");
    serial.print(g_closedloop_initfoc_done ? "yes" : "no");
    serial.print(" shaft_vel=");
    serial.print(measured_rad_s, 3);
    serial.print(" shaft_angle=");
    serial.print(sensor_angle, 3);
    serial.print(" raw_hall=");
    printHallBits(serial, hall);
    serial.print(" raw_hall_idx=");
    serial.print(hallIndex(hall));
    serial.print(" valid=");
    serial.print(isValidHallState(hall) ? "yes" : "no");
    serial.print(" hall_irq_a=");
    serial.print(irq_a);
    serial.print(" hall_irq_b=");
    serial.print(irq_b);
    serial.print(" hall_irq_c=");
    serial.print(irq_c);
    serial.print(" hall_irq_a_delta=");
    serial.print(g_hall_a_irq_delta);
    serial.print(" hall_irq_b_delta=");
    serial.print(g_hall_b_irq_delta);
    serial.print(" hall_irq_c_delta=");
    serial.print(g_hall_c_irq_delta);
    serial.print(" hall_irq_counts_increasing=");
    serial.print(g_hall_irq_counts_increasing ? "yes" : "no");
    serial.print(" sensor_angle=");
    serial.print(sensor_angle, 3);
    serial.print(" sensor_velocity=");
    serial.print(sensor_velocity, 3);
    serial.print(" sensor_angle_changes=");
    serial.print(g_sensor_angle_changes ? "yes" : "no");
    serial.print(" sensor_velocity_valid=");
    serial.print(g_sensor_velocity_valid ? "yes" : "no");
    serial.print(" motor_shaft_angle=");
    serial.print(motor_shaft_angle, 3);
    serial.print(" motor_shaft_velocity=");
    serial.print(motor_shaft_velocity, 3);
    serial.print(" drv_configured=");
    serial.print(g_drv_configured ? "yes" : "no");
    serial.print(" hall_fwd=");
    serial.print(g_closedloop_hall_forward_count);
    serial.print(" hall_rev=");
    serial.print(g_closedloop_hall_reverse_count);
    serial.print(" hall_jump=");
    serial.print(g_closedloop_hall_jump_count);
    serial.print(" hall_invalid=");
    serial.print(g_closedloop_hall_invalid_count);
    serial.print(" last_hall=");
    serial.print(g_closedloop_last_hall_idx);
    serial.print(" hall_delta=");
    serial.print(g_closedloop_hall_delta);
    printDrvRegisters(serial);
    serial.println();
    return;
  }

  if (config::kTestMode == config::TEST_SIMPLEFOC_OPENLOOP_DEBUG) {
    const uint8_t hall = readHallState();
    serial.print("motor_test reason=");
    serial.print(reason);
    serial.print(" en=");
    serial.print(g_driver_enabled ? "HIGH" : "LOW");
    serial.print(" fault=");
    serial.print(faultActive() ? "ACTIVE" : "inactive");
    serial.print(" dbg_voltage=");
    serial.print(g_openloop_debug_voltage, 2);
    serial.print(" angle_el=");
    serial.print(g_openloop_debug_angle_el, 3);
    serial.print(" step_deg=");
    serial.print(config::kSimpleFocOpenloopDebugStepDeg, 1);
    serial.print(" step_ms=");
    serial.print(config::kSimpleFocOpenloopDebugStepMs);
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
    serial.print(" target_rad_s=");
    serial.print(g_openloop_target_rad_s, 3);
    serial.print(" openloop_move_called=");
    serial.print(g_openloop_move_called ? "yes" : "no");
    serial.print(" motor.voltage_limit=");
    serial.print(g_motor.voltage_limit, 2);
    serial.print(" driver.voltage_limit=");
    serial.print(g_driver3.voltage_limit, 2);
    serial.print(" drv_configured=");
    serial.print(g_drv_configured ? "yes" : "no");
    serial.print(" hall=");
    printHallBits(serial, hall);
    serial.print(" hall_idx=");
    serial.print(hallIndex(hall));
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
  serial.print(" mode=");
  serial.print(g_sixstep_startup_active ? "startup_kick" : "hall_track");
  serial.print(" hall_idx=");
  serial.print(hallIndex(hall));
  serial.print(" pwm=[");
  serial.print(pattern.pwm_a ? g_sixstep_current_duty : 0);
  serial.print(",");
  serial.print(pattern.pwm_b ? g_sixstep_current_duty : 0);
  serial.print(",");
  serial.print(pattern.pwm_c ? g_sixstep_current_duty : 0);
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
  g_driver_enabled = false;
  g_openloop_started = false;
  g_boot_ms = millis();
  g_last_print_ms = 0;
  g_openloop_enable_ms = 0;
  g_openloop_target_rad_s = 0.0f;
  g_openloop_move_called = false;
  g_drv_configured = false;
  g_drv = {};
  pwmOff();
  const bool spi_ok = drvSpiInit();

  g_driver3.pwm_frequency = 20000;
  g_driver3.voltage_power_supply = config::kBusVoltage;
  g_driver3.voltage_limit = config::kSimpleFocOpenloopVoltageLimit;
  g_driver3.init();

  g_motor.linkDriver(&g_driver3);
  g_motor.torque_controller = TorqueControlType::voltage;
  g_motor.controller = MotionControlType::velocity_openloop;
  g_motor.foc_modulation = FOCModulationType::Trapezoid_120;
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

void beginSimpleFocOpenloopDebug(Stream& serial) {
  pinMode(config::kDrvEnablePin, OUTPUT);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);
  digitalWrite(config::kDrvEnablePin, LOW);

  g_fault_latched = false;
  g_driver_enabled = false;
  g_openloop_debug_started = false;
  g_boot_ms = millis();
  g_last_print_ms = 0;
  g_openloop_debug_step_ms = 0;
  g_openloop_debug_angle_el = 0.0f;
  g_openloop_debug_voltage = config::kSimpleFocOpenloopDebugVoltage;
  g_drv = {};
  pwmOff();
  const bool spi_ok = drvSpiInit();

  g_driver3.pwm_frequency = 20000;
  g_driver3.voltage_power_supply = config::kBusVoltage;
  g_driver3.voltage_limit = config::kSimpleFocOpenloopDebugVoltage;
  g_driver3.init();

  g_motor.linkDriver(&g_driver3);
  g_motor.torque_controller = TorqueControlType::voltage;
  g_motor.controller = MotionControlType::angle_openloop;
  g_motor.foc_modulation = FOCModulationType::Trapezoid_120;
  g_motor.voltage_limit = config::kSimpleFocOpenloopDebugVoltage;
  g_motor.init();

  serial.println("SimpleFOC 3PWM open-loop direct debug");
  serial.println("Driver starts disabled, then direct electrical angle stepping is applied.");
  serial.print("Enable delay ms=");
  serial.print(config::kSimpleFocEnableDelayMs);
  serial.print(" debug_voltage=");
  serial.print(config::kSimpleFocOpenloopDebugVoltage, 2);
  serial.print(" step_deg=");
  serial.print(config::kSimpleFocOpenloopDebugStepDeg, 1);
  serial.print(" step_ms=");
  serial.println(config::kSimpleFocOpenloopDebugStepMs);
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
  g_driver_enabled = false;
  g_drv_configured = false;
  g_closedloop_started = false;
  g_closedloop_startup_active = false;
  g_closedloop_loopfoc_called = false;
  g_closedloop_move_called = false;
  g_closedloop_initfoc_started = false;
  g_closedloop_initfoc_done = false;
  g_closedloop_handoff_allowed = false;
  g_closedloop_handoff_evaluated = false;
  g_boot_ms = millis();
  g_closedloop_enable_ms = 0;
  g_last_print_ms = 0;
  g_last_hall_state = readHallState();
  g_closedloop_startup_target_rad_s = 0.0f;
  g_closedloop_measured_rad_s_at_handoff = 0.0f;
  g_closedloop_hall_forward_count = 0;
  g_closedloop_hall_reverse_count = 0;
  g_closedloop_hall_jump_count = 0;
  g_closedloop_hall_invalid_count = 0;
  g_closedloop_last_hall_idx = hallIndex(g_last_hall_state);
  g_closedloop_hall_delta = 0;
  hall_a_irq_count = 0;
  hall_b_irq_count = 0;
  hall_c_irq_count = 0;
  g_last_hall_a_irq_count = 0;
  g_last_hall_b_irq_count = 0;
  g_last_hall_c_irq_count = 0;
  g_hall_a_irq_delta = 0;
  g_hall_b_irq_delta = 0;
  g_hall_c_irq_delta = 0;
  g_last_sensor_angle = 0.0f;
  g_last_sensor_velocity = 0.0f;
  g_sensor_angle_changes = false;
  g_sensor_velocity_valid = false;
  g_hall_irq_counts_increasing = false;
  g_drv = {};
  pwmOff();

  const bool spi_ok = drvSpiInit();

  g_hall_sensor.pullup = Pullup::USE_INTERN;
  g_hall_sensor.init();
  g_hall_sensor.enableInterrupts(doHallA, doHallB, doHallC);

  g_driver3.pwm_frequency = 20000;
  g_driver3.voltage_power_supply = config::kBusVoltage;
  g_driver3.voltage_limit = config::kSimpleFocClosedloopVoltageLimit;
  g_driver3.init();

  g_motor.linkDriver(&g_driver3);
  g_motor.linkSensor(&g_hall_sensor);
  g_motor.controller = MotionControlType::velocity;
  g_motor.torque_controller = TorqueControlType::voltage;
  g_motor.voltage_limit = config::kSimpleFocClosedloopVoltageLimit;
  g_motor.velocity_limit = rpmToRadPerSec(config::kSimpleFocClosedloopVelocityLimit);
  g_motor.PID_velocity.P = config::kSimpleFocClosedloopVelP;
  g_motor.PID_velocity.I = config::kSimpleFocClosedloopVelI;
  g_motor.PID_velocity.D = config::kSimpleFocClosedloopVelD;
  g_motor.LPF_velocity.Tf = config::kSimpleFocClosedloopVelLpfTf;
  g_motor.voltage_sensor_align = 0.5f;
  g_motor.init();

  // új rész 
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

void beginSimpleFocClosedloop6Pwm(Stream& serial) {
  pinMode(config::kDrvEnablePin, OUTPUT);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);
  pinMode(config::kPwmAPin, OUTPUT);
  pinMode(config::kPwmBPin, OUTPUT);
  pinMode(config::kPwmCPin, OUTPUT);
  pinMode(config::kPwmALowPin, OUTPUT);
  pinMode(config::kPwmBLowPin, OUTPUT);
  pinMode(config::kPwmCLowPin, OUTPUT);
  digitalWrite(config::kDrvEnablePin, LOW);

  g_fault_latched = false;
  g_driver_enabled = false;
  g_drv_configured = false;
  g_closedloop_started = false;
  g_closedloop_startup_active = false;
  g_closedloop_loopfoc_called = false;
  g_closedloop_move_called = false;
  g_closedloop_initfoc_started = false;
  g_closedloop_initfoc_done = false;
  g_closedloop_handoff_allowed = false;
  g_closedloop_handoff_evaluated = false;
  g_boot_ms = millis();
  g_closedloop_enable_ms = 0;
  g_last_print_ms = 0;
  g_last_hall_state = readHallState();
  g_closedloop_startup_target_rad_s = 0.0f;
  g_closedloop_measured_rad_s_at_handoff = 0.0f;
  g_closedloop_hall_forward_count = 0;
  g_closedloop_hall_reverse_count = 0;
  g_closedloop_hall_jump_count = 0;
  g_closedloop_hall_invalid_count = 0;
  g_closedloop_last_hall_idx = hallIndex(g_last_hall_state);
  g_closedloop_hall_delta = 0;
  hall_a_irq_count = 0;
  hall_b_irq_count = 0;
  hall_c_irq_count = 0;
  g_last_hall_a_irq_count = 0;
  g_last_hall_b_irq_count = 0;
  g_last_hall_c_irq_count = 0;
  g_hall_a_irq_delta = 0;
  g_hall_b_irq_delta = 0;
  g_hall_c_irq_delta = 0;
  g_last_sensor_angle = 0.0f;
  g_last_sensor_velocity = 0.0f;
  g_sensor_angle_changes = false;
  g_sensor_velocity_valid = false;
  g_hall_irq_counts_increasing = false;
  g_drv = {};
  pwmOff();

  const bool spi_ok = drvSpiInit();

  g_hall_sensor.pullup = Pullup::USE_INTERN;
  g_hall_sensor.init();
  g_hall_sensor.enableInterrupts(doHallA, doHallB, doHallC);

  g_driver6.pwm_frequency = 20000;
  g_driver6.voltage_power_supply = config::kBusVoltage;
  g_driver6.voltage_limit = config::kSimpleFocClosedloopVoltageLimit;
  g_driver6.init();

  g_motor.linkDriver(&g_driver6);
  g_motor.linkSensor(&g_hall_sensor);
  g_motor.controller = MotionControlType::velocity;
  g_motor.torque_controller = TorqueControlType::voltage;
  g_motor.voltage_limit = config::kSimpleFocClosedloopVoltageLimit;
  g_motor.velocity_limit = rpmToRadPerSec(config::kSimpleFocClosedloopVelocityLimit);
  g_motor.PID_velocity.P = config::kSimpleFocClosedloopVelP;
  g_motor.PID_velocity.I = config::kSimpleFocClosedloopVelI;
  g_motor.PID_velocity.D = config::kSimpleFocClosedloopVelD;
  g_motor.LPF_velocity.Tf = config::kSimpleFocClosedloopVelLpfTf;
  g_motor.voltage_sensor_align = 0.5f;
  g_motor.init();

  serial.println("SimpleFOC 6PWM closed-loop hall test");
  serial.println("Driver starts disabled, then DRV8301 is configured over SPI for 6PWM.");
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
  drvDisable();

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
  drvDisable();

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
      pwmOff();
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
      g_fault_latched = true;
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    (void)drvReadAll();
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kPhaseCheckEnableDelayMs) {
    if (drvEnableAndConfigure(serial)) {
      applySinglePhase(g_step_index % 3u);
      g_last_phase_change_ms = now;
    }
    printState(serial, drvReadyForPwm() ? "enable_applied" : "enable_failed");
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
      pwmOff();
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
      g_fault_latched = true;
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    (void)drvReadAll();
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kDrvSpiStabilityEnableDelayMs) {
    if (drvEnableAndConfigure(serial)) {
      g_last_phase_change_ms = now;
    }
    printState(serial, drvReadyForPwm() ? "enable_applied" : "enable_failed");
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
      g_driver3.disable();
      pwmOff();
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
      g_openloop_started = false;
      g_openloop_enable_ms = 0;
      g_openloop_target_rad_s = 0.0f;
      g_openloop_move_called = false;
      g_fault_latched = true;
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    g_openloop_move_called = false;
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kSimpleFocEnableDelayMs) {
    if (drvEnableAndConfigure(serial)) {
      g_driver3.enable();
      g_motor.enable();
      g_openloop_started = true;
      g_openloop_enable_ms = now;
      g_last_hall_state = hall;
    } else {
      g_openloop_started = false;
    }
    printState(serial, g_openloop_started ? "enable_applied" : "enable_failed");
  }

  g_openloop_target_rad_s = rpmToRadPerSec(config::kSimpleFocOpenloopTargetRpm);
  g_openloop_move_called = false;

  if (drvReadyForPwm() && g_openloop_started) {
    g_motor.voltage_limit = config::kSimpleFocOpenloopVoltageLimit;
    g_motor.move(g_openloop_target_rad_s);
    g_openloop_move_called = true;
  } else {
    pwmOff();
  }

  if ((now - g_last_print_ms) >= config::kSimpleFocStatusIntervalMs) {
    const uint32_t irq_a = hall_a_irq_count;
    const uint32_t irq_b = hall_b_irq_count;
    const uint32_t irq_c = hall_c_irq_count;
    const float sensor_angle = g_hall_sensor.getAngle();
    const float sensor_velocity = g_hall_sensor.getVelocity();
    g_hall_irq_counts_increasing =
        (irq_a > g_last_hall_a_irq_count) || (irq_b > g_last_hall_b_irq_count) ||
        (irq_c > g_last_hall_c_irq_count);
    g_sensor_angle_changes = fabsf(sensor_angle - g_last_sensor_angle) > 0.0001f;
    g_sensor_velocity_valid = isfinite(sensor_velocity) && fabsf(sensor_velocity) < 10000.0f;
    g_last_hall_a_irq_count = irq_a;
    g_last_hall_b_irq_count = irq_b;
    g_last_hall_c_irq_count = irq_c;
    g_last_sensor_angle = sensor_angle;
    g_last_sensor_velocity = sensor_velocity;
    g_last_print_ms = now;
    if (g_driver_enabled) {
      (void)drvReadAll();
    }
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
    serial.print(" target_rad_s=");
    serial.print(g_openloop_target_rad_s, 3);
    serial.print(" openloop_move_called=");
    serial.print(g_openloop_move_called ? "yes" : "no");
    serial.print(" drv_configured=");
    serial.print(g_drv_configured ? "yes" : "no");
    printDrvRegisters(serial);
    serial.println();
    g_last_hall_state = hall;
  }
}

void updateSimpleFocOpenloopDebug(Stream& serial) {
  const uint32_t now = millis();
  const bool fault_active = faultActive();
  const uint8_t hall = readHallState();

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      g_motor.disable();
      g_driver3.disable();
      pwmOff();
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
      g_openloop_debug_started = false;
      g_fault_latched = true;
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kSimpleFocEnableDelayMs) {
    if (drvEnableAndConfigure(serial)) {
      g_driver3.enable();
      g_motor.enable();
      g_openloop_debug_started = true;
      g_openloop_debug_step_ms = now;
      g_openloop_debug_angle_el = 0.0f;
      g_openloop_debug_voltage = config::kSimpleFocOpenloopDebugVoltage;
      g_last_hall_state = hall;
      if (drvReadyForPwm()) {
        g_motor.setPhaseVoltage(g_openloop_debug_voltage, 0.0f, g_openloop_debug_angle_el);
      }
    }
    printState(serial, g_openloop_debug_started ? "enable_applied" : "enable_failed");
  }

  if (g_openloop_debug_started && (now - g_openloop_debug_step_ms) >= config::kSimpleFocOpenloopDebugStepMs) {
    g_openloop_debug_step_ms = now;
    advanceOpenloopAngleStep();
    if (drvReadyForPwm()) {
      g_motor.setPhaseVoltage(g_openloop_debug_voltage, 0.0f, g_openloop_debug_angle_el);
    } else {
      pwmOff();
    }
    printState(serial, "angle_step");
  }

  if ((now - g_last_print_ms) >= config::kSimpleFocStatusIntervalMs) {
    g_last_print_ms = now;
    if (g_driver_enabled) {
      (void)drvReadAll();
    }
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
    serial.print(" angle_el=");
    serial.print(g_openloop_debug_angle_el, 3);
    printDrvRegisters(serial);
    serial.println();
    g_last_hall_state = hall;
  }
}

void updateSimpleFocClosedloop(Stream& serial) {
  const uint32_t now = millis();
  const bool fault_active = faultActive();
  const uint8_t hall = readHallState();
  const float target_sign = config::kSimpleFocClosedloopReverse ? -1.0f : 1.0f;
  const float closed_target_rad_s = closedloopTargetRadS();
  g_closedloop_loopfoc_called = false;
  g_closedloop_move_called = false;

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      g_motor.disable();
      g_driver3.disable();
      pwmOff();
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
      g_closedloop_started = false;
      g_closedloop_startup_active = false;
      g_closedloop_initfoc_done = false;
      g_fault_latched = true;
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kSimpleFocEnableDelayMs) {
    if (drvEnableAndConfigure(serial)) {
      g_driver3.enable();
      g_motor.enable();
      g_closedloop_enable_ms = now;
      g_last_hall_state = hall;
      g_closedloop_last_hall_idx = hallIndex(hall);
      g_closedloop_hall_delta = 0;
      g_closedloop_initfoc_started = true;
      if (drvReadyForPwm()) {
        printState(serial, "initFOC_start");
        g_motor.initFOC();
        g_closedloop_initfoc_done = true;
        printState(serial, "initFOC_done");
        if (drvReadyForPwm()) {
          g_closedloop_started = true;
          g_closedloop_startup_active = config::kSimpleFocClosedloopUseStartupAssist;
          g_closedloop_handoff_allowed = false;
          g_closedloop_handoff_evaluated = false;
          g_closedloop_startup_target_rad_s =
              rpmToRadPerSec(config::kSimpleFocClosedloopStartupRpm) * target_sign;
          g_closedloop_measured_rad_s_at_handoff = 0.0f;
          g_motor.controller = g_closedloop_startup_active ? MotionControlType::velocity_openloop
                                                           : MotionControlType::velocity;
        } else {
          pwmOff();
          g_closedloop_started = false;
        }
      } else {
        pwmOff();
        g_closedloop_started = false;
      }
    }
    printState(serial, g_closedloop_started ? "enable_applied" : "enable_failed");
  }

  if (!g_closedloop_started) {
    if ((now - g_last_print_ms) >= config::kSimpleFocStatusIntervalMs) {
      g_last_print_ms = now;
      if (g_driver_enabled) {
        (void)drvReadAll();
      }
      printState(serial, "periodic");
    }
    return;
  }

  if (!g_drv_configured || !g_drv.control_match || g_drv.frame_fault || faultActive()) {
    pwmOff();
    if (g_driver_enabled) {
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
    }
    g_closedloop_started = false;
    g_closedloop_startup_active = false;
    return;
  }

  if (hall != g_last_hall_state) {
    recordClosedloopHallTransition(g_last_hall_state, hall);
    g_last_hall_state = hall;
  }

  if (g_closedloop_startup_active) {
    const uint32_t elapsed_ms = now - g_closedloop_enable_ms;
    const float measured_rad_s = closedloopMeasuredRadS();
    const float handoff_limit = fabsf(closed_target_rad_s) * 1.5f;
    if (elapsed_ms >= config::kSimpleFocClosedloopStartupMs) {
      const bool prev_handoff_allowed = g_closedloop_handoff_allowed;
      g_closedloop_measured_rad_s_at_handoff = measured_rad_s;
      g_closedloop_handoff_allowed = fabsf(measured_rad_s) <= handoff_limit;
      const bool should_log_handoff =
          !g_closedloop_handoff_evaluated || prev_handoff_allowed != g_closedloop_handoff_allowed;
      g_closedloop_handoff_evaluated = true;
      if (g_closedloop_handoff_allowed) {
        g_closedloop_startup_active = false;
        g_motor.controller = MotionControlType::velocity;
      }
      if (should_log_handoff) {
        printState(serial, "startup_handoff");
      }
    }

    if (g_closedloop_startup_active) {
      float startup_target_rad_s = g_closedloop_startup_target_rad_s;
      if (fabsf(measured_rad_s) > handoff_limit) {
        startup_target_rad_s *= 0.5f;
      }
      g_motor.move(startup_target_rad_s);
      g_closedloop_move_called = true;
    }
  }

  if (!g_closedloop_startup_active) {
    if (!drvReadyForPwm()) {
      pwmOff();
      return;
    }
    g_motor.loopFOC();
    g_closedloop_loopfoc_called = true;
    g_motor.move(closed_target_rad_s);
    g_closedloop_move_called = true;
  }

  if (!drvReadyForPwm()) {
    pwmOff();
  }

  if ((now - g_last_print_ms) >= config::kSimpleFocStatusIntervalMs) {
    noInterrupts();
    const uint32_t irq_a = hall_a_irq_count;
    const uint32_t irq_b = hall_b_irq_count;
    const uint32_t irq_c = hall_c_irq_count;
    interrupts();
    const float sensor_angle = g_hall_sensor.getAngle();
    const float sensor_velocity = g_hall_sensor.getVelocity();
    g_hall_a_irq_delta = irq_a - g_last_hall_a_irq_count;
    g_hall_b_irq_delta = irq_b - g_last_hall_b_irq_count;
    g_hall_c_irq_delta = irq_c - g_last_hall_c_irq_count;
    g_hall_irq_counts_increasing =
        (g_hall_a_irq_delta > 0u) || (g_hall_b_irq_delta > 0u) || (g_hall_c_irq_delta > 0u);
    g_sensor_angle_changes = fabsf(sensor_angle - g_last_sensor_angle) > 0.0001f;
    g_sensor_velocity_valid = isfinite(sensor_velocity) && fabsf(sensor_velocity) > 0.001f;
    g_last_hall_a_irq_count = irq_a;
    g_last_hall_b_irq_count = irq_b;
    g_last_hall_c_irq_count = irq_c;
    g_last_sensor_angle = sensor_angle;
    g_last_sensor_velocity = sensor_velocity;
    g_last_print_ms = now;
    if (g_driver_enabled) {
      (void)drvReadAll();
    }
    printState(serial, "periodic");
    g_closedloop_hall_forward_count = 0;
    g_closedloop_hall_reverse_count = 0;
    g_closedloop_hall_jump_count = 0;
    g_closedloop_hall_invalid_count = 0;
  }
}

void updateSimpleFocClosedloop6Pwm(Stream& serial) {
  const uint32_t now = millis();
  const bool fault_active = faultActive();
  const uint8_t hall = readHallState();
  const float target_sign = config::kSimpleFocClosedloopReverse ? -1.0f : 1.0f;
  const float closed_target_rad_s = closedloopTargetRadS();
  g_closedloop_loopfoc_called = false;
  g_closedloop_move_called = false;

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      g_motor.disable();
      g_driver6.disable();
      pwmOff();
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
      g_closedloop_started = false;
      g_closedloop_startup_active = false;
      g_closedloop_initfoc_done = false;
      g_fault_latched = true;
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kSimpleFocEnableDelayMs) {
    if (drvEnableAndConfigure(serial)) {
      g_driver6.enable();
      g_motor.enable();
      g_closedloop_enable_ms = now;
      g_last_hall_state = hall;
      g_closedloop_last_hall_idx = hallIndex(hall);
      g_closedloop_hall_delta = 0;
      g_closedloop_initfoc_started = true;
      if (drvReadyForPwm()) {
        printState(serial, "initFOC_start");
        g_motor.initFOC();
        g_closedloop_initfoc_done = true;
        printState(serial, "initFOC_done");
        if (drvReadyForPwm()) {
          g_closedloop_started = true;
          g_closedloop_startup_active = config::kSimpleFocClosedloopUseStartupAssist;
          g_closedloop_handoff_allowed = false;
          g_closedloop_handoff_evaluated = false;
          g_closedloop_startup_target_rad_s =
              rpmToRadPerSec(config::kSimpleFocClosedloopStartupRpm) * target_sign;
          g_closedloop_measured_rad_s_at_handoff = 0.0f;
          g_motor.controller = g_closedloop_startup_active ? MotionControlType::velocity_openloop
                                                           : MotionControlType::velocity;
        } else {
          pwmOff();
          g_closedloop_started = false;
        }
      } else {
        pwmOff();
        g_closedloop_started = false;
      }
    }
    printState(serial, g_closedloop_started ? "enable_applied" : "enable_failed");
  }

  if (!g_closedloop_started) {
    if ((now - g_last_print_ms) >= config::kSimpleFocStatusIntervalMs) {
      g_last_print_ms = now;
      if (g_driver_enabled) {
        (void)drvReadAll();
      }
      printState(serial, "periodic");
    }
    return;
  }

  if (!g_drv_configured || !g_drv.control_match || g_drv.frame_fault || faultActive()) {
    pwmOff();
    if (g_driver_enabled) {
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
    }
    g_closedloop_started = false;
    g_closedloop_startup_active = false;
    return;
  }

  if (hall != g_last_hall_state) {
    recordClosedloopHallTransition(g_last_hall_state, hall);
    g_last_hall_state = hall;
  }

  if (g_closedloop_startup_active) {
    const uint32_t elapsed_ms = now - g_closedloop_enable_ms;
    const float measured_rad_s = closedloopMeasuredRadS();
    const float handoff_limit = fabsf(closed_target_rad_s) * 1.5f;
    if (elapsed_ms >= config::kSimpleFocClosedloopStartupMs) {
      const bool prev_handoff_allowed = g_closedloop_handoff_allowed;
      g_closedloop_measured_rad_s_at_handoff = measured_rad_s;
      g_closedloop_handoff_allowed = fabsf(measured_rad_s) <= handoff_limit;
      const bool should_log_handoff =
          !g_closedloop_handoff_evaluated || prev_handoff_allowed != g_closedloop_handoff_allowed;
      g_closedloop_handoff_evaluated = true;
      if (g_closedloop_handoff_allowed) {
        g_closedloop_startup_active = false;
        g_motor.controller = MotionControlType::velocity;
      }
      if (should_log_handoff) {
        printState(serial, "startup_handoff");
      }
    }

    if (g_closedloop_startup_active) {
      float startup_target_rad_s = g_closedloop_startup_target_rad_s;
      if (fabsf(measured_rad_s) > handoff_limit) {
        startup_target_rad_s *= 0.5f;
      }
      g_motor.move(startup_target_rad_s);
      g_closedloop_move_called = true;
    }
  }

  if (!g_closedloop_startup_active) {
    if (!drvReadyForPwm()) {
      pwmOff();
      return;
    }
    g_motor.loopFOC();
    g_closedloop_loopfoc_called = true;
    g_motor.move(closed_target_rad_s);
    g_closedloop_move_called = true;
  }

  if (!drvReadyForPwm()) {
    pwmOff();
  }

  if ((now - g_last_print_ms) >= config::kSimpleFocStatusIntervalMs) {
    noInterrupts();
    const uint32_t irq_a = hall_a_irq_count;
    const uint32_t irq_b = hall_b_irq_count;
    const uint32_t irq_c = hall_c_irq_count;
    interrupts();
    const float sensor_angle = g_hall_sensor.getAngle();
    const float sensor_velocity = g_hall_sensor.getVelocity();
    g_hall_a_irq_delta = irq_a - g_last_hall_a_irq_count;
    g_hall_b_irq_delta = irq_b - g_last_hall_b_irq_count;
    g_hall_c_irq_delta = irq_c - g_last_hall_c_irq_count;
    g_hall_irq_counts_increasing =
        (g_hall_a_irq_delta > 0u) || (g_hall_b_irq_delta > 0u) || (g_hall_c_irq_delta > 0u);
    g_sensor_angle_changes = fabsf(sensor_angle - g_last_sensor_angle) > 0.0001f;
    g_sensor_velocity_valid = isfinite(sensor_velocity) && fabsf(sensor_velocity) > 0.001f;
    g_last_hall_a_irq_count = irq_a;
    g_last_hall_b_irq_count = irq_b;
    g_last_hall_c_irq_count = irq_c;
    g_last_sensor_angle = sensor_angle;
    g_last_sensor_velocity = sensor_velocity;
    g_last_print_ms = now;
    if (g_driver_enabled) {
      (void)drvReadAll();
    }
    printState(serial, "periodic");
    g_closedloop_hall_forward_count = 0;
    g_closedloop_hall_reverse_count = 0;
    g_closedloop_hall_jump_count = 0;
    g_closedloop_hall_invalid_count = 0;
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

  if (config::kTestMode == config::TEST_SIMPLEFOC_CLOSEDLOOP_6PWM) {
    beginSimpleFocClosedloop6Pwm(serial);
    return;
  }

  if (config::kTestMode == config::TEST_SIMPLEFOC_OPENLOOP_DEBUG) {
    beginSimpleFocOpenloopDebug(serial);
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
  drvDisable();

  g_fault_latched = false;
  g_driver_enabled = false;
  g_drv_configured = false;
  g_sixstep_startup_active = false;
  g_step_index = 0;
  g_last_hall_state = readHallState();
  g_boot_ms = millis();
  g_last_print_ms = 0;
  g_sixstep_enable_ms = 0;
  g_sixstep_last_kick_ms = 0;
  g_sixstep_current_duty = config::kSixStepPwmDuty;
  g_drv = {};

  const bool spi_ok = drvSpiInit();
  if (spi_ok) {
    (void)drvReadAll();
  }

  serial.println("6-step hall commutation test");
  serial.println("Driver starts disabled and outputs are zero.");
  serial.print("Enable delay ms=");
  serial.print(config::kSixStepEnableDelayMs);
  serial.print(" pwm duty=");
  serial.print(config::kSixStepPwmDuty);
  serial.print(" startup duty=");
  serial.print(config::kSixStepStartupPwmDuty);
  serial.print(" startup step ms=");
  serial.print(config::kSixStepStartupStepMs);
  serial.print(" startup kick ms=");
  serial.print(config::kSixStepStartupKickMs);
  serial.print(" hall advance=");
  serial.print(config::kSixStepHallAdvance);
  serial.print(" reverse=");
  serial.println(config::kSixStepReverseDirection ? "yes" : "no");
  serial.print("SPI init=");
  serial.println(spi_ok ? "ok" : "failed");
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

  if (config::kTestMode == config::TEST_SIMPLEFOC_CLOSEDLOOP_6PWM) {
    updateSimpleFocClosedloop6Pwm(serial);
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

  if (config::kTestMode == config::TEST_SIMPLEFOC_OPENLOOP_DEBUG) {
    updateSimpleFocOpenloopDebug(serial);
    return;
  }

  const uint32_t now = millis();
  const bool fault_active = faultActive();

  if (fault_active) {
    if (g_driver_enabled || !g_fault_latched) {
      (void)drvReadAll();
      printState(serial, "fault_before_disable");
      drvDisable();
      g_fault_latched = true;
    }
    return;
  }

  if (g_fault_latched) {
    g_fault_latched = false;
    (void)drvReadAll();
    printState(serial, "fault_cleared");
  }

  if (!g_driver_enabled && (now - g_boot_ms) >= config::kSixStepEnableDelayMs) {
    const bool drv_ok = drvEnableAndConfigure(serial);
    g_sixstep_enable_ms = now;
    g_sixstep_last_kick_ms = now;
    g_sixstep_startup_active = drv_ok;
    const bool commutation_ok = drv_ok && applyCommutationForHall(readHallState());
    if (commutation_ok) {
      applyStepIndexWithDuty(g_step_index, config::kSixStepStartupPwmDuty);
    }
    printState(serial, commutation_ok ? "enable_applied" : "enable_failed");
  }

  const uint8_t hall = readHallState();
  if (g_driver_enabled && hall != g_last_hall_state) {
    g_last_hall_state = hall;
    g_sixstep_startup_active = false;
    const bool commutation_ok = applyCommutationForHall(hall);
    printState(serial, commutation_ok ? "hall_update" : "invalid_hall");
  }

  if (g_driver_enabled && g_sixstep_startup_active) {
    if ((now - g_sixstep_enable_ms) >= config::kSixStepStartupKickMs) {
      g_sixstep_startup_active = false;
      (void)applyCommutationForHall(readHallState());
      printState(serial, "startup_complete");
    } else if ((now - g_sixstep_last_kick_ms) >= config::kSixStepStartupStepMs) {
      g_sixstep_last_kick_ms = now;
      applyStepIndexWithDuty(static_cast<uint8_t>(g_step_index + 1u), config::kSixStepStartupPwmDuty);
      printState(serial, "startup_kick");
    }
  }

  if ((now - g_last_print_ms) >= config::kSixStepStatusIntervalMs) {
    g_last_print_ms = now;
    if (g_driver_enabled) {
      (void)drvReadAll();
    }
    printState(serial, "periodic");
  }
}

bool driverEnabled() {
  return g_driver_enabled;
}

}  // namespace motor_test
