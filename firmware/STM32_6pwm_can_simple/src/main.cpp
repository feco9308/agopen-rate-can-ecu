#include <Arduino.h>
#include <SimpleFOC.h>

#include "app_status.h"
#include "can_status.h"
#include "config.h"

extern "C" {
#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_spi.h"
}

HardwareSerial DebugSerial(config::kDebugRxPin, config::kDebugTxPin);

namespace {

// A DRV8301 gate driver allapota. Ezt az SPI olvasasokbol toltjuk fel.
struct DrvState {
  uint16_t status1 = 0;
  uint16_t status2 = 0;
  uint16_t control1 = 0;
  uint16_t control2 = 0;
  bool spi_ok = false;
  bool configured = false;
  bool frame_fault = false;
  bool control_match = false;
};

constexpr uint8_t kDrvRegStatus1 = 0x00u;
constexpr uint8_t kDrvRegStatus2 = 0x01u;
constexpr uint8_t kDrvRegControl1 = 0x02u;
constexpr uint8_t kDrvRegControl2 = 0x03u;

SPI_HandleTypeDef g_hspi1 = {};
DrvState g_drv = {};
AppStatus g_status = {};

BLDCMotor g_motor(config::kPolePairs);
BLDCDriver6PWM g_driver(config::kPwmAHin, config::kPwmALin, config::kPwmBHin,
                        config::kPwmBLin, config::kPwmCHin, config::kPwmCLin);
HallSensor g_hall(config::kHallAPin, config::kHallBPin, config::kHallCPin, config::kPolePairs);

// Futasi allapotok: ezek nem konfiguraciok, hanem a program menet kozbeni jelzoi.
bool g_open_loop_started = false;
bool g_closed_loop_started = false;
bool g_fault_latched = false;
uint32_t g_boot_ms = 0;
uint32_t g_last_control_us = 0;
uint32_t g_last_print_ms = 0;
uint32_t g_last_driver_retry_ms = 0;
uint32_t g_last_current_sample_ms = 0;
uint32_t g_startup_ms = 0;
uint32_t g_startup_ready_since_ms = 0;
uint32_t g_closed_loop_ramp_start_ms = 0;
float g_closed_loop_ramp_start_rad_s = 0.0f;
uint32_t g_open_loop_start_ms = 0;
uint32_t g_last_hall_ms = 0;
uint8_t g_last_hall_state = 0;

bool debugWriteLine(const char* line) {
  if (!config::kSerialDebugEnabled) {
    return false;
  }

  const size_t len = strlen(line);
  if (config::kSerialDebugDropIfBusy && DebugSerial.availableForWrite() < static_cast<int>(len + 2u)) {
    return false;
  }

  DebugSerial.write(reinterpret_cast<const uint8_t*>(line), len);
  DebugSerial.write('\r');
  DebugSerial.write('\n');
  return true;
}

float rpmToRadPerSec(float rpm) {
  return rpm * 0.104719755f;
}

float radPerSecToRpm(float rad_s) {
  return rad_s * 9.549296586f;
}

long roundFloatToLong(float value) {
  return static_cast<long>(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

float drvShuntGain() {
  switch (config::kDrvShuntGainCode) {
    case 0:
      return 10.0f;
    case 1:
      return 20.0f;
    case 2:
      return 40.0f;
    case 3:
      return 80.0f;
    default:
      return 10.0f;
  }
}

float adcToVoltage(uint16_t raw) {
  return (static_cast<float>(raw) * config::kAdcReferenceVoltage) / 4095.0f;
}

float readAveragedAdcVoltage(uint32_t pin) {
  uint32_t sum = 0;
  for (uint16_t index = 0; index < config::kCurrentAdcAverages; ++index) {
    sum += static_cast<uint32_t>(analogRead(pin));
  }
  const uint16_t raw = static_cast<uint16_t>(sum / config::kCurrentAdcAverages);
  return adcToVoltage(raw);
}

float ioutToAmp(float iout_v, float ref_v) {
  return (iout_v - ref_v) / (drvShuntGain() * config::kCurrentShuntOhm);
}

uint32_t tim1EstimatedPwmHz() {
#ifdef TIM1
  const uint32_t prescaler = TIM1->PSC + 1u;
  const uint32_t period = TIM1->ARR + 1u;
  if (prescaler == 0u || period == 0u) {
    return 0u;
  }
  const bool center_aligned = (TIM1->CR1 & TIM_CR1_CMS) != 0u;
  const uint32_t divisor = center_aligned ? 2u : 1u;
  return SystemCoreClock / prescaler / period / divisor;
#else
  return 0u;
#endif
}

bool closedLoopMode() {
  return config::kControlMode == config::CLOSED_LOOP;
}

bool debugForActiveModeEnabled() {
  if (closedLoopMode()) {
    return config::kClosedLoopSerialDebugEnabled;
  }
  return config::kOpenLoopSerialDebugEnabled;
}

uint32_t debugIntervalMs() {
  if (closedLoopMode()) {
    return config::kClosedLoopDebugPrintMs;
  }
  return config::kOpenLoopDebugPrintMs;
}

bool faultActive() {
  return digitalRead(config::kDrvFaultPin) == LOW;
}

uint8_t readHallState() {
  const uint8_t a = static_cast<uint8_t>(digitalRead(config::kHallAPin) == HIGH);
  const uint8_t b = static_cast<uint8_t>(digitalRead(config::kHallBPin) == HIGH);
  const uint8_t c = static_cast<uint8_t>(digitalRead(config::kHallCPin) == HIGH);
  return static_cast<uint8_t>((a << 2) | (b << 1) | c);
}

// SimpleFOC HallSensor megszakitas kezelo fuggvenyek.
void doHallA() {
  g_hall.handleA();
}

void doHallB() {
  g_hall.handleB();
}

void doHallC() {
  g_hall.handleC();
}

// DRV8301 Control1 regiszter osszeallitasa a config.h beallitasai alapjan.
uint16_t drvControl1(bool gate_reset) {
  uint16_t value = 0;
  value |= static_cast<uint16_t>((config::kDrvGateCurrentCode & 0x03u) << 0);
  value |= static_cast<uint16_t>((config::kDrvOcpModeCode & 0x03u) << 4);
  value |= static_cast<uint16_t>((config::kDrvOcpAdjustCode & 0x1Fu) << 6);
  if (gate_reset) {
    value |= static_cast<uint16_t>(1u << 2);
  }
  return value;
}

// DRV8301 Control2 regiszter osszeallitasa a config.h beallitasai alapjan.
uint16_t drvControl2() {
  uint16_t value = 0;
  if (config::kDrvOcToff) {
    value |= static_cast<uint16_t>(1u << 0);
  }
  if (config::kDrvDcCalCh2) {
    value |= static_cast<uint16_t>(1u << 1);
  }
  if (config::kDrvDcCalCh1) {
    value |= static_cast<uint16_t>(1u << 2);
  }
  value |= static_cast<uint16_t>((config::kDrvShuntGainCode & 0x03u) << 3);
  value |= static_cast<uint16_t>((config::kDrvOctwModeCode & 0x03u) << 5);
  return value;
}

// SPI1 inicializalasa a DRV8301-hez.
bool drvSpiInit() {
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

// Egy 16 bites SPI keret kuldese/fogadasa a DRV8301 fele.
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
  g_drv.spi_ok = (status == HAL_OK);
  return status;
}

// DRV8301 regiszter olvasas. A chipnel az olvasas ket SPI keretbol all.
HAL_StatusTypeDef drvReadRegister(uint8_t reg, uint16_t* value) {
  uint16_t rx_word = 0;
  HAL_StatusTypeDef status =
      drvTransfer16(static_cast<uint16_t>(0x8000u | ((reg & 0x0Fu) << 11)), &rx_word);
  if (status != HAL_OK) {
    return status;
  }
  status = drvTransfer16(0u, &rx_word);
  if (status != HAL_OK) {
    return status;
  }
  g_drv.frame_fault = ((rx_word & 0x8000u) != 0u);
  if (value != nullptr) {
    *value = static_cast<uint16_t>(rx_word & 0x07FFu);
  }
  return HAL_OK;
}

// DRV8301 regiszter iras.
HAL_StatusTypeDef drvWriteRegister(uint8_t reg, uint16_t value) {
  uint16_t rx_word = 0;
  return drvTransfer16(static_cast<uint16_t>(((reg & 0x0Fu) << 11) | (value & 0x07FFu)),
                       &rx_word);
}

// Fontos DRV8301 status/control regiszterek frissitese.
bool drvReadAll() {
  bool ok = true;
  ok = (drvReadRegister(kDrvRegStatus1, &g_drv.status1) == HAL_OK) && ok;
  ok = (drvReadRegister(kDrvRegStatus2, &g_drv.status2) == HAL_OK) && ok;
  ok = (drvReadRegister(kDrvRegControl1, &g_drv.control1) == HAL_OK) && ok;
  ok = (drvReadRegister(kDrvRegControl2, &g_drv.control2) == HAL_OK) && ok;
  g_drv.control_match = (g_drv.control1 == drvControl1(false)) && (g_drv.control2 == drvControl2());
  return ok;
}

bool drvStatus2GvddOv() {
  return (g_drv.status2 & 0x80u) != 0u;
}

bool drvReady() {
  return g_status.driver_enabled && g_drv.configured && g_drv.control_match &&
         !g_drv.frame_fault && !faultActive() && g_drv.status1 == 0u && !drvStatus2GvddOv();
}

bool drvConfigValid() {
  return g_status.driver_enabled && g_drv.control_match && !g_drv.frame_fault && !faultActive() &&
         g_drv.status1 == 0u && !drvStatus2GvddOv();
}

// Motor es gate driver teljes letiltasa.
void drvDisable() {
  g_motor.disable();
  g_driver.disable();
  digitalWrite(config::kDrvEnablePin, LOW);
  g_status.driver_enabled = false;
  g_drv.configured = false;
}

// Gate driver bekapcsolasa es 6PWM-hez szukseges DRV8301 konfiguracio beirasa.
bool drvEnableAndConfigure() {
  g_driver.disable();
  // EN_GATE magasra huzasa utan a DRV8301 mar fogadja az SPI konfiguraciot.
  digitalWrite(config::kDrvEnablePin, HIGH);
  delay(50);
  g_status.driver_enabled = true;

  (void)drvWriteRegister(kDrvRegControl1, drvControl1(true));
  delay(1);
  const bool write_ok = (drvWriteRegister(kDrvRegControl1, drvControl1(false)) == HAL_OK) &&
                        (drvWriteRegister(kDrvRegControl2, drvControl2()) == HAL_OK);
  const bool read_ok = drvReadAll();
  g_drv.configured = write_ok && read_ok && drvConfigValid();
  if (!g_drv.configured) {
    drvDisable();
    return false;
  }
  return true;
}

// A config.h-ban megadott RPM cel atvaltasa rad/s-ra.
// Itt valik szet az open-loop es closed-loop cel fordulatszam.
float targetRadS() {
  if (closedLoopMode()) {
    return rpmToRadPerSec(config::kClosedLoopTargetRpm) *
           (config::kClosedLoopReverse ? -1.0f : 1.0f);
  }
  return rpmToRadPerSec(config::kOpenLoopTargetRpm);
}

float openLoopRampedTargetRadS(uint32_t now) {
  const float final_target = rpmToRadPerSec(config::kOpenLoopTargetRpm);
  if (g_open_loop_start_ms == 0u || config::kOpenLoopRampMs == 0u) {
    return final_target;
  }

  const uint32_t elapsed_ms = now - g_open_loop_start_ms;
  if (elapsed_ms >= config::kOpenLoopRampMs) {
    return final_target;
  }

  return final_target * (static_cast<float>(elapsed_ms) /
                         static_cast<float>(config::kOpenLoopRampMs));
}

float openLoopRampRatio(uint32_t now) {
  if (g_open_loop_start_ms == 0u || config::kOpenLoopRampMs == 0u) {
    return 1.0f;
  }
  const uint32_t elapsed_ms = now - g_open_loop_start_ms;
  if (elapsed_ms >= config::kOpenLoopRampMs) {
    return 1.0f;
  }
  return static_cast<float>(elapsed_ms) / static_cast<float>(config::kOpenLoopRampMs);
}

float openLoopRampedVoltageLimit(uint32_t now) {
  const float ratio = openLoopRampRatio(now);
  return config::kOpenLoopStartVoltageLimit +
         (config::kOpenLoopVoltageLimit - config::kOpenLoopStartVoltageLimit) * ratio;
}

float closedLoopStartupTargetRadS() {
  const float direction = config::kClosedLoopReverse ? -1.0f : 1.0f;
  return rpmToRadPerSec(config::kStartupAssistRpm) * direction;
}

float closedLoopRampedTargetRadS(uint32_t now) {
  const float final_target = targetRadS();
  const float start_target = g_closed_loop_ramp_start_rad_s;
  if (!g_closed_loop_started) {
    return 0.0f;
  }
  if (g_status.startup_assist) {
    return start_target;
  }
  if (g_closed_loop_ramp_start_ms == 0u || config::kClosedLoopRampMs == 0u) {
    return final_target;
  }

  const uint32_t elapsed_ms = now - g_closed_loop_ramp_start_ms;
  if (elapsed_ms >= config::kClosedLoopRampMs) {
    return final_target;
  }

  const float ratio = static_cast<float>(elapsed_ms) / static_cast<float>(config::kClosedLoopRampMs);
  return start_target + ((final_target - start_target) * ratio);
}

void updateCurrentSense(uint32_t now) {
  if (!config::kCurrentSenseEnabled) {
    g_status.current_a = 0.0f;
    g_status.current_b = 0.0f;
    g_status.current_c = 0.0f;
    g_status.current_ref_v = 0.0f;
    return;
  }
  if ((now - g_last_current_sample_ms) < config::kCurrentSampleIntervalMs) {
    return;
  }
  g_last_current_sample_ms = now;

  const float ref_v = readAveragedAdcVoltage(config::kCurrentRefPin);
  const float iout_a_v = readAveragedAdcVoltage(config::kCurrentIoutAPin);
  const float iout_b_v = readAveragedAdcVoltage(config::kCurrentIoutBPin);
  const float iout_c_v = readAveragedAdcVoltage(config::kCurrentIoutCPin);

  g_status.current_ref_v = ref_v;
  g_status.current_a = ioutToAmp(iout_a_v, ref_v);
  g_status.current_b = ioutToAmp(iout_b_v, ref_v);
  g_status.current_c = ioutToAmp(iout_c_v, ref_v);
}

// A pillanatnyi allapot osszegyujtese CAN-hez es debug kiirashoz.
void statusUpdate(uint32_t now) {
  updateCurrentSense(now);
  g_status.fault_active = faultActive();
  g_status.closed_loop = closedLoopMode();
  g_status.hall_state = readHallState();
  g_status.target_rad_s = closedLoopMode() ? closedLoopRampedTargetRadS(now) : openLoopRampedTargetRadS(now);
  g_status.measured_rad_s = closedLoopMode() ? g_motor.shaft_velocity : g_hall.getVelocity();
  g_status.voltage_limit =
      closedLoopMode() ? config::kClosedLoopVoltageLimit : openLoopRampedVoltageLimit(now);
  g_status.driver_ready = drvReady();
}

// A bekapcsolas utan kEnableDelayMs ido mulva engedelyezi a DRV8301-et es a motort.
// Closed-loop modban itt fut az initFOC is.
void motorEnableTick(uint32_t now) {
  if (g_status.driver_enabled || (now - g_boot_ms) < config::kEnableDelayMs) {
    return;
  }
  if ((now - g_last_driver_retry_ms) < config::kDriverRetryMs) {
    return;
  }
  g_last_driver_retry_ms = now;

  if (!drvEnableAndConfigure()) {
    debugWriteLine("DRV configure failed");
    return;
  }

  g_driver.enable();
  g_motor.enable();

  if (closedLoopMode()) {
    debugWriteLine("initFOC start");
    g_motor.initFOC();
    debugWriteLine("initFOC done");
    g_closed_loop_started = true;
    g_status.startup_assist = config::kUseStartupAssist;
    g_startup_ms = now;
    g_startup_ready_since_ms = 0u;
    g_closed_loop_ramp_start_ms = config::kUseStartupAssist ? 0u : now;
    g_closed_loop_ramp_start_rad_s = config::kUseStartupAssist ? closedLoopStartupTargetRadS() : 0.0f;
    g_last_hall_ms = now;
    g_last_hall_state = readHallState();
    g_motor.controller =
        g_status.startup_assist ? MotionControlType::velocity_openloop : MotionControlType::velocity;
  } else {
    g_open_loop_started = true;
    g_open_loop_start_ms = now;
  }
  debugWriteLine("motor enabled");
}

// Ez a fo motorvezerlesi tick.
// Open-loop: g_motor.move(target) visszacsatolas nelkul.
// Closed-loop: eloszor opcionelis startup assist, utana loopFOC() + move(target).
void motorControlTick(uint32_t now) {
  const uint32_t now_us = micros();
  if ((now_us - g_last_control_us) < config::kControlTickUs) {
    return;
  }
  g_last_control_us = now_us;

  if (faultActive()) {
    if (!g_fault_latched) {
      debugWriteLine("driver fault, disabled");
      g_fault_latched = true;
    }
    g_open_loop_started = false;
    g_closed_loop_started = false;
    drvDisable();
    return;
  }

  motorEnableTick(now);
  if (!drvReady()) {
    return;
  }

  if (closedLoopMode()) {
    const uint8_t hall = readHallState();
    if (hall != g_last_hall_state) {
      g_last_hall_state = hall;
      g_last_hall_ms = now;
    }

    if (g_status.startup_assist) {
      g_motor.voltage_limit = config::kStartupAssistVoltageLimit;
      g_motor.move(closedLoopStartupTargetRadS());

      const uint32_t startup_elapsed_ms = now - g_startup_ms;
      const float measured_abs_rpm = fabsf(radPerSecToRpm(g_hall.getVelocity()));
      if (startup_elapsed_ms >= config::kStartupAssistMinMs &&
          measured_abs_rpm >= config::kStartupAssistHandoffRpm) {
        if (g_startup_ready_since_ms == 0u) {
          g_startup_ready_since_ms = now;
        }
      } else {
        g_startup_ready_since_ms = 0u;
      }

      const bool handoff_ready =
          g_startup_ready_since_ms != 0u &&
          (now - g_startup_ready_since_ms) >= config::kStartupAssistStableMs;
      if (handoff_ready) {
        g_status.startup_assist = false;
        g_motor.controller = MotionControlType::velocity;
        g_motor.PID_velocity.reset();
        g_closed_loop_ramp_start_ms = now;
        if (config::kClosedLoopRampFromMeasuredRpm) {
          const float direction = config::kClosedLoopReverse ? -1.0f : 1.0f;
          const float start_rpm = max(config::kClosedLoopRampMinStartRpm, measured_abs_rpm);
          g_closed_loop_ramp_start_rad_s = rpmToRadPerSec(start_rpm) * direction;
        } else {
          g_closed_loop_ramp_start_rad_s = closedLoopStartupTargetRadS();
        }
      }
      return;
    }

    if ((now - g_last_hall_ms) > config::kHallWatchdogMs) {
      debugWriteLine("hall watchdog, disabled");
      g_closed_loop_started = false;
      drvDisable();
      return;
    }

    g_motor.voltage_limit = config::kClosedLoopVoltageLimit;
    g_motor.loopFOC();
    g_motor.move(closedLoopRampedTargetRadS(now));
  } else if (g_open_loop_started) {
    g_motor.voltage_limit = openLoopRampedVoltageLimit(now);
    g_motor.move(openLoopRampedTargetRadS(now));
  }
}

void writeOpenLoopDebugLine() {
  const long target_rpm = roundFloatToLong(radPerSecToRpm(g_status.target_rad_s));
  const long measured_rpm = roundFloatToLong(radPerSecToRpm(g_status.measured_rad_s));
  const long voltage_cv = roundFloatToLong(g_status.voltage_limit * 100.0f);
  char line[56] = {};
  snprintf(line, sizeof(line), "O;%ld;%ld;%ld;%lu;%u", target_rpm, measured_rpm, voltage_cv,
           static_cast<unsigned long>(tim1EstimatedPwmHz()),
           static_cast<unsigned int>(g_status.hall_state));
  (void)debugWriteLine(line);
}

void writeClosedLoopDebugLine() {
  const long target_rpm = roundFloatToLong(radPerSecToRpm(g_status.target_rad_s));
  const long measured_rpm = roundFloatToLong(radPerSecToRpm(g_status.measured_rad_s));
  const long voltage_cv = roundFloatToLong(g_status.voltage_limit * 100.0f);
  char line[64] = {};
  snprintf(line, sizeof(line), "C;%ld;%ld;%ld;%lu;%u;%u", target_rpm, measured_rpm, voltage_cv,
           static_cast<unsigned long>(tim1EstimatedPwmHz()),
           static_cast<unsigned int>(g_status.hall_state),
           static_cast<unsigned int>(g_status.startup_assist));
  (void)debugWriteLine(line);
}

// Lassu soros status kiiras, hogy latszodjon mit csinal a vezerlo.
void debugPrintTick(uint32_t now) {
  if (!config::kSerialDebugEnabled || !debugForActiveModeEnabled()) {
    return;
  }
  if ((now - g_last_print_ms) < debugIntervalMs()) {
    return;
  }
  g_last_print_ms = now;

  if (closedLoopMode()) {
    writeClosedLoopDebugLine();
  } else {
    writeOpenLoopDebugLine();
  }
}

// Alap GPIO iranyok. A driver indulaskor le van tiltva.
void initPins() {
  pinMode(config::kDrvEnablePin, OUTPUT);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);
  pinMode(config::kPwmAHin, OUTPUT);
  pinMode(config::kPwmBHin, OUTPUT);
  pinMode(config::kPwmCHin, OUTPUT);
  pinMode(config::kPwmALin, OUTPUT);
  pinMode(config::kPwmBLin, OUTPUT);
  pinMode(config::kPwmCLin, OUTPUT);
  if (config::kCurrentSenseEnabled) {
    pinMode(config::kCurrentIoutAPin, INPUT_ANALOG);
    pinMode(config::kCurrentIoutBPin, INPUT_ANALOG);
    pinMode(config::kCurrentIoutCPin, INPUT_ANALOG);
    pinMode(config::kCurrentRefPin, INPUT_ANALOG);
    analogReadResolution(12);
  }
  digitalWrite(config::kDrvEnablePin, LOW);
}

// SimpleFOC objektumok inicializalasa.
// Itt is a config::kControlMode donti el, hogy open-loop vagy closed-loop beallitas kell.
void initSimpleFoc() {
  g_driver.pwm_frequency = 20000;
  g_driver.voltage_power_supply = config::kBusVoltage;
  g_driver.voltage_limit = config::kBusVoltage;
  g_driver.dead_zone = config::kSixPwmDeadZone;
  g_driver.init();

  g_motor.linkDriver(&g_driver);
  // AKTIV NYOMATEK MOD:
  // Voltage torque control. Ehhez nem kell arammeres, a motor feszultsegparancsot kap.
  g_motor.torque_controller = TorqueControlType::voltage;

  // AKTIV PWM / MODULACIOS MOD:
  // SinePWM = szinuszos fazisfeszultseg PWM-mel, nem 6 lepeses trapez kommutacio.
  // Kiserlethez masik lehetoseg: FOCModulationType::SpaceVectorPWM.
  g_motor.foc_modulation = FOCModulationType::SinePWM;
  g_motor.modulation_centered = config::kModulationCentered ? 1 : 0;

  if (closedLoopMode()) {
    // CLOSED_LOOP eseten SimpleFOC motion mode:
    // MotionControlType::velocity = Hall szenzorral zart hurku fordulatszam szabalyzas.
    g_hall.pullup = Pullup::USE_INTERN;
    g_hall.init();
    g_hall.enableInterrupts(doHallA, doHallB, doHallC);
    g_motor.linkSensor(&g_hall);
    if (config::kUseFixedHallSensorDirection) {
      g_motor.sensor_direction =
          config::kHallSensorDirectionCw ? Direction::CW : Direction::CCW;
    }
    g_motor.controller = MotionControlType::velocity;
    g_motor.voltage_limit = config::kClosedLoopVoltageLimit;
    g_motor.velocity_limit = rpmToRadPerSec(config::kClosedLoopVelocityLimitRpm);
    g_motor.PID_velocity.P = config::kClosedLoopP;
    g_motor.PID_velocity.I = config::kClosedLoopI;
    g_motor.PID_velocity.D = config::kClosedLoopD;
    g_motor.PID_velocity.output_ramp = config::kClosedLoopOutputRamp;
    g_motor.LPF_velocity.Tf = config::kClosedLoopLpfTf;
    g_motor.motion_downsample = config::kClosedLoopMotionDownsample;
    g_motor.voltage_sensor_align = 0.5f;
  } else {
    // OPEN_LOOP eseten SimpleFOC motion mode:
    // MotionControlType::velocity_openloop = szenzor nelkuli fordulatszam parancs.
    // A Hall jel ilyenkor csak mereshez/debughoz kell, nem szabalyzashoz.
    g_hall.pullup = Pullup::USE_INTERN;
    g_hall.init();
    g_hall.enableInterrupts(doHallA, doHallB, doHallC);
    g_motor.controller = MotionControlType::velocity_openloop;
    g_motor.voltage_limit = config::kOpenLoopStartVoltageLimit;
    g_motor.velocity_limit = rpmToRadPerSec(config::kOpenLoopTargetRpm);
  }

  g_motor.init();
}

}  // namespace

void setup() {
  if (config::kSerialDebugEnabled) {
    DebugSerial.begin(config::kSerialBaud);
    debugWriteLine("");
    debugWriteLine("STM32 6PWM simple CAN controller");
  }

  initPins();
  const bool spi_ok = drvSpiInit();
  initSimpleFoc();
  const bool can_ok = can_status::begin();

  g_boot_ms = millis();
  g_last_control_us = micros();

  if (config::kSerialDebugEnabled) {
    char line[96] = {};
    snprintf(line, sizeof(line), "mode=%s SPI=%s CAN=%s",
             closedLoopMode() ? "closed_loop" : "open_loop", spi_ok ? "ok" : "failed",
             can_ok ? "ok" : "failed");
    debugWriteLine(line);
  } else {
    (void)spi_ok;
    (void)can_ok;
  }
}

void loop() {
  const uint32_t now = millis();

  // Szandekos sorrend:
  // 1. motorvezerles, hogy a PWM/FOC idoben fusson
  // 2. status frissites
  // 3. kulon CAN modul meghivasa, nem blokkolva
  // 4. lassu debug print
  motorControlTick(now);
  statusUpdate(now);
  can_status::tick(g_status, now);
  debugPrintTick(now);
}
