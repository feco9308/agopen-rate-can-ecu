#pragma once

#include <Arduino.h>

namespace config {

enum TestMode : uint8_t {
  TEST_HALL = 0,
  TEST_POLE_PAIR,
  TEST_DRIVER_ENABLE,
  TEST_DRIVER_SPI_STABILITY,
  TEST_HALL_SENSOR_CHECK,
  TEST_PHASE_CHECK,
  TEST_6STEP,
  TEST_SIMPLEFOC_OPENLOOP,
  TEST_SIMPLEFOC_OPENLOOP_DEBUG,
  TEST_SIMPLEFOC_CLOSEDLOOP
};

constexpr TestMode kTestMode = TEST_SIMPLEFOC_CLOSEDLOOP;

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kStatusPrintIntervalMs = 250;
constexpr uint32_t kBannerDelayMs = 400;
constexpr uint32_t kPolePairReminderIntervalMs = 2000;
constexpr uint32_t kDriverEnableDelayMs = 1500;
constexpr uint32_t kDriverSpiReadbackIntervalMs = 500;
constexpr uint32_t kSixStepEnableDelayMs = 1500;
constexpr uint32_t kSixStepStatusIntervalMs = 250;
constexpr uint32_t kSixStepStartupKickMs = 400;
constexpr uint32_t kSixStepStartupStepMs = 35;
constexpr uint32_t kPhaseCheckEnableDelayMs = 1500;
constexpr uint32_t kPhaseCheckHoldMs = 1800;
constexpr uint32_t kPhaseCheckStatusIntervalMs = 250;
constexpr uint32_t kDrvSpiStabilityEnableDelayMs = 1500;
constexpr uint32_t kDrvSpiStabilityNoPwmMs = 3000;
constexpr uint32_t kDrvSpiStabilityStepMs = 700;
constexpr uint32_t kDrvSpiStabilityStatusIntervalMs = 100;
constexpr uint32_t kHallSensorCheckStatusIntervalMs = 100;
constexpr uint32_t kSimpleFocEnableDelayMs = 1500;
constexpr uint32_t kSimpleFocStatusIntervalMs = 250;
constexpr bool kSimpleFocLogHallChanges = true;
constexpr float kSimpleFocClosedloopTargetRpm = 120.0f;
constexpr float kSimpleFocClosedloopVoltageLimit = 1.4f;
constexpr float kSimpleFocClosedloopVelocityLimit = 400.0f;
constexpr bool kSimpleFocClosedloopReverse = false;
constexpr float kSimpleFocClosedloopVelP = 0.008f;
constexpr float kSimpleFocClosedloopVelI = 0.02f;
constexpr float kSimpleFocClosedloopVelD = 0.0f;
constexpr float kSimpleFocClosedloopVelLpfTf = 0.25f;
constexpr bool kSimpleFocClosedloopUseStartupAssist = true;
constexpr uint32_t kSimpleFocClosedloopStartupMs = 1500;
constexpr float kSimpleFocClosedloopStartupRpm = 50.0f;

constexpr uint32_t kDebugTxPin = PA2;
constexpr uint32_t kDebugRxPin = PA3;

constexpr uint32_t kHallAPin = PB6;
constexpr uint32_t kHallBPin = PB7;
constexpr uint32_t kHallCPin = PB11;

constexpr uint32_t kDrvEnablePin = PB12;
constexpr uint32_t kDrvFaultPin = PB0;

constexpr uint32_t kPwmAPin = PA8;
constexpr uint32_t kPwmBPin = PA9;
constexpr uint32_t kPwmCPin = PA10;

constexpr uint32_t kDrvCsPin = PA4;
constexpr uint32_t kDrvSckPin = PA5;
constexpr uint32_t kDrvSdoPin = PA6;
constexpr uint32_t kDrvSdiPin = PA7;

constexpr float kBusVoltage = 12.0f;
constexpr float kPwmLimit = 0.10f;
constexpr float kVoltageLimit = 0.50f;
constexpr float kSimpleFocOpenloopTargetRpm = 60.0f;
constexpr float kSimpleFocOpenloopVoltageLimit = 1.20f;
constexpr float kSimpleFocOpenloopStartupRpm = 20.0f;
constexpr float kSimpleFocOpenloopStartupVoltageLimit = 1.20f;
constexpr uint32_t kSimpleFocOpenloopStartupHoldMs = 1500;
constexpr uint32_t kSimpleFocOpenloopRampMs = 4000;
constexpr float kSimpleFocOpenloopDebugVoltage = 1.35f;
constexpr float kSimpleFocOpenloopDebugStepDeg = 10.0f;
constexpr uint32_t kSimpleFocOpenloopDebugStepMs = 50;
constexpr bool kDrvUse3PwmMode = true;
constexpr uint8_t kDrvGateCurrentCode = 0;   // 0=1.7A, 1=0.7A, 2=0.25A
constexpr uint8_t kDrvOcpModeCode = 0;       // 0=current limit, 1=latch shutdown, 2=report only, 3=disabled
constexpr uint8_t kDrvOcpAdjustCode = 8;     // 5-bit OC_ADJ_SET field
constexpr uint8_t kDrvOctwModeCode = 0;      // 0=report OT+OC on nOCTW
constexpr uint8_t kDrvShuntGainCode = 0;     // 0=10V/V, 1=20V/V, 2=40V/V, 3=80V/V
constexpr bool kDrvDcCalCh1 = false;
constexpr bool kDrvDcCalCh2 = false;
constexpr bool kDrvOcToff = false;           // false=cycle by cycle
constexpr int kConfiguredPolePairs = 2;
constexpr bool kInvertMotorDirection = false;
constexpr bool kDriverEnableAutoStart = true;
constexpr bool kDriverSpiAutoConfigure = true;
constexpr uint8_t kSixStepPwmDuty = 24;
constexpr uint8_t kSixStepStartupPwmDuty = 32;
constexpr uint8_t kPhaseCheckPwmDuty = 20;
constexpr uint8_t kDrvSpiStabilityPwmDuty = 16;
constexpr bool kSixStepReverseDirection = false;
constexpr int8_t kSixStepHallAdvance = 1;

}  // namespace config
