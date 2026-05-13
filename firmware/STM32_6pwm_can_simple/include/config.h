#pragma once

#include <Arduino.h>

namespace config {

enum ControlMode : uint8_t {
  OPEN_LOOP = 0,  // Szenzor nelkuli fordulatszam parancs, indulashoz/teszthez egyszerubb.
  CLOSED_LOOP,    // Hall szenzorral visszacsatolt fordulatszam szabalyzas.
};

// AKTIV FO VEZERLESI MOD:
// Most: OPEN_LOOP, vagyis velocity_openloop lesz a SimpleFOC motion mode.
//
// Itt tudsz valtani a ket vezerlesi mod kozott:
//   constexpr ControlMode kControlMode = OPEN_LOOP;
//   constexpr ControlMode kControlMode = CLOSED_LOOP;
constexpr ControlMode kControlMode = OPEN_LOOP;

// AKTIV NYOMATEK MOD:
// A main.cpp-ben jelenleg fixen:
//   g_motor.torque_controller = TorqueControlType::voltage;
// Ez arammeres nelkuli feszultseg alapu hajtas.
//
// AKTIV PWM / MODULACIOS MOD:
// A main.cpp-ben jelenleg fixen:
//   g_motor.foc_modulation = FOCModulationType::SinePWM;
// Ez nem hatlepeses kommutacio, hanem szinuszos PWM.

constexpr uint32_t kSerialBaud = 115200;
constexpr uint32_t kEnableDelayMs = 1500;
constexpr uint32_t kControlTickUs = 1000;
constexpr uint32_t kDriverRetryMs = 100;
constexpr uint32_t kDebugPrintMs = 100;

constexpr uint32_t kDebugTxPin = PA2;
constexpr uint32_t kDebugRxPin = PA3;

constexpr uint32_t kHallAPin = PB6;
constexpr uint32_t kHallBPin = PB7;
constexpr uint32_t kHallCPin = PB11;

constexpr uint32_t kDrvEnablePin = PB12;
constexpr uint32_t kDrvFaultPin = PB0;
constexpr uint32_t kDrvCsPin = PA4;
constexpr uint32_t kDrvSckPin = PA5;
constexpr uint32_t kDrvSdoPin = PA6;
constexpr uint32_t kDrvSdiPin = PA7;

constexpr uint32_t kPwmAHin = PA8;
constexpr uint32_t kPwmBHin = PA9;
constexpr uint32_t kPwmCHin = PA10;
constexpr uint32_t kPwmALin = PB13;
constexpr uint32_t kPwmBLin = PB14;
constexpr uint32_t kPwmCLin = PB1;

// FDCAN1 labak es status uzenet beallitasok.
constexpr uint32_t kCanRxPin = PA11;
constexpr uint32_t kCanTxPin = PA12;
constexpr uint32_t kCanStatusIntervalMs = 20;
constexpr uint32_t kCanNodeId = 0x21;
constexpr uint32_t kCanStatusId = 0x180 + kCanNodeId;

constexpr float kBusVoltage = 12.0f;
constexpr int kPolePairs = 2;
constexpr float kSixPwmDeadZone = 0.005f;

// Open-loop mod: nincs FOC visszacsatolas, csak a megadott fordulatszamot hajtja.
constexpr float kOpenLoopTargetRpm = 1000.0f;
constexpr float kOpenLoopStartVoltageLimit = 0.80f;
constexpr float kOpenLoopVoltageLimit = 1.60f;
constexpr uint32_t kOpenLoopRampMs = 6000;

// Closed-loop mod: Hall szenzor alapjan tartja a cel fordulatszamot.
constexpr float kClosedLoopTargetRpm = 200.0f;
constexpr bool kClosedLoopReverse = false;
constexpr float kClosedLoopVoltageLimit = 0.60f;
constexpr float kClosedLoopVelocityLimitRpm = 400.0f;
constexpr float kClosedLoopP = 0.003f;
constexpr float kClosedLoopI = 0.01f;
constexpr float kClosedLoopD = 0.0f;
constexpr float kClosedLoopLpfTf = 0.3f;
constexpr float kClosedLoopOutputRamp = 20.0f;
constexpr uint32_t kClosedLoopMotionDownsample = 4;
constexpr bool kUseStartupAssist = true;
constexpr uint32_t kStartupAssistMs = 800;
constexpr float kStartupAssistRpm = 160.0f;
constexpr float kStartupAssistVoltageLimit = 1.90f;
constexpr uint32_t kHallWatchdogMs = 600;

// DRV8301 gate driver SPI konfiguracio. Ezeket csak akkor piszkald, ha a hardver kivanja.
constexpr uint8_t kDrvGateCurrentCode = 0;  // 0=1.7A, 1=0.7A, 2=0.25A
constexpr uint8_t kDrvOcpModeCode = 3;      // 0=current limit, 1=latch, 2=report, 3=off
constexpr uint8_t kDrvOcpAdjustCode = 27;
constexpr uint8_t kDrvOctwModeCode = 0;
constexpr uint8_t kDrvShuntGainCode = 0;
constexpr bool kDrvDcCalCh1 = false;
constexpr bool kDrvDcCalCh2 = false;
constexpr bool kDrvOcToff = false;

}  // namespace config
