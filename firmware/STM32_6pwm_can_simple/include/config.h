#pragma once

#include <Arduino.h>

namespace config {

enum ControlMode : uint8_t {
  OPEN_LOOP = 0,  // Szenzor nelkuli fordulatszam parancs, indulashoz/teszthez egyszerubb.
  CLOSED_LOOP,    // Hall szenzorral visszacsatolt fordulatszam szabalyzas.
};

// AKTIV FO VEZERLESI MOD:
// Most: CLOSED_LOOP, vagyis Hall szenzoros fordulatszam szabalyzas.
//
// Itt tudsz valtani a ket vezerlesi mod kozott:
//   constexpr ControlMode kControlMode = OPEN_LOOP;
//   constexpr ControlMode kControlMode = CLOSED_LOOP;
constexpr ControlMode kControlMode = CLOSED_LOOP;

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
// Ha false, a firmware egyaltalan nem ir a soros debug portra.
// Fontos: a webes RPM plotterhez true kell, mert az a soros debug sort olvassa.
constexpr bool kSerialDebugEnabled = true;
// Ha true, a debug sor kimarad, amikor tele van a UART TX buffer.
// Igy a soros kiiras nem blokkolja a motorvezerlest.
constexpr bool kSerialDebugDropIfBusy = true;
// Kulon debug kapcsolok: ha closed-loopban gyanus a soros port, itt false-ra teheto
// ugy, hogy open-loop meresnel meg maradjon a webes grafikon.
constexpr bool kOpenLoopSerialDebugEnabled = true;
constexpr bool kClosedLoopSerialDebugEnabled = true;
constexpr uint32_t kOpenLoopDebugPrintMs = 100;
constexpr uint32_t kClosedLoopDebugPrintMs = 100;
constexpr uint32_t kEnableDelayMs = 1500;
constexpr uint32_t kControlTickUs = 1000;
constexpr uint32_t kDriverRetryMs = 100;

constexpr uint32_t kDebugTxPin = PA2;
constexpr uint32_t kDebugRxPin = PA3;

constexpr uint32_t kHallAPin = PB6;
constexpr uint32_t kHallBPin = PB7;
constexpr uint32_t kHallCPin = PB11;
// Ha true, a SimpleFOC nem rangatja jobbra-balra a motort iranykereseshez initFOC alatt.
// Ha closed-loop indulaskor rossz iranyba fut vagy nem tart, allitsd false-ra, vagy valtsd
// at a kHallSensorDirectionCw erteket.
constexpr bool kUseFixedHallSensorDirection = true;
constexpr bool kHallSensorDirectionCw = true;

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
constexpr uint32_t kCanCurrentId = 0x280 + kCanNodeId;

constexpr float kBusVoltage = 12.0f;
constexpr int kPolePairs = 2;
constexpr float kSixPwmDeadZone = 0.005f;
// SimpleFOC SinePWM kozepre igazitas. 6PWM-nel erdemes true-val probalni.
constexpr bool kModulationCentered = true;

// Open-loop mod: nincs FOC visszacsatolas, csak a megadott fordulatszamot hajtja.
constexpr float kOpenLoopTargetRpm = 2000.0f;
constexpr float kOpenLoopStartVoltageLimit = 0.80f;
constexpr float kOpenLoopVoltageLimit = 2.00f;
constexpr uint32_t kOpenLoopRampMs = 10000;

// Closed-loop mod: Hall szenzor alapjan tartja a cel fordulatszamot.
constexpr float kClosedLoopTargetRpm = 2000.0f;
constexpr bool kClosedLoopReverse = false;
constexpr float kClosedLoopVoltageLimit = 2.00f;
constexpr float kClosedLoopVelocityLimitRpm = 2500.0f;
constexpr uint32_t kClosedLoopRampMs = 10000;
// Closed-loop PID. Ha rangat: eloszor az I-t csokkentsd, utana a P-t.
constexpr float kClosedLoopP = 0.003f;
constexpr float kClosedLoopI = 0.01f;
constexpr float kClosedLoopD = 0.0f;
constexpr float kClosedLoopLpfTf = 0.3f;
constexpr float kClosedLoopOutputRamp = 20.0f;
constexpr uint32_t kClosedLoopMotionDownsample = 4;
constexpr bool kUseStartupAssist = true;
// Allo helyzetbol a Hall nem ad sebesseget, ezert kell egy rovid open-loop meglokes.
// Fontos: nem fix ido utan adja at, hanem csak stabil Hall fordulatnal.
constexpr uint32_t kStartupAssistMinMs = 250;
constexpr uint32_t kStartupAssistStableMs = 50;
constexpr float kStartupAssistHandoffRpm = 100.0f;
constexpr float kStartupAssistRpm = 150.0f;
constexpr float kStartupAssistVoltageLimit = 2.20f;
// Atadaskor a closed-loop rampa az aktualisan mert fordulatrol induljon,
// ne fix startup RPM-rol. Ettol nem esik vissza a celjel az atadasnal.
constexpr bool kClosedLoopRampFromMeasuredRpm = true;
constexpr float kClosedLoopRampMinStartRpm = 300.0f;
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

// IOUT arammeres diagnosztika.
// Bekotesi javaslat, ha ezek a labak szabadok a panelen:
//   IOUT_A -> PA0
//   IOUT_B -> PA1
//   IOUT_C -> PC_0
//   IOUT_REF / 1.65V kozeppont -> PC_1
// Ha mas ADC labakra kotod, itt kell atirni.
constexpr bool kCurrentSenseEnabled = true;
constexpr uint32_t kCurrentIoutAPin = PA0;
constexpr uint32_t kCurrentIoutBPin = PA1;
constexpr uint32_t kCurrentIoutCPin = PC_0;
constexpr uint32_t kCurrentRefPin = PC_1;
constexpr float kAdcReferenceVoltage = 3.3f;
constexpr float kCurrentShuntOhm = 0.005f;  // R005 = 5 milliohm
constexpr uint16_t kCurrentAdcAverages = 8;
constexpr uint32_t kCurrentSampleIntervalMs = 5;

}  // namespace config
