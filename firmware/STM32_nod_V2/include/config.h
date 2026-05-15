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
// SOROS DEBUG SZINT:
//   0 = teljesen kikapcsolva
//   1 = csak grafikon-kompatibilis O;/C; adatfolyam a tools/plot_hall_rpm_web.py-hoz
//   2 = grafikon adatfolyam + D; diagnosztikai sorok es szoveges esemenyek
constexpr uint8_t kSerialDebugLevel = 2;
// Ha true, a debug sor kimarad, amikor tele van a UART TX buffer.
// Igy a soros kiiras nem blokkolja a motorvezerlest.
constexpr bool kSerialDebugDropIfBusy = true;
// Kulon grafikon kapcsolok: ha closed-loopban gyanus a soros port, itt false-ra teheto
// ugy, hogy open-loop meresnel meg maradjon a webes grafikon adatfolyam.
constexpr bool kOpenLoopGraphDebugEnabled = true;
constexpr bool kClosedLoopGraphDebugEnabled = true;
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
// Ha kUseFixedHallSensorDirection = true, ezt az erteket is be kell allitani, kulonben
// az initFOC meg mindig vegrehajt egy 700 ms-os alingazasi impulzust (ez okozza az
// indulaskori RPM spike-ot). Hall szenzoros sebessegvezerleshel 0.0f altalaban megfelelo.
// Pontosabb ertek egyszer bemerheto: keress "MOT: Zero elec. angle:" sort a debug kimenten.
constexpr float kHallZeroElectricAngle = 0.0f;

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

constexpr float kBusVoltage = 14.0f;

// Motor fizikai jellemzők.
// Kv = névleges max fordulatszám / névleges feszültség (RC motor definicio).
// FONTOS: kMotorRatedVoltage a MOTOR sajatsaga, nem a busz feszultsege!
// Ha kBusVoltage > kMotorRatedVoltage, a motor tobb mint 10000 RPM-re is felgyorsulhat
// terheletlen allapotban – ezt a celfurdulat-limittel kell megelozni.
constexpr float kMotorMaxRpm      = 10000.0f;  // Gyartoi max RPM a nevleges fesz.-en
constexpr float kMotorRatedVoltage = 12.0f;    // A motor nevleges feszultsege
constexpr float kMotorKv           = kMotorMaxRpm / kMotorRatedVoltage;  // 833 RPM/V

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
// Feszultsegkorlat: Kv-alapu szamitas a cel RPM-bol.
// V_limit = (target_rpm / Kv) * margin, de KOTELEZOEN korlatozva a modulaciohoz
// biztonsagos maximumra, kulonben VCP_UV (bootstrap) fault lesz!
//
// Modulaciotipusonkenti max fazisfeszultseg 90%-os biztonsagi hatarral:
//   SinePWM + kModulationCentered=true  (centered):    Vbus * 0.45  (Vbus/2 * 0.9)   <-- aktiv
//   SinePWM + kModulationCentered=false (min-clamped): Vbus * 0.52  (Vbus/sqrt(3) * 0.9)
//   SpaceVectorPWM:                                     Vbus * 0.52  (Vbus/sqrt(3) * 0.9)
//
// FONTOS: kModulationCentered=true eseten max Vq = Vbus/2 (NEM Vbus/sqrt(3)!).
// 24V-on centered SinePWM-mel: 24 * 0.45 = 10.8V -> max ~9000 RPM biztonsagosan.
// 10 000 RPM-hez kModulationCentered=false kell (Vbus/sqrt(3) headroom, 24V * 0.52 = 12.48V).
constexpr float kClosedLoopVoltageMargin = 1.3f;
constexpr float kMaxSafePhaseVoltage = kBusVoltage * 0.80f; // eredeti 0,45F 
constexpr float kClosedLoopVoltageLimit =
    (kClosedLoopTargetRpm / kMotorKv * kClosedLoopVoltageMargin < kMaxSafePhaseVoltage)
    ? kClosedLoopTargetRpm / kMotorKv * kClosedLoopVoltageMargin
    : kMaxSafePhaseVoltage;
// Sebessegkorlat: automatikusan a cel RPM 1.25-szorose.
constexpr float kClosedLoopVelocityLimitRpm = kClosedLoopTargetRpm * 1.25f;
// Closed-loop cel RPM felfutasi ideje. Kisebb ertek = gyorsabban eri el a 2000 RPM-et.
constexpr uint32_t kClosedLoopRampMs = 4000;
// Closed-loop sebesseg-PID. Voltage torque modban a kimenet V.
// Ha tul lassu: P-t novelje. Ha rangat/oszcillal: P-t csokkentse.
constexpr float kClosedLoopP = 0.070f;
constexpr float kClosedLoopI = 0.01f;
constexpr float kClosedLoopD = 0.0f;
constexpr float kClosedLoopLpfTf = 0.12f;
constexpr float kClosedLoopOutputRamp = 300.0f;
constexpr uint32_t kClosedLoopMotionDownsample = 4;
constexpr bool kUseStartupAssist = true;
// Allo helyzetbol a Hall nem ad sebesseget, ezert kell egy rovid open-loop meglokes.
// Fontos: nem fix ido utan adja at, hanem csak stabil Hall fordulatnal.
constexpr uint32_t kStartupAssistMinMs = 250;
constexpr uint32_t kStartupAssistStableMs = 50;
constexpr float kStartupAssistHandoffRpm = 100.0f;
// Startup assist feszultseglimit: Kv-alapu szamitas, nem fix ertek.
// A feszultseg a rampa aktualis RPM-jebol szamitodik: V = (ramp_rpm / Kv) * boost.
// Igy t=0-ban 0V, es aranyosan no a fordulat-rámpával – nem ugrik azonnal nagyra.
// Ha a motor nem tud felgyorsulni: novelje a boost-ot.
// Ha rug/rang indulaskor: csoksentse a boost-ot.
constexpr float kStartupAssistVoltageBoost = 1.4f;
// Startup assist csak akkor adja at closed-loopnak, ha a Hall RPM mar koveti
// az aktualis celrampa legalabb ekkora reszet.
constexpr float kStartupAssistHandoffTargetRatio = 0.70f;
// Handoff utan csak diagnosztikai idobelyeg; a celrampa nem all meg, open-loopbol
// closed-loopba ugyanazt az egy celgörbet folytatjuk.
constexpr uint32_t kClosedLoopHandoffSettleMs = 80;
constexpr uint32_t kHallWatchdogMs = 1000;

// DRV8301 gate driver SPI konfiguracio. Ezeket csak akkor piszkald, ha a hardver kivanja.
constexpr uint8_t kDrvGateCurrentCode = 0;  // 0=1.7A, 1=0.7A, 2=0.25A
constexpr uint8_t kDrvOcpModeCode = 3;      // 0=current limit, 1=latch, 2=report, 3=off
constexpr uint8_t kDrvOcpAdjustCode = 27;
constexpr uint8_t kDrvOctwModeCode = 0;
constexpr uint8_t kDrvShuntGainCode = 0;
constexpr bool kDrvDcCalCh1 = false;
constexpr bool kDrvDcCalCh2 = false;
constexpr bool kDrvOcToff = false;

// IOUT arammeres - GenericCurrentSense 2-shunt konfig.
// Ic = -(Ia+Ib) szamitva (Kirchhoff).
//   IOUT_A -> PA0  (ADC1_IN1)
//   IOUT_B -> PA1  (ADC1_IN2)
constexpr bool kCurrentSenseEnabled = true;
constexpr uint32_t kCurrentIoutAPin = PA0;
constexpr uint32_t kCurrentIoutBPin = PA1;
constexpr float kAdcReferenceVoltage = 3.3f;
constexpr float kCurrentRefVoltage = kAdcReferenceVoltage / 2.0f;
constexpr float kCurrentShuntOhm = 0.005f;  // R005 = 5 milliohm
constexpr float kCurrentSenseGain = 10.0f;  // DRV gain code 0 = 10x
constexpr uint32_t kCurrentSampleIntervalMs = 5;

// Mert az aktiv torque mode voltage, a SimpleFOC belso current control most nincs hasznalva.
// Az arammeres ettol meg fut: CAN/debug log, valamint egyszeru szoftveres tularam letiltas.
constexpr bool kCurrentProtectionEnabled = false;
constexpr uint32_t kCurrentProtectionBlankingMs = 500;
constexpr float kMeasuredPhaseCurrentLimitA = 8.0f;

}  // namespace config
