#include <Arduino.h>

#include "config.h"
#include "driver_test.h"
#include "hall_test.h"
#include "motor_test.h"

HardwareSerial DebugSerial(config::kDebugRxPin, config::kDebugTxPin);
HallTest g_hall_test;

namespace {

const char* selectedModeName() {
  switch (config::kTestMode) {
    case config::TEST_HALL:
      return "TEST_HALL";
    case config::TEST_POLE_PAIR:
      return "TEST_POLE_PAIR";
    case config::TEST_DRIVER_ENABLE:
      return "TEST_DRIVER_ENABLE";
    case config::TEST_DRIVER_SPI_STABILITY:
      return "TEST_DRIVER_SPI_STABILITY";
    case config::TEST_HALL_SENSOR_CHECK:
      return "TEST_HALL_SENSOR_CHECK";
    case config::TEST_PHASE_CHECK:
      return "TEST_PHASE_CHECK";
    case config::TEST_6STEP:
      return "TEST_6STEP";
    case config::TEST_SIMPLEFOC_OPENLOOP:
      return "TEST_SIMPLEFOC_OPENLOOP";
    case config::TEST_SIMPLEFOC_OPENLOOP_DEBUG:
      return "TEST_SIMPLEFOC_OPENLOOP_DEBUG";
    case config::TEST_SIMPLEFOC_CLOSEDLOOP:
      return "TEST_SIMPLEFOC_CLOSEDLOOP";
    default:
      return "UNKNOWN";
  }
}

void printBootBanner() {
  DebugSerial.println();
  DebugSerial.println("STM32 motor test project");
  DebugSerial.print("Selected mode: ");
  DebugSerial.println(selectedModeName());
  DebugSerial.println("Safety: driver starts disabled, no CAN, no ECU logic.");
}

}  // namespace

void setup() {
  pinMode(config::kDrvEnablePin, OUTPUT);
  digitalWrite(config::kDrvEnablePin, LOW);
  pinMode(config::kDrvFaultPin, INPUT_PULLUP);

  DebugSerial.begin(config::kSerialBaud);
  delay(config::kBannerDelayMs);
  printBootBanner();

  switch (config::kTestMode) {
    case config::TEST_HALL:
    case config::TEST_POLE_PAIR:
      g_hall_test.begin();
      break;
    case config::TEST_DRIVER_ENABLE:
      driver_test::begin(DebugSerial);
      break;
    case config::TEST_DRIVER_SPI_STABILITY:
    case config::TEST_HALL_SENSOR_CHECK:
    case config::TEST_PHASE_CHECK:
    case config::TEST_6STEP:
    case config::TEST_SIMPLEFOC_OPENLOOP:
    case config::TEST_SIMPLEFOC_OPENLOOP_DEBUG:
    case config::TEST_SIMPLEFOC_CLOSEDLOOP:
      motor_test::begin(DebugSerial);
      break;
  }
}

void loop() {
  switch (config::kTestMode) {
    case config::TEST_HALL:
    case config::TEST_POLE_PAIR:
      g_hall_test.update(DebugSerial);
      break;
    case config::TEST_DRIVER_ENABLE:
      driver_test::update(DebugSerial);
      break;
    case config::TEST_DRIVER_SPI_STABILITY:
    case config::TEST_HALL_SENSOR_CHECK:
    case config::TEST_PHASE_CHECK:
    case config::TEST_6STEP:
    case config::TEST_SIMPLEFOC_OPENLOOP:
    case config::TEST_SIMPLEFOC_OPENLOOP_DEBUG:
    case config::TEST_SIMPLEFOC_CLOSEDLOOP:
      motor_test::update(DebugSerial);
      break;
  }
}
