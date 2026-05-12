#include "hall_test.h"

#include "config.h"

#include <Arduino.h>

namespace {

constexpr uint8_t kForwardSequence[6] = {0b001, 0b101, 0b100, 0b110, 0b010, 0b011};

const char* modeName() {
  switch (config::kTestMode) {
    case config::TEST_HALL:
      return "TEST_HALL";
    case config::TEST_POLE_PAIR:
      return "TEST_POLE_PAIR";
    default:
      return "UNKNOWN";
  }
}

void printStateBits(Stream& serial, uint8_t state) {
  serial.print((state >> 2) & 0x01);
  serial.print((state >> 1) & 0x01);
  serial.print(state & 0x01);
}

}  // namespace

void HallTest::begin() {
  pinMode(config::kHallAPin, INPUT_PULLUP);
  pinMode(config::kHallBPin, INPUT_PULLUP);
  pinMode(config::kHallCPin, INPUT_PULLUP);

  previous_state_ = readState();
  start_state_ = previous_state_;
  state_change_count_ = 0;
  invalid_state_count_ = 0;
  electrical_cycle_count_ = 0;
  last_status_print_ms_ = 0;
  last_pole_pair_hint_ms_ = 0;
}

void HallTest::update(Stream& serial) {
  const uint32_t now = millis();
  const uint8_t current_state = readState();
  const bool state_changed = current_state != previous_state_;
  const bool valid_state = isValidState(current_state);
  bool expected_step = true;

  if (state_changed) {
    if (valid_state && isValidState(previous_state_)) {
      expected_step = isExpectedStep(previous_state_, current_state);
      if (expected_step) {
        ++state_change_count_;
        if (current_state == start_state_ && state_change_count_ >= 6) {
          ++electrical_cycle_count_;
        }
      }
    } else {
      expected_step = false;
    }

    if (!valid_state) {
      ++invalid_state_count_;
    }

    printStatus(serial, current_state, true, valid_state, expected_step);
    previous_state_ = current_state;
    return;
  }

  const bool should_print_status = (now - last_status_print_ms_) >= config::kStatusPrintIntervalMs;
  const bool should_print_hint =
      config::kTestMode == config::TEST_POLE_PAIR &&
      (now - last_pole_pair_hint_ms_) >= config::kPolePairReminderIntervalMs;

  if (should_print_status || should_print_hint) {
    printStatus(serial, current_state, false, valid_state, true);
  }
}

uint8_t HallTest::readState() const {
  const uint8_t a = static_cast<uint8_t>(digitalRead(config::kHallAPin) == HIGH);
  const uint8_t b = static_cast<uint8_t>(digitalRead(config::kHallBPin) == HIGH);
  const uint8_t c = static_cast<uint8_t>(digitalRead(config::kHallCPin) == HIGH);
  return static_cast<uint8_t>((a << 2) | (b << 1) | c);
}

bool HallTest::isValidState(uint8_t state) const {
  return state != 0b000 && state != 0b111;
}

bool HallTest::isExpectedStep(uint8_t previous, uint8_t current) const {
  for (size_t index = 0; index < 6; ++index) {
    if (kForwardSequence[index] != previous) {
      continue;
    }

    const uint8_t next_forward = kForwardSequence[(index + 1) % 6];
    const uint8_t next_reverse = kForwardSequence[(index + 5) % 6];
    return current == next_forward || current == next_reverse;
  }

  return false;
}

void HallTest::printBanner(Stream& serial) {
  serial.println();
  serial.println("STM32 BLDC hall bringup");
  serial.print("Mode: ");
  serial.println(modeName());
  serial.print("Hall pins: A=");
  serial.print(config::kHallAPin);
  serial.print(" B=");
  serial.print(config::kHallBPin);
  serial.print(" C=");
  serial.println(config::kHallCPin);
  serial.println("Expected 6-step pattern: 001 101 100 110 010 011");
  if (config::kTestMode == config::TEST_POLE_PAIR) {
    serial.println("Rotate the shaft exactly one mechanical turn and note delta changes.");
    serial.println("pole_pairs = valid_hall_changes / 6");
  }
  serial.println();
}

void HallTest::printStatus(Stream& serial, uint8_t state, bool changed, bool valid_state,
                           bool expected_step) {
  const uint32_t now = millis();
  if (last_status_print_ms_ == 0) {
    printBanner(serial);
  }

  serial.print("t_ms=");
  serial.print(now);
  serial.print(" abc=");
  printStateBits(serial, state);
  serial.print(" changed=");
  serial.print(changed ? "yes" : "no");
  serial.print(" valid=");
  serial.print(valid_state ? "yes" : "no");
  serial.print(" seq_ok=");
  serial.print(expected_step ? "yes" : "no");
  serial.print(" changes=");
  serial.print(state_change_count_);
  serial.print(" invalid=");
  serial.print(invalid_state_count_);

  if (config::kTestMode == config::TEST_POLE_PAIR) {
    serial.print(" electrical_cycles=");
    serial.print(electrical_cycle_count_);
    serial.print(" est_pole_pairs_if_1_mech_rev=");
    serial.print(static_cast<float>(state_change_count_) / 6.0f, 2);
  }

  if (!valid_state) {
    serial.print(" WARNING=INVALID_HALL_STATE");
  } else if (!expected_step) {
    serial.print(" WARNING=UNEXPECTED_SEQUENCE");
  }

  serial.println();

  last_status_print_ms_ = now;
  if (config::kTestMode == config::TEST_POLE_PAIR) {
    last_pole_pair_hint_ms_ = now;
  }
}
