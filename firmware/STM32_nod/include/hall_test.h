#pragma once

#include <Arduino.h>

class Stream;

class HallTest {
 public:
  void begin();
  void update(Stream& serial);

 private:
  uint8_t readState() const;
  bool isValidState(uint8_t state) const;
  bool isExpectedStep(uint8_t previous, uint8_t current) const;
  void printBanner(Stream& serial);
  void printStatus(Stream& serial, uint8_t state, bool changed, bool valid_state,
                   bool expected_step);

  uint8_t previous_state_ = 0xFF;
  uint8_t start_state_ = 0xFF;
  uint32_t state_change_count_ = 0;
  uint32_t invalid_state_count_ = 0;
  uint32_t electrical_cycle_count_ = 0;
  uint32_t last_status_print_ms_ = 0;
  uint32_t last_pole_pair_hint_ms_ = 0;
};
