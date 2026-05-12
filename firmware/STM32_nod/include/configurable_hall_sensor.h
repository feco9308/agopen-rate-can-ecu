#pragma once

#include <Arduino.h>
#include <SimpleFOC.h>

class ConfigurableHallSensor : public Sensor {
 public:
  ConfigurableHallSensor(int hall_a, int hall_b, int hall_c, int pole_pairs,
                         int8_t sector_offset = 0);

  void init();
  void enableInterrupts(void (*doA)() = nullptr, void (*doB)() = nullptr,
                        void (*doC)() = nullptr);

  void handleA();
  void handleB();
  void handleC();

  void update() override;
  float getSensorAngle() override;
  float getVelocity() override;

  void attachSectorCallback(void (*onSectorChange)(int sector) = nullptr);
  void setSectorOffset(int8_t sector_offset);
  int8_t getSectorOffset() const;

  int pinA;
  int pinB;
  int pinC;
  int use_interrupt;
  Pullup pullup;
  int cpr;
  Direction direction;
  Direction old_direction;
  volatile int8_t hall_state;
  volatile int8_t electric_sector;
  volatile long electric_rotations;
  volatile long total_interrupts;
  float velocity_max = 1000.0f;

 private:
  static constexpr int8_t kDefaultElectricSectors[8] = {-1, 0, 4, 5, 2, 1, 3, -1};

  void updateState();
  int8_t remapElectricSector(int8_t hall_state_value) const;
  static int8_t normalizeSectorOffset(int8_t sector_offset);

  volatile unsigned long pulse_timestamp;
  volatile int A_active;
  volatile int B_active;
  volatile int C_active;
  void (*onSectorChange)(int sector) = nullptr;
  volatile long pulse_diff;
  int8_t sector_offset_;
};
