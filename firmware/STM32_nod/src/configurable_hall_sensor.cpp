#include "configurable_hall_sensor.h"

ConfigurableHallSensor::ConfigurableHallSensor(int hall_a, int hall_b, int hall_c,
                                               int pole_pairs, int8_t sector_offset)
    : pinA(hall_a),
      pinB(hall_b),
      pinC(hall_c),
      use_interrupt(0),
      pullup(Pullup::USE_EXTERN),
      cpr(pole_pairs * 6),
      direction(Direction::CW),
      old_direction(Direction::CW),
      hall_state(-1),
      electric_sector(-1),
      electric_rotations(0),
      total_interrupts(0),
      pulse_timestamp(0),
      A_active(0),
      B_active(0),
      C_active(0),
      pulse_diff(0),
      sector_offset_(normalizeSectorOffset(sector_offset)) {}

int8_t ConfigurableHallSensor::normalizeSectorOffset(int8_t sector_offset) {
  int8_t normalized = static_cast<int8_t>(sector_offset % 6);
  if (normalized < 0) {
    normalized = static_cast<int8_t>(normalized + 6);
  }
  return normalized;
}

void ConfigurableHallSensor::setSectorOffset(int8_t sector_offset) {
  sector_offset_ = normalizeSectorOffset(sector_offset);
}

int8_t ConfigurableHallSensor::getSectorOffset() const {
  return sector_offset_;
}

void ConfigurableHallSensor::handleA() {
  A_active = digitalRead(pinA);
  updateState();
}

void ConfigurableHallSensor::handleB() {
  B_active = digitalRead(pinB);
  updateState();
}

void ConfigurableHallSensor::handleC() {
  C_active = digitalRead(pinC);
  updateState();
}

int8_t ConfigurableHallSensor::remapElectricSector(int8_t hall_state_value) const {
  if (hall_state_value < 0 || hall_state_value > 7) {
    return -1;
  }
  const int8_t base_sector = kDefaultElectricSectors[hall_state_value];
  if (base_sector < 0) {
    return -1;
  }
  return static_cast<int8_t>((base_sector + sector_offset_) % 6);
}

void ConfigurableHallSensor::updateState() {
  const int8_t new_hall_state = static_cast<int8_t>(C_active + (B_active << 1) + (A_active << 2));
  if (new_hall_state == hall_state) {
    return;
  }

  const long new_pulse_timestamp = _micros();
  hall_state = new_hall_state;

  const int8_t new_electric_sector = remapElectricSector(hall_state);
  if (new_electric_sector < 0) {
    electric_sector = new_electric_sector;
    pulse_timestamp = new_pulse_timestamp;
    total_interrupts++;
    return;
  }

  if (electric_sector >= 0) {
    const int8_t electric_sector_dif = new_electric_sector - electric_sector;
    if (electric_sector_dif > 3) {
      direction = Direction::CCW;
      electric_rotations += direction;
    } else if (electric_sector_dif < -3) {
      direction = Direction::CW;
      electric_rotations += direction;
    } else {
      direction = (new_electric_sector > electric_sector) ? Direction::CW : Direction::CCW;
    }
  }
  electric_sector = new_electric_sector;

  if (direction == old_direction) {
    pulse_diff = new_pulse_timestamp - pulse_timestamp;
  } else {
    pulse_diff = 0;
  }

  pulse_timestamp = new_pulse_timestamp;
  total_interrupts++;
  old_direction = direction;
  if (onSectorChange != nullptr) {
    onSectorChange(electric_sector);
  }
}

void ConfigurableHallSensor::attachSectorCallback(void (*sector_callback)(int sector)) {
  onSectorChange = sector_callback;
}

void ConfigurableHallSensor::update() {
  if (use_interrupt) {
    noInterrupts();
  } else {
    A_active = digitalRead(pinA);
    B_active = digitalRead(pinB);
    C_active = digitalRead(pinC);
    updateState();
  }

  angle_prev_ts = pulse_timestamp;
  const long last_electric_rotations = electric_rotations;
  const int8_t last_electric_sector = electric_sector;
  if (use_interrupt) {
    interrupts();
  }

  angle_prev =
      ((float)((last_electric_rotations * 6 + last_electric_sector) % cpr) / (float)cpr) * _2PI;
  full_rotations = (int32_t)((last_electric_rotations * 6 + last_electric_sector) / cpr);
}

float ConfigurableHallSensor::getSensorAngle() {
  return ((float)(electric_rotations * 6 + electric_sector) / (float)cpr) * _2PI;
}

float ConfigurableHallSensor::getVelocity() {
  noInterrupts();
  const long last_pulse_timestamp = pulse_timestamp;
  const long last_pulse_diff = pulse_diff;
  interrupts();

  if (last_pulse_diff == 0 ||
      ((long)(_micros() - last_pulse_timestamp) > last_pulse_diff * 2)) {
    return 0.0f;
  }

  return direction * (_2PI / (float)cpr) / (last_pulse_diff / 1000000.0f);
}

void ConfigurableHallSensor::init() {
  electric_rotations = 0;
  total_interrupts = 0;
  pulse_diff = 0;
  hall_state = -1;
  electric_sector = -1;

  if (pullup == Pullup::USE_INTERN) {
    pinMode(pinA, INPUT_PULLUP);
    pinMode(pinB, INPUT_PULLUP);
    pinMode(pinC, INPUT_PULLUP);
  } else {
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    pinMode(pinC, INPUT);
  }

  A_active = digitalRead(pinA);
  B_active = digitalRead(pinB);
  C_active = digitalRead(pinC);
  updateState();
  pulse_timestamp = _micros();
}

void ConfigurableHallSensor::enableInterrupts(void (*doA)(), void (*doB)(), void (*doC)()) {
  if (doA != nullptr) {
    attachInterrupt(digitalPinToInterrupt(pinA), doA, CHANGE);
  }
  if (doB != nullptr) {
    attachInterrupt(digitalPinToInterrupt(pinB), doB, CHANGE);
  }
  if (doC != nullptr) {
    attachInterrupt(digitalPinToInterrupt(pinC), doC, CHANGE);
  }
  use_interrupt = true;
}
