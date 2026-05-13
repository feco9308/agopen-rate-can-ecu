#pragma once

#include <Arduino.h>

// Kozos allapotcsomag: a motoros resz tolti, a CAN modul csak elkuldi.
struct AppStatus {
  bool driver_enabled = false;
  bool driver_ready = false;
  bool fault_active = false;
  bool closed_loop = false;
  bool startup_assist = false;
  uint8_t hall_state = 0;
  float target_rad_s = 0.0f;
  float measured_rad_s = 0.0f;
  float voltage_limit = 0.0f;
};
