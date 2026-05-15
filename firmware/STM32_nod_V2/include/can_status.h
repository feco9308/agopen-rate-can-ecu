#pragma once

#include <Arduino.h>

#include "app_status.h"

namespace can_status {

// FDCAN1 inicializalasa.
bool begin();

// Ciklikus CAN status kuldes. Nem blokkol, ha tele van a TX FIFO.
void tick(const AppStatus& status, uint32_t now);

bool ready();

}  // namespace can_status
