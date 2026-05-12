#pragma once

class Stream;

namespace motor_test {

void begin(Stream& serial);
void update(Stream& serial);
bool driverEnabled();

}  // namespace motor_test
