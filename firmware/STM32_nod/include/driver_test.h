#pragma once

class Stream;

namespace driver_test {

void begin(Stream& serial);
void update(Stream& serial);
bool driverEnabled();

}  // namespace driver_test
