# STM32_nod Motor Bringup Summary

## Status

This project is a standalone STM32 motor bringup sandbox.

Current state:
- partial success
- the driver SPI communication is now stable
- Hall reading is working
- open-loop rotation is the current best baseline
- closed-loop with Hall still runs with visible pulsing/lurching and is not yet acceptable

Important conclusion:
- the motor is **not finished**
- there were real improvements, but the closed-loop result is still not good enough visually or behaviorally

## Goal Of This Folder

This folder was intentionally separated from the main ECU logic so motor bringup can be debugged safely.

Out of scope during bringup:
- CAN logic
- ECU state machine
- AgOpenGPS communication
- custom advanced control algorithms

Main goal:
- first make the motor rotate correctly and repeatably

## Hardware / Pin Mapping

- Debug TX: `PA2`
- Hall A: `PB6`
- Hall B: `PB7`
- Hall C: `PB11`
- DRV enable: `PB12`
- DRV fault: `PB0`
- PWM A: `PA8`
- PWM B: `PA9`
- PWM C: `PA10`
- DRV CS: `PA4`
- DRV SCK: `PA5`
- DRV SDO/MISO: `PA6`
- DRV SDI/MOSI: `PA7`

Motor phase wiring used during tests:
- OUTA: white
- OUTB: yellow
- OUTC: blue

## What Was Verified

### Hall side

- raw Hall inputs are readable
- valid 6-state Hall sequence was observed
- Hall cable issue was found and corrected earlier during testing
- `HallSensor` integration was improved by enabling internal pullups and forcing update calls where needed

### Driver side

- driver enable/fault handling works
- DRV register readback was originally unstable
- earlier symptoms:
  - `ctrl1` readback jumping between values like `0x0`, `0x90`, `0x120`, `0x400`
  - `frame_fault=yes`
  - instability even in `no_pwm` stage

### SPI side

The biggest breakthrough was on the SPI implementation.

Working changes:
- SPI switched to `16-bit` transfers
- SPI clock slowed to `SPI_BAUDRATEPRESCALER_256`
- extra chip-select timing margin added around transfers

After these changes:
- `frame_fault=no`
- `ctrl1=0x88`
- `ctrl_match=yes`
- stability test became repeatable both with and without low PWM stress

This strongly suggests the earlier DRV register instability was a real SPI/protocol problem, not just motor noise.

## Current Test Results

### 1. DRV SPI stability test

Result:
- good after SPI fixes

Observed healthy state:
- `frame_fault=no`
- `ctrl1=0x88`
- `ctrl2=0x0`
- `ctrl_match=yes`

### 2. SimpleFOC open-loop

Result:
- usable baseline

Observed:
- Hall transitions continue consistently while rotating
- no DRV SPI frame fault during run
- no major driver fault observed in the stable configuration

This is the current reference state for "basic motor rotation works".

### 3. SimpleFOC closed-loop with Hall

Result:
- technically improved, but still not acceptable mechanically

Observed:
- no SPI frame fault
- DRV register state stays stable
- Hall events are coming in
- closed-loop runs, but motion is still visibly pulsing / lurching

User observation:
- "olyan lüktetve megy"
- visually it still looks basically like before

That means the previous SPI bug was real and had to be fixed, but it was not the only reason for bad motion quality.

## Current Config Snapshot

Current selected mode in `include/config.h`:
- `TEST_SIMPLEFOC_CLOSEDLOOP`

Current closed-loop values:
- `target_rpm = 120.0`
- `voltage_limit = 1.20`
- `velocity_limit_rpm = 180.0`
- `vel P = 0.02`
- `vel I = 0.08`
- `vel D = 0.0`
- `LPF Tf = 0.12`
- `startup assist = enabled`
- `startup_rpm = 120.0`
- `startup_ms = 1500`

Current open-loop reference values:
- `target_rpm = 40.0`
- `voltage_limit = 1.00`

Motor config:
- configured pole pairs: `2`
- 3PWM mode enabled

## Honest Technical Assessment

What is now solid:
- project structure for isolated motor bringup
- Hall signal reading
- DRV enable/fault handling
- DRV SPI register communication after 16-bit slow SPI fix
- open-loop baseline rotation

What is still not solved:
- smooth closed-loop Hall control
- visually acceptable motion quality

Most likely current reality:
- the system is no longer blocked by broken SPI
- the remaining problem is in the quality of Hall-based closed-loop behavior in this setup
- more tuning and/or sensing improvements are still needed

## Recommended Next Steps

1. Keep the current SPI implementation as the baseline.
2. Keep open-loop as the known-good reference mode.
3. Continue closed-loop work only from this stable SPI baseline.
4. Compare Hall transition order carefully against mechanical direction and estimated electrical angle.
5. If closed-loop still remains visibly pulsing after tuning, consider a better position sensor later.

## Git / Repository Notes

This folder lives inside a larger repository that currently has many unrelated modified and untracked files.

Important:
- this motor bringup work should **not** be pushed blindly from the whole repo state
- unrelated work from other folders could easily get mixed into the same commit

Safe upload strategy:
- stage only `firmware/STM32_nod`
- review staged files
- create a dedicated commit only for this bringup folder
- push only after that review

## Key Files

- `include/config.h`
- `src/main.cpp`
- `src/motor_test.cpp`
- `include/motor_test.h`

