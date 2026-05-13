# STM32 6PWM CAN simple

Egyszerusitett motorvezerlo projekt STM32G431-hez.

- Csak 6PWM SimpleFOC driver van benne.
- `config::kControlMode` valasztja az `OPEN_LOOP` vagy `CLOSED_LOOP` modot.
- A `loop()` eloszor a motor vezerlesi tick-et futtatja, utana a nem blokkolt CAN status kuldest.
- A CAN status alapbol 20 ms-onkent megy a `0x180 + node_id` azonositoon.

Status CAN payload, 8 byte:

- byte 0: bitek: driver enabled, driver ready, fault, closed loop, startup assist
- byte 1: hall allapot, also 3 bit
- byte 2..3: target rad/s * 100, signed little endian
- byte 4..5: measured rad/s * 100, signed little endian
- byte 6..7: bus voltage * 100, signed little endian

## Hall RPM grafikon

A firmware debug sora tartalmazza:

- `target_rpm=...`
- `measured_rpm=...`
- `tim1_est_pwm_hz=...`

Python grafikon bongeszoben, matplotlib nelkul:

```powershell
& $env:USERPROFILE\.platformio\penv\Scripts\python.exe tools\plot_hall_rpm_web.py COM7 --baud 115200
```

A `COM7` helyere a sajat soros portot kell irni.
