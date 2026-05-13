# STM32 6PWM CAN simple

Egyszerusitett motorvezerlo projekt STM32G431-hez.

- Csak 6PWM SimpleFOC driver van benne.
- `config::kControlMode` valasztja az `OPEN_LOOP` vagy `CLOSED_LOOP` modot.
- A `loop()` eloszor a motor vezerlesi tick-et futtatja, utana a nem blokkolt CAN status kuldest.
- A CAN status alapbol 20 ms-onkent megy a `0x180 + node_id` azonositoon.

Closed-loop indulaskor a program eloszor rovid open-loop startup assistot futtat,
majd a cel RPM-et rampaval emeli. Ez azert kell, mert nagyon alacsony fordulaton
a Hall sebessegmeres darabos, es a zart hurku szabalyozas konnyen megrangathatja
a motort, ha rogton teljes cel fordulatszamot kap.

Mostani beallitasban a startup assist csak addig fut, amig stabil Hall fordulat
nem latszik. Az atadas feltetele: `kStartupAssistHandoffRpm` folott legyen a
Hallbol mert fordulat `kStartupAssistStableMs` ideig.

A soros debug open-loop es closed-loop modra kulon kapcsolhato:
`kOpenLoopSerialDebugEnabled`, `kClosedLoopSerialDebugEnabled`.
Closed-loopban alapbol ritkabban kuld status sort, hogy a motor ciklust kevesbe terhelje.

Az inditaskori jobbra-balra mozgast a SimpleFOC szenzorirany-keresese okozza.
Ez fix Hall irannyal kihagyhato: `kUseFixedHallSensorDirection`.

Startup assist utan a closed-loop rampa indulhat az aktualisan mert RPM-rol:
`kClosedLoopRampFromMeasuredRpm`. Ez csokkenti az atadasi visszaesest.

Status CAN payload, 8 byte:

- byte 0: bitek: driver enabled, driver ready, fault, closed loop, startup assist
- byte 1: hall allapot, also 3 bit
- byte 2..3: target rad/s * 100, signed little endian
- byte 4..5: measured rad/s * 100, signed little endian
- byte 6..7: bus voltage * 100, signed little endian

Current CAN payload, ID `0x280 + node_id`, 8 byte:

- byte 0..1: IOUT_A current A * 100, signed little endian
- byte 2..3: IOUT_B current A * 100, signed little endian
- byte 4..5: IOUT_C current A * 100, signed little endian
- byte 6..7: IOUT_REF voltage V * 1000, signed little endian

IOUT bekotes alap config szerint:

- `IOUT_A -> PA0`
- `IOUT_B -> PA1`
- `IOUT_C -> PC_0`
- `IOUT_REF / 1.65V -> PC_1`

R005 shunttal es DRV gain code 0 mellett a skala:

```text
I = (V_iout - V_ref) / (10 * 0.005)
1 A = 50 mV
```

## Hall RPM grafikon

A firmware debug sora tartalmazza:

- Open-loop: `O;target_rpm;measured_rpm;voltage_centivolt;tim1_est_pwm_hz;hall_state`
- Closed-loop: `C;target_rpm;measured_rpm;voltage_centivolt;tim1_est_pwm_hz;hall_state;startup_assist`

Pelda:

```text
O;1000;998;160;20000;5
```

A `hall_state` also 3 bitje az `A/B/C` Hall jel, vagyis peldaul `5 = 101`.
Ervenyes Hall allapotok: `001`, `010`, `011`, `100`, `101`, `110`.
A webes grafikon aljan harom digitalis nyom is latszik: Hall A, Hall B, Hall C.

Python grafikon bongeszoben, matplotlib nelkul:

```powershell
& $env:USERPROFILE\.platformio\penv\Scripts\python.exe tools\plot_hall_rpm_web.py COM7 --baud 115200
```

A `COM7` helyere a sajat soros portot kell irni.
