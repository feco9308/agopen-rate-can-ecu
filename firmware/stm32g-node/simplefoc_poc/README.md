# SimpleFOC POC

Kulon probaprojekt a jelenlegi STM32G431 + DRV8301 + Hall motor bringuphoz.

Cel:
- ellenorizni, hogy a motor + Hall + 6PWM lanc stabilabban mukodik-e `SimpleFOC` alatt
- nem keverni ossze a jelenlegi CMake/CAN firmware-rel

## Pinout

PWM / DRV8301:
- `PA8`  -> phase A high
- `PB13` -> phase A low
- `PA9`  -> phase B high
- `PB14` -> phase B low
- `PA10` -> phase C high
- `PB15` -> phase C low
- `PB12` -> `EN_GATE`
- `PB0`  -> `nFAULT`

Hall:
- `PB6`  -> Hall A
- `PB7`  -> Hall B
- `PB11` -> Hall C

Motor fazis:
- `U -> A`
- `V -> B`
- `W -> C`

Hall motor oldali vezetek a jelenlegi feltetelezes szerint:
- `BLUE` -> Hall A / `PB6`
- `YELLOW` -> Hall B / `PB7`
- `WHITE` -> Hall C / `PB11`
- `BLACK` -> `GND`
- `RED` -> `3.3V` elso probara

## Build

Ez PlatformIO projektnkent van felteve.

Szukseges:
- PlatformIO telepitve
- ST-LINK feltoltes

Parancsok:

```powershell
cd simplefoc_poc
pio run
pio run -t upload
pio device monitor
```

## Serial parancsok

`115200` baud.

- `T5` -> kb. `5 rad/s` cel
- `T10` -> kb. `10 rad/s` cel
- `T0` -> stop

## Megjegyzes

Jelenleg:
- `pole_pairs = 7` feltetelezett
- `voltage_limit = 2.0V` ovatos kezdoertek
- Hall feedbacket hasznal, velocity modban

Ha a motor rossz iranyba indul vagy rangat:
- ket fazist cserelni
- vagy a Hall sorrendet ellenorizni
- vagy a `pole_pairs` erteket pontositani
