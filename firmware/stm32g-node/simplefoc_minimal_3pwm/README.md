# SimpleFOC Minimal 3PWM

Ez egy tiszta bringup projekt:

- `SimpleFOC`
- `3PWM`
- `DRV8301 SPI init`
- `open-loop velocity`

Nincs benne:

- `CAN`
- regi debug logika
- zart hurku Hall szabalyzas
- USB CDC debug

Bekotes a jelenlegi szoftver szerint:

- `PA8` -> DRV `INH_A` / `PWM A high`
- `PA9` -> DRV `INH_B` / `PWM B high`
- `PA10` -> DRV `INH_C` / `PWM C high`
- `PB12` -> DRV `EN_GATE`
- `PB0` -> DRV `nFAULT`
- `PA5` -> DRV `SCLK`
- `PA6` -> DRV `SDO/MISO`
- `PA7` -> DRV `SDI/MOSI`
- `PA4` -> DRV `nSCS`
- `PC6` -> heartbeat LED

Hall bemenetek:

- `PB6` -> Hall U
- `PB7` -> Hall V
- `PB11` -> Hall W

TTL soros debug:

- `PA2` -> MCU `TX` -> USB-UART adapter `RX`
- `PA3` -> MCU `RX` -> USB-UART adapter `TX`
- kozos `GND` kotelezo
- `115200 8N1`

Fontos:

- csak `3.3V TTL` USB-UART adaptert hasznalj
- az adapter `5V` labat ne kosd ra
- ha csak logot akarsz olvasni, eleg a `PA2 -> adapter RX` es a kozos `GND`

Motor parameter a mostani motorhoz:

- `pole_pairs = 2`

Feltoltes utan a firmware:

- inicializalja a `DRV8301`-et `3PWM` modba
- 1 masodperc varakozas utan lassan raemeli a celt kb. `25 rpm` kore
- open-loop modban probal forgatni
- TTL soros portra kiirja a Hall allapotot es a DRV regisztereket
