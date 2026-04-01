# 🔌 Hardware csatlakoztatás – Node modul

## 🎯 Cél

Ez a dokumentum a CAN-alapú motorvezérlő node hardveres felépítését és bekötését írja le.

A node feladata:

* BLDC motor vezérlés (DRV8301)
* vetőtárcsa pozíció mérés (AS5048A)
* motor kommutáció (Hall szenzor)
* hőmérséklet és áram monitorozás
* CAN kommunikáció az ECU-val

---

# 🧱 Rendszer felépítés

```text
STM32G431 (MCU)
      │
      ├── DRV8301 (motor driver)
      │        └── BLDC motor
      │
      ├── AS5048A (abszolút encoder)
      │
      ├── Hall szenzor (motorból)
      │
      ├── Hőmérők (2 db NTC)
      │
      └── CAN (MCP2562)
```

---

# 🧠 MCU

Használt fejlesztői board:

* **STM32G431CBU6 (WeAct)**

Feladatok:

* PWM generálás (6PWM)
* SPI kommunikáció (DRV8301 + AS5048A)
* ADC mérések (áram, feszültség, hőmérséklet)
* Hall szenzor feldolgozás
* CAN kommunikáció

---

# ⚡ DRV8301 – motor driver

Feladat:

* 3 fázisú BLDC motor meghajtás
* gate driver + MOSFET vezérlés
* áram visszacsatolás (SO1, SO2)

---

# 🔗 Pin kiosztás

## PWM kimenetek (motor vezérlés)

| Funkció | STM32 pin | DRV8301 |
| ------- | --------- | ------- |
| PWM_AH  | PA8       | INH_A   |
| PWM_AL  | PB13      | INL_A   |
| PWM_BH  | PA9       | INH_B   |
| PWM_BL  | PB14      | INL_B   |
| PWM_CH  | PA10      | INH_C   |
| PWM_CL  | PB15      | INL_C   |

---

## SPI busz (közös)

| Funkció | STM32 pin |
| ------- | --------- |
| SCK     | PA5       |
| MISO    | PA6       |
| MOSI    | PA7       |

### Chip Select

| Eszköz  | STM32 pin |
| ------- | --------- |
| DRV8301 | PA4       |
| AS5048A | PB10      |

---

## DRV8301 vezérlés és státusz

| Funkció | STM32 pin |
| ------- | --------- |
| nFAULT  | PB0       |
| nOCTW   | PB1       |
| EN_GATE | PB12      |
| DC_CAL  | PB2       |

---

## Árammérés (ADC)

| Funkció | STM32 pin |
| ------- | --------- |
| SO1     | PA0       |
| SO2     | PA1       |

---

## Feszültség mérés

| Funkció    | STM32 pin |
| ---------- | --------- |
| VBUS_SENSE | PA2       |

---

## Hőmérséklet mérés

| Funkció     | STM32 pin | Típus   |
| ----------- | --------- | ------- |
| TEMP_MOTOR  | PA3       | NTC 10k |
| TEMP_DRIVER | PB5       | NTC 10k |

---

## Motor Hall szenzor

| Funkció | STM32 pin |
| ------- | --------- |
| HALL_A  | PB6       |
| HALL_B  | PB7       |
| HALL_C  | PB11      |

Feladat:

* kommutáció
* rotor pozíció (alap szint)

---

## AS5048A abszolút encoder

| Funkció | STM32 pin |
| ------- | --------- |
| SCK     | PA5       |
| MISO    | PA6       |
| MOSI    | PA7       |
| CS      | PB10      |

Feladat:

* vetőtárcsa pontos pozíció
* szinkronizáció CAN rendszeren

---

## CAN kommunikáció

| Funkció | STM32 pin |
| ------- | --------- |
| CAN_RX  | PB8       |
| CAN_TX  | PB9       |

CAN transceiver:

* **MCP2562**

---

## Debug / programozás

### SWD

| Funkció | STM32 pin |
| ------- | --------- |
| SWDIO   | PA13      |
| SWCLK   | PA14      |

---

### UART (opcionális)

| Funkció | STM32 pin |
| ------- | --------- |
| TX      | PA15      |
| RX      | PB3       |

---

# 🔌 Külső csatlakozások

## Motor (DRV8301)

| Jel  | Leírás  |
| ---- | ------- |
| OUTA | fázis A |
| OUTB | fázis B |
| OUTC | fázis C |

---

## Táp

| Jel | Leírás |
| --- | ------ |
| VIN | 12V    |
| GND | föld   |

---

## CAN

| Jel  | Leírás   |
| ---- | -------- |
| CANH | CAN high |
| CANL | CAN low  |

---

## Encoder (AS5048A)

| Jel  | Leírás      |
| ---- | ----------- |
| VCC  | 3.3V        |
| GND  | föld        |
| SCK  | SPI clock   |
| MISO | adat        |
| MOSI | adat        |
| CS   | chip select |

---

## Hall szenzor

| Jel | Leírás    |
| --- | --------- |
| A   | Hall A    |
| B   | Hall B    |
| C   | Hall C    |
| VCC | 5V / 3.3V |
| GND | föld      |

---

# ⚠️ Megjegyzések

* SPI busz közös, de **külön CS kötelező**
* SWD lábakat ne használd másra
* CAN buszon szükséges a 120Ω lezárás
* minden GND közös legyen
* hőmérők NTC osztóval ADC-re
* DRV8301 EN_GATE csak inicializálás után legyen aktív

---

# 🧠 Rendszer logika

```text
Motor Hall → motor vezérlés
AS5048A → vetőtárcsa pozíció
ADC → áram + hő + feszültség
CAN → kommunikáció ECU-val
```

---

# 🚀 Rövid összefoglaló

Ez a node modul:

* STM32G431 alapú
* DRV8301 motor driverrel működik
* AS5048A abszolút encoderrel méri a vetőtárcsát
* Hall szenzorral vezérli a motort
* CAN buszon kommunikál

👉 cél: **sebességarányos, szinkron vetés vezérlés**

