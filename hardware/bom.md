# Alkatrész lista (BOM)

## Áttekintés

Ez a lista a vetőgép CAN-alapú ECU rendszer kezdeti hardver elemeit tartalmazza.

❗ A lista folyamatosan változik a fejlesztés során.

---

## 1. Központi ECU (Teensy)

| Komponens       | Típus                | Megjegyzés       |
| --------------- | -------------------- | ---------------- |
| MCU             | Teensy 4.1           | központi vezérlő |
| Ethernet        | beépített / W5500    | LAN kommunikáció |
| CAN transceiver | SN65HVD230 / MCP2562 | CAN bus          |

---

## 2. Motor node (STM32G)

| Komponens       | Típus     | Megjegyzés                                                                     |
| --------------- | --------- | ------------------------------------------------------------------------------ |
| MCU             | STM32G4xx | motorvezérlés [Link](https://www.aliexpress.com/item/1005006552207317.html)    |
| Motor driver    | DRV83xx   | BLDC driver [Link](https://www.aliexpress.com/item/1005006448391882.html)      |
| CAN transceiver | MCP2562   | CAN kommunikáció [Link](https://www.aliexpress.com/item/1005005569468051.html) |

---

## 3. Tápellátás

| Komponens  | Típus      | Megjegyzés |
| ---------- | ---------- | ---------- |
| DC-DC      | 12V → 5V   | ECU táp    |
| DC-DC      | 12V → 3.3V | MCU táp    |
| biztosíték | -          | védelem    |

---

## 4. Szenzorok

| Komponens  | Típus         | Megjegyzés                                                            |
| ---------- | ------------- | --------------------------------------------------------------------- |
| Encoder    | AS5600 / AMT  | pozíció [Link](https://www.aliexpress.com/item/1005009516781005.html) |
| áram mérés | shunt + opamp | motor áram [Link](https://www.aliexpress.com/item/32885099898.html)   |

---

## 5. Kommunikáció

| Komponens | Típus        | Megjegyzés      |
| --------- | ------------ | --------------- |
| CAN bus   | twisted pair | hálózat         |
| lezárás   | 120 ohm      | bus termination |

---

## 6. Megjegyzések

* Az alkatrészek típusa változhat
* több alternatíva is lehetséges
* a lista nem végleges
