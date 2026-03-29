# CAN mátrix

## Áttekintés

A rendszer CAN buszon kommunikál a Teensy ECU és az ESP32 node-ok között.

### Sebesség
- 500 kbit/s

### Formátum
- Standard CAN (11 bit ID)

---

## ID struktúra

- `0x080` → broadcast
- `0x100 + node_id` → master → node
- `0x180 + node_id` → node → master
- `0x1C0 + node_id` → diagnosztika

---

## GLOBAL_CONTROL (0x080)

### Leírás
Közös rendszer állapot és referencia

| Byte | Jel | Leírás |
|------|-----|--------|
| 0 | system_mode | OFF / MANUAL / AUTO / stb |
| 1 | control_flags | enable, sync, estop |
| 2-3 | base_rpm | cél fordulat |
| 4-5 | sync_pos | referencia pozíció |
| 6 | sequence | számláló |
| 7 | reserved | - |

---

## NODE_CMD (0x101)

| Byte | Jel | Leírás |
|------|-----|--------|
| 0 | command | enable, disable, zero |
| 1 | flags | opciók |
| 2-3 | trim_rpm | finom korrekció |
| 4-5 | pos_offset | pozíció offset |
| 6 | seq | számláló |
| 7 | reserved | - |

---

## NODE_STATUS_FAST (0x181)

| Byte | Jel | Leírás |
|------|-----|--------|
| 0 | status_flags | állapot |
| 1 | error_code | hiba |
| 2-3 | actual_rpm | aktuális fordulat |
| 4-5 | actual_pos | aktuális pozíció |
| 6 | alive | heartbeat |
| 7 | sync_error | eltérés |

---

## NODE_DIAG (0x1C1)

| Byte | Jel | Leírás |
|------|-----|--------|
| 0-1 | bus_voltage | tápfesz |
| 2-3 | motor_current | áram |
| 4 | controller_temp | hőmérséklet |
| 5 | motor_temp | hőmérséklet |
| 6 | fault_flags | hibák |
| 7 | warning_flags | figyelmeztetések |

---

## GLOBAL_ESTOP (0x082)

| Byte | Jel | Leírás |
|------|-----|--------|
| 0 | estop_cmd | vészleállítás |
| 1 | reason | ok |

---

## Megjegyzések

- RPM + pozíció együtt szükséges a szinkronhoz
- node oldali szabályozás gyorsabb
- master referencia alapú működés
- rendszer könnyen bővíthető több node-ra
