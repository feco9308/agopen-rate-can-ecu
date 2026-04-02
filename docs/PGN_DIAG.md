# Vetőgép ECU Saját PGN Tábla

## Tartományok

- `32800..32899` = ECU config / service
- `32900..32999` = Node provisioning / node diag

## 1. ECU Config PGN-ek

| PGN | Név | Irány | Cél |
|---|---|---|---|
| 32800 | `ECU_CFG_GET` | Tool/App -> ECU | Config blokk lekérése |
| 32801 | `ECU_CFG_SET` | Tool/App -> ECU | Config blokk írása |
| 32802 | `ECU_CFG_SAVE` | Tool/App -> ECU | Config mentése flash-be |
| 32803 | `ECU_CFG_LOAD` | Tool/App -> ECU | Config visszatöltése flash-ből |
| 32804 | `ECU_CFG_STATUS` | ECU -> Tool/App | Aktuális config visszaküldése |
| 32805 | `ECU_DIAG_CONTROL` | Tool/App -> ECU | Diagnosztika be/ki, szint állítás |
| 32806 | `ECU_DIAG_STATUS` | ECU -> Tool/App | ECU összesített állapot |

## 32800 - ECU_CFG_GET

### Payload

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0 | `block_id` | `uint8` | Melyik config blokkot kérjük |
| 1 | `reserved` | `uint8` | 0 |
| 2..7 | `reserved` | - | 0 |

### `block_id` értékek

| Érték | Blokk |
|---|---|
| 0 | gép alap config |
| 1 | szenzor/csatorna config |
| 2 | diag config |

## 32801 - ECU_CFG_SET

### Payload

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0 | `block_id` | `uint8` | Melyik blokkot írjuk |
| 1 | `apply_now` | `uint8` | 0/1 |
| 2..7 | `data` | blokkfüggő | Konfigurációs adatok |

### `block_id = 0` gép alap config

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 2 | `active_sensor_count` | `uint8` | 1..3 |
| 3 | `configured_row_count` | `uint8` | sorok/szektorok száma |
| 4..5 | `holes_per_rev` | `uint16` | tárcsa furatszám |
| 6..7 | `upm_scale_x10` | `uint16` | pl. 100.0 => 1000 |

Megjegyzés:
- `gear_ratio` külön blokkban is lehet, de egyszerűbb, ha egy következő frame-ben küldjük vagy `block_id=1` alá tesszük.

### `block_id = 1` hajtás/számítás config

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 2..3 | `gear_ratio_x100` | `uint16` | pl. 2.00 => 200 |
| 4..5 | `trim_limit_rpm_x10` | `int16` | pl. 200.0 => 2000 |
| 6..7 | `position_kp_x1000` | `uint16` | pl. 0.500 => 500 |

### `block_id = 2` diag config

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 2 | `diag_enable` | `uint8` | 0/1 |
| 3 | `diag_stream_enable` | `uint8` | 0/1 |
| 4 | `diag_period_ms_div10` | `uint8` | pl. 20 => 200 ms |
| 5 | `reserved` | `uint8` | 0 |
| 6..7 | `reserved` | - | 0 |

## 32804 - ECU_CFG_STATUS

Az ECU ugyanazokat a blokkokat küldi vissza, mint amiket a `SET` használ.

## 32805 - ECU_DIAG_CONTROL

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0 | `diag_enable` | `uint8` | 0/1 |
| 1 | `stream_enable` | `uint8` | 0/1 |
| 2 | `sensor_mask` | `uint8` | bit0=s0, bit1=s1, bit2=s2 |
| 3 | `node_mask_lo` | `uint8` | node 1..8 |
| 4 | `node_mask_hi` | `uint8` | node 9..16 |
| 5 | `period_ms_div10` | `uint8` | pl. 10 => 100 ms |
| 6 | `detail_level` | `uint8` | 0=basic, 1=extended |
| 7 | `reserved` | `uint8` | 0 |

## 32806 - ECU_DIAG_STATUS

Összesített ECU állapot.

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0 | `active_sensor_count` | `uint8` | aktív szenzorcsatornák |
| 1 | `configured_row_count` | `uint8` | konfigurált sorok |
| 2 | `eth_link` | `uint8` | 0/1 |
| 3 | `can_link` | `uint8` | 0/1 |
| 4 | `diag_enable` | `uint8` | 0/1 |
| 5 | `reserved` | `uint8` | 0 |
| 6..7 | `uptime_s_div10` | `uint16` | üzemidő |

---

## 2. Node Provisioning PGN-ek

| PGN | Név | Irány | Cél |
|---|---|---|---|
| 32900 | `NODE_DISCOVER` | Tool/App -> Node broadcast | node keresés |
| 32901 | `NODE_UID_REPORT` | Node -> Tool/App | UID és alap adatok |
| 32902 | `NODE_ASSIGN` | Tool/App -> Node | UID alapján csatorna és node ID kiosztás |
| 32903 | `NODE_CFG_READ` | Tool/App -> Node | node config visszaolvasás |
| 32904 | `NODE_CFG_SAVE` | Tool/App -> Node | node config mentés |
| 32905 | `NODE_TEST_SPIN` | Tool/App -> Node | motor forgatás teszt |
| 32906 | `NODE_DIAG_STATUS` | Node -> Tool/App | részletes node diag |
| 32907 | `NODE_REBOOT` | Tool/App -> Node | node újraindítás |

## 32900 - NODE_DISCOVER

Broadcast lekérés.

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0 | `request_type` | `uint8` | 1 = discover |
| 1..7 | `reserved` | - | 0 |

## 32901 - NODE_UID_REPORT

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_low32` | `uint32` | STM UID alsó rész |
| 4 | `sensor_channel` | `uint8` | 0..2 |
| 5 | `node_id` | `uint8` | 1..16 |
| 6 | `fw_version_major` | `uint8` | fw verzió |
| 7 | `fw_version_minor` | `uint8` | fw verzió |

Megjegyzés:
- ha kell a teljes UID, akkor 2 frame-re lehet bontani:
  - `NODE_UID_REPORT_A`
  - `NODE_UID_REPORT_B`

## 32902 - NODE_ASSIGN

UID alapján hozzárendelés.

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_low32` | `uint32` | cél node UID azonosító |
| 4 | `sensor_channel` | `uint8` | melyik “kocsi” / csatorna |
| 5 | `node_id` | `uint8` | melyik sor |
| 6 | `save_now` | `uint8` | 0/1 |
| 7 | `flags` | `uint8` | tartalék |

Értelmezés:
- a node ebből maga számolja a figyelt CAN ID-kat
- nem kell nyers CAN ID-t beírni

Példa:
- `sensor_channel = 1`
- `node_id = 4`

Akkor a node figyelheti:
- `GLOBAL_CONTROL = 0x200`
- `NODE_CMD = 0x214`
- `NODE_STATUS = 0x284`

## 32903 - NODE_CFG_READ

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_low32` | `uint32` | cél node UID |
| 4..7 | `reserved` | - | 0 |

## 32904 - NODE_CFG_SAVE

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_low32` | `uint32` | cél node UID |
| 4 | `save` | `uint8` | 1 = mentsd |
| 5..7 | `reserved` | - | 0 |

## 32905 - NODE_TEST_SPIN

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_low32` | `uint32` | cél node UID |
| 4..5 | `test_rpm_x10` | `int16` | teszt fordulat |
| 6 | `duration_s` | `uint8` | időtartam |
| 7 | `direction` | `uint8` | 0=default, 1=reverse |

Megjegyzés:
- ez nagyon jó szervizhez, mert rögtön látni, melyik motor mozog

## 32906 - NODE_DIAG_STATUS

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0 | `sensor_channel` | `uint8` | 0..2 |
| 1 | `node_id` | `uint8` | 1..16 |
| 2..3 | `actual_rpm_x10` | `int16` | aktuális motor RPM |
| 4..5 | `trim_rpm_x10` | `int16` | ECU által küldött trim |
| 6 | `error_code` | `uint8` | node hiba |
| 7 | `status_flags` | `uint8` | online/run/fault |

Ha kell részletesebb diag, akkor ezt 2 PGN-re bontanám.

## Javasolt bővített diag PGN

| PGN | Név | Irány | Cél |
|---|---|---|---|
| 32908 | `NODE_DIAG_EXT` | Node -> Tool/App | pozíció, sync hiba, alive |

### 32908 - NODE_DIAG_EXT

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0 | `sensor_channel` | `uint8` | 0..2 |
| 1 | `node_id` | `uint8` | 1..16 |
| 2..3 | `actual_pos_u16` | `uint16` | motor pozíció |
| 4 | `sync_error_x256rev` | `int8` | szinkronhiba |
| 5 | `alive_counter` | `uint8` | alive |
| 6..7 | `reserved` | - | 0 |

## 32907 - NODE_REBOOT

| Byte | Név | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_low32` | `uint32` | cél node UID |
| 4 | `magic` | `uint8` | pl. `0xA5` |
| 5..7 | `reserved` | - | 0 |

---

## Javasolt működési sorrend szervizhez

1. `NODE_DISCOVER`
2. node válaszol `NODE_UID_REPORT`
3. kiválasztod az adott motort
4. `NODE_TEST_SPIN`
5. ha jó motor mozdult:
6. `NODE_ASSIGN`
7. `NODE_CFG_SAVE`

---

## Megjegyzések

- A `sensor_channel` logikai mező legyen, ne nyers CAN ID.
- A node ebből számolja ki, melyik CAN bankot figyelje.
- A `trim_rpm_x10` legyen előjeles `int16`.
- A diag stream legyen kapcsolható, mert folyamatos küldésnél buszterhelést okoz.
- Ha teljes STM UID kell, érdemes 64 vagy 96 bites UID riportot 2 külön PGN-be bontani.

## Későbbi web UI-hoz ez jól illeszkedik

A webes felület ezekre a funkciókra tud épülni:
- ECU config olvasás/írás
- node discover
- test spin
- node assign
- diag stream megjelenítés
