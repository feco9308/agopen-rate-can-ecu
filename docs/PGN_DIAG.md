# Vetőgép ECU Saját PGN Tábla
## Implementálható V1 javaslat

Ez a tábla a jelenlegi rendszerhez készült:

- Rate App -> ECU UDP/PGN megmarad
- ECU -> CAN motorvezérlés megmarad
- ehhez jön egy saját szerviz/config/diag PGN készlet
- cél:
  - szenzorszám állítás
  - szektorszám / sorszám állítás
  - gép paraméterek állítás
  - motor node felderítés
  - STM UID lekérés
  - motor forgatás teszt
  - node hozzárendelés `sensor_channel + node_id`
  - mentés flash-be
  - diagnosztikai stream

---

## Alapelvek

- `32800..32899` = ECU config / ECU diag
- `32900..32999` = Node provisioning / node diag
- minden PGN 8 byte payloados CAN frame
- ahol több adat kell, ott több PGN vagy több blokk használható
- a `sensor_channel` mindig logikai csatorna:
  - `0`
  - `1`
  - `2`
- a node nem nyers CAN ID-t kap, hanem:
  - melyik csatornához tartozik
  - milyen `node_id`
- ebből a node maga számolja a figyelt CAN ID-ket

---

## Skálázások

| Név | Kódolt típus | Skála | Példa |
|---|---|---|---|
| `rpm_x10` | `int16` | `value / 10` | `859` = `85.9 rpm` |
| `ratio_x100` | `uint16` | `value / 100` | `200` = `2.00` |
| `scale_x10` | `uint16` | `value / 10` | `1000` = `100.0` |
| `kp_x1000` | `uint16` | `value / 1000` | `500` = `0.500` |
| `upm_x1000` | `uint32` | `value / 1000` | `67000` = `67.000` |
| `time_div10_ms` | `uint8` | `value * 10 ms` | `20` = `200 ms` |

---

## 1. ECU Config / ECU Diag PGN-ek

| PGN | Név | Irány | Funkció |
|---|---|---|---|
| 32800 | `ECU_CFG_GET` | Tool/App -> ECU | config blokk lekérése |
| 32801 | `ECU_CFG_SET` | Tool/App -> ECU | config blokk írása |
| 32802 | `ECU_CFG_SAVE` | Tool/App -> ECU | config mentése flash-be |
| 32803 | `ECU_CFG_LOAD` | Tool/App -> ECU | config visszatöltése flash-ből |
| 32804 | `ECU_CFG_STATUS` | ECU -> Tool/App | config blokk válasz |
| 32805 | `ECU_DIAG_CONTROL` | Tool/App -> ECU | diagnosztikai stream vezérlés |
| 32806 | `ECU_DIAG_STATUS` | ECU -> Tool/App | összesített ECU állapot |
| 32807 | `ECU_DIAG_SENSOR` | ECU -> Tool/App | szenzorcsatorna állapot |
| 32808 | `ECU_DIAG_NODE_SUMMARY` | ECU -> Tool/App | csatorna összesített motor állapot |

---

## 32800 - ECU_CFG_GET

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `block_id` | `uint8` | melyik blokkot kérjük |
| 1 | `index` | `uint8` | szenzor index vagy 0 |
| 2..7 | `reserved` | - | 0 |

### `block_id` értékek

| Érték | Blokk |
|---|---|
| 0 | globális gépconfig |
| 1 | számítási config |
| 2 | diag config |
| 3 | csatorna config |
| 4 | hálózati config |

### Megjegyzés
- `index` mező csak olyan blokkoknál számít, ahol csatornafüggő adat van
- globális blokknál `index = 0`

---

## 32801 - ECU_CFG_SET

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `block_id` | `uint8` | melyik blokkot írjuk |
| 1 | `index` | `uint8` | csatorna index vagy 0 |
| 2..7 | `data` | blokkfüggő | blokk tartalom |

---

## 32801 / blokk 0 - Globális gépconfig

### `block_id = 0`
### `index = 0`

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2 | `active_sensor_count` | `uint8` | 1..3 |
| 3 | `configured_row_count` | `uint8` | sorok/szektorok száma |
| 4..5 | `holes_per_rev` | `uint16` | furatok száma tárcsán |
| 6..7 | `upm_scale_x10` | `uint16` | ECU oldali UPM skála |

### Példa
- `active_sensor_count = 3`
- `configured_row_count = 6`
- `holes_per_rev = 26`
- `upm_scale = 100.0`

Kódolva:
- byte2 = `3`
- byte3 = `6`
- byte4..5 = `26`
- byte6..7 = `1000`

---

## 32801 / blokk 1 - Számítási config

### `block_id = 1`
### `index = 0`

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2..3 | `gear_ratio_x100` | `uint16` | motor/tárcsa arány |
| 4..5 | `trim_limit_rpm_x10` | `int16` | trim korlát |
| 6..7 | `position_kp_x1000` | `uint16` | pozíció P erősítés |

### Példa
- `gear_ratio = 2.00`
- `trim_limit = 200.0 rpm`
- `position_kp = 0.500`

Kódolva:
- `200`
- `2000`
- `500`

---

## 32801 / blokk 2 - Diag config

### `block_id = 2`
### `index = 0`

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2 | `diag_enable` | `uint8` | 0/1 |
| 3 | `diag_stream_enable` | `uint8` | 0/1 |
| 4 | `diag_period_div10_ms` | `uint8` | pl. `20` = 200 ms |
| 5 | `diag_detail_level` | `uint8` | 0=basic, 1=extended |
| 6 | `reserved` | `uint8` | 0 |
| 7 | `reserved` | `uint8` | 0 |

---

## 32801 / blokk 3 - Csatorna config

### `block_id = 3`
### `index = sensor_channel`

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2 | `channel_enable` | `uint8` | 0/1 |
| 3 | `can_profile_index` | `uint8` | 0/1/2 |
| 4 | `status_bit_mode` | `uint8` | app kompatibilitás |
| 5 | `reserved` | `uint8` | 0 |
| 6..7 | `reserved` | `uint16` | 0 |

### Megjegyzés
- `can_profile_index` jelenleg lehet azonos a `sensor_channel` értékkel
- később ezzel elválasztható a logikai szenzor és a tényleges CAN bank

---

## 32801 / blokk 4 - Hálózati config

### `block_id = 4`
### `index = 0`

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2 | `ip_last_octet` | `uint8` | ECU IP utolsó oktett |
| 3 | `module_id` | `uint8` | Rate App modul ID |
| 4 | `reserved` | `uint8` | 0 |
| 5 | `reserved` | `uint8` | 0 |
| 6..7 | `reserved` | `uint16` | 0 |

---

## 32802 - ECU_CFG_SAVE

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `magic` | `uint8` | `0xA5` |
| 1 | `scope` | `uint8` | 0=all, 1=global only |
| 2..7 | `reserved` | - | 0 |

### Funkció
- aktuális ECU config mentése flash-be / EEPROM-ba

---

## 32803 - ECU_CFG_LOAD

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `magic` | `uint8` | `0x5A` |
| 1 | `scope` | `uint8` | 0=all |
| 2..7 | `reserved` | - | 0 |

### Funkció
- korábban mentett config visszatöltése

---

## 32804 - ECU_CFG_STATUS

### DLC
- `8`

### Funkció
- válasz ugyanabban a blokkstruktúrában, mint a `32801 ECU_CFG_SET`
- a `block_id` és `index` mező ugyanúgy szerepel benne

### Formátum
| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `block_id` | `uint8` | visszaküldött blokk |
| 1 | `index` | `uint8` | blokk index |
| 2..7 | `data` | blokkfüggő | aktuális érték |

---

## 32805 - ECU_DIAG_CONTROL

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `diag_enable` | `uint8` | 0/1 |
| 1 | `stream_enable` | `uint8` | 0/1 |
| 2 | `sensor_mask` | `uint8` | bit0=s0, bit1=s1, bit2=s2 |
| 3 | `node_mask_lo` | `uint8` | node 1..8 |
| 4 | `node_mask_hi` | `uint8` | node 9..16 |
| 5 | `period_div10_ms` | `uint8` | pl. 10 = 100 ms |
| 6 | `detail_level` | `uint8` | 0=basic, 1=extended |
| 7 | `reserved` | `uint8` | 0 |

### Funkció
- ezzel lehet ki-be kapcsolni a motor diag streamet
- szűrhető, melyik szenzorok és mely node-ok jelenjenek meg

---

## 32806 - ECU_DIAG_STATUS

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `active_sensor_count` | `uint8` | aktív csatornák száma |
| 1 | `configured_row_count` | `uint8` | sorok száma |
| 2 | `eth_link` | `uint8` | 0/1 |
| 3 | `can_link` | `uint8` | 0/1 |
| 4 | `diag_enable` | `uint8` | 0/1 |
| 5 | `rc_timeout` | `uint8` | 0/1 |
| 6..7 | `uptime_s` | `uint16` | üzemidő másodpercben |

---

## 32807 - ECU_DIAG_SENSOR

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `sensor_channel` | `uint8` | 0..2 |
| 1 | `mode` | `uint8` | off/manual/auto |
| 2..3 | `base_rpm_x10` | `int16` | cél motor rpm |
| 4..7 | `target_upm_x1000` | `uint32` | appból jövő target UPM |

### Funkció
- csatornánként látni a bejövő és számolt alap értékeket

---

## 32808 - ECU_DIAG_NODE_SUMMARY

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `sensor_channel` | `uint8` | 0..2 |
| 1 | `online_nodes` | `uint8` | online node db |
| 2..3 | `avg_rpm_x10` | `int16` | átlagos motor rpm |
| 4..5 | `total_rpm_x10` | `int16` | összegzett motor rpm |
| 6..7 | `avg_pos_u16` | `uint16` | átlagos motor pozíció |

---

# 2. Node Provisioning / Node Diag PGN-ek

| PGN | Név | Irány | Funkció |
|---|---|---|---|
| 32900 | `NODE_DISCOVER` | Tool/App -> broadcast | node keresés |
| 32901 | `NODE_UID_REPORT_A` | Node -> Tool/App | UID első rész |
| 32902 | `NODE_UID_REPORT_B` | Node -> Tool/App | UID második rész |
| 32903 | `NODE_ASSIGN` | Tool/App -> Node | csatorna + node ID kiosztás |
| 32904 | `NODE_CFG_READ` | Tool/App -> Node | aktuális node config lekérés |
| 32905 | `NODE_CFG_STATUS` | Node -> Tool/App | node config válasz |
| 32906 | `NODE_CFG_SAVE` | Tool/App -> Node | node config mentés |
| 32907 | `NODE_TEST_SPIN` | Tool/App -> Node | motor forgatás teszt |
| 32908 | `NODE_DIAG_STATUS` | Node -> Tool/App | alap node diag |
| 32909 | `NODE_DIAG_EXT` | Node -> Tool/App | bővített diag |
| 32910 | `NODE_REBOOT` | Tool/App -> Node | újraindítás |
| 32911 | `NODE_BLINK_IDENT` | Tool/App -> Node | azonosítás LED/mozgatás |
| 32912 | `NODE_CMD_SOURCE_CFG` | Tool/App -> Node | mely CAN bankot figyelje |

---

## 32900 - NODE_DISCOVER

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `request_type` | `uint8` | `1=discover` |
| 1 | `response_delay_ms_div10` | `uint8` | válasz szórás |
| 2..7 | `reserved` | - | 0 |

### Funkció
- minden node válaszol saját UID riporttal

---

## 32901 - NODE_UID_REPORT_A

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `report_type` | `uint8` | `1` |
| 1..4 | `uid_0_31` | `uint32` | STM UID alsó 32 bit |
| 5 | `sensor_channel` | `uint8` | jelenlegi csatorna |
| 6 | `node_id` | `uint8` | jelenlegi node ID |
| 7 | `fw_major` | `uint8` | fw major |

---

## 32902 - NODE_UID_REPORT_B

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `report_type` | `uint8` | `2` |
| 1..4 | `uid_32_63` | `uint32` | STM UID következő 32 bit |
| 5 | `fw_minor` | `uint8` | fw minor |
| 6 | `hw_rev` | `uint8` | hardware rev |
| 7 | `cap_flags` | `uint8` | képességek bitmask |

### `cap_flags` javaslat

| Bit | Jelentés |
|---|---|
| 0 | támogat test spin-t |
| 1 | támogat config mentést |
| 2 | támogat diag ext-et |
| 3 | van helyi szenzor |
| 4..7 | tartalék |

---

## 32903 - NODE_ASSIGN

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_0_31` | `uint32` | cél node UID alsó rész |
| 4 | `sensor_channel` | `uint8` | 0..2 |
| 5 | `node_id` | `uint8` | 1..16 |
| 6 | `save_now` | `uint8` | 0/1 |
| 7 | `flags` | `uint8` | opciók |

### `flags` javaslat

| Bit | Jelentés |
|---|---|
| 0 | apply immediately |
| 1 | reboot after apply |
| 2..7 | tartalék |

### Funkció
- a node ebből állítja be:
  - melyik csatornához tartozik
  - melyik sor/node ő
- ebből maga számolja a figyelt CAN ID-ket

---

## 32904 - NODE_CFG_READ

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_0_31` | `uint32` | cél node UID |
| 4 | `block_id` | `uint8` | melyik blokk |
| 5..7 | `reserved` | - | 0 |

### `block_id` javaslat

| Érték | Blokk |
|---|---|
| 0 | identitás |
| 1 | CAN source config |
| 2 | helyi motor config |
| 3 | diag config |

---

## 32905 - NODE_CFG_STATUS

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `block_id` | `uint8` | visszaküldött blokk |
| 1 | `node_id` | `uint8` | aktuális node id |
| 2..7 | `data` | blokkfüggő | blokk tartalom |

---

## 32905 / blokk 0 - Identitás

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2 | `sensor_channel` | `uint8` | aktuális csatorna |
| 3 | `node_id_cfg` | `uint8` | aktuális node ID |
| 4 | `fw_major` | `uint8` | fw major |
| 5 | `fw_minor` | `uint8` | fw minor |
| 6 | `hw_rev` | `uint8` | hw rev |
| 7 | `flags` | `uint8` | capability/status |

---

## 32905 / blokk 1 - CAN source config

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2 | `sensor_channel` | `uint8` | 0..2 |
| 3 | `node_id_cfg` | `uint8` | 1..16 |
| 4 | `can_profile_index` | `uint8` | 0..2 |
| 5 | `listen_enable` | `uint8` | 0/1 |
| 6..7 | `reserved` | - | 0 |

---

## 32905 / blokk 2 - Helyi motor config

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2..3 | `local_trim_limit_rpm_x10` | `int16` | helyi trim limit |
| 4..5 | `rpm_limit_x10` | `uint16` | helyi rpm limit |
| 6 | `invert_dir` | `uint8` | 0/1 |
| 7 | `use_local_sensor` | `uint8` | 0/1 |

---

## 32906 - NODE_CFG_SAVE

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_0_31` | `uint32` | cél node UID |
| 4 | `magic` | `uint8` | `0xA5` |
| 5..7 | `reserved` | - | 0 |

---

## 32907 - NODE_TEST_SPIN

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_0_31` | `uint32` | cél node UID |
| 4..5 | `test_rpm_x10` | `int16` | teszt fordulat előjelesen |
| 6 | `duration_s` | `uint8` | időtartam másodperc |
| 7 | `mode` | `uint8` | 0=run, 1=brake, 2=coast |

### Megjegyzés
- ez szervizhez nagyon hasznos
- egyértelműen kiderül, melyik motor a kiválasztott node

---

## 32908 - NODE_DIAG_STATUS

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `sensor_channel` | `uint8` | 0..2 |
| 1 | `node_id` | `uint8` | 1..16 |
| 2..3 | `actual_rpm_x10` | `int16` | aktuális motor rpm |
| 4..5 | `trim_rpm_x10` | `int16` | aktuális trim |
| 6 | `error_code` | `uint8` | node hiba |
| 7 | `status_flags` | `uint8` | online/run/fault stb |

### `status_flags` javaslat

| Bit | Jelentés |
|---|---|
| 0 | online |
| 1 | run enabled |
| 2 | fault active |
| 3 | local sensor valid |
| 4 | synced |
| 5..7 | tartalék |

---

## 32909 - NODE_DIAG_EXT

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `sensor_channel` | `uint8` | 0..2 |
| 1 | `node_id` | `uint8` | 1..16 |
| 2..3 | `actual_pos_u16` | `uint16` | motor pozíció |
| 4 | `sync_error_x256rev` | `int8` | szinkron hiba |
| 5 | `alive_counter` | `uint8` | életjel számláló |
| 6..7 | `target_base_rpm_x10` | `int16` | ECU cél rpm |

### Funkció
- ez már szervizre és webes diagnosztikára is nagyon jó

---

## 32910 - NODE_REBOOT

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_0_31` | `uint32` | cél node UID |
| 4 | `magic` | `uint8` | `0x5A` |
| 5..7 | `reserved` | - | 0 |

---

## 32911 - NODE_BLINK_IDENT

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_0_31` | `uint32` | cél node UID |
| 4 | `duration_s` | `uint8` | villogás / mozgatás idő |
| 5 | `mode` | `uint8` | 0=LED, 1=short jog |
| 6..7 | `reserved` | - | 0 |

### Funkció
- gyors azonosítás szereléskor

---

## 32912 - NODE_CMD_SOURCE_CFG

### DLC
- `8`

### Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid_0_31` | `uint32` | cél node UID |
| 4 | `sensor_channel` | `uint8` | melyik csatorna |
| 5 | `node_id` | `uint8` | melyik sor/node |
| 6 | `can_profile_index` | `uint8` | melyik CAN bankot figyelje |
| 7 | `save_now` | `uint8` | 0/1 |

### Funkció
- külön is állítható, melyik üzenetbankot figyelje
- akkor hasznos, ha a logikai kiosztás és a fizikai CAN bank szétválik

---

# CAN bank számítási szabály

A node a `sensor_channel` vagy `can_profile_index` alapján választ profilt.

## Profil 0
- `GLOBAL_CONTROL = 0x080`
- `NODE_CMD_BASE = 0x100`
- `NODE_STATUS_BASE = 0x180`

## Profil 1
- `GLOBAL_CONTROL = 0x200`
- `NODE_CMD_BASE = 0x210`
- `NODE_STATUS_BASE = 0x280`

## Profil 2
- `GLOBAL_CONTROL = 0x300`
- `NODE_CMD_BASE = 0x310`
- `NODE_STATUS_BASE = 0x380`

## Példa
Ha:
- `sensor_channel = 1`
- `node_id = 4`

akkor:
- `GLOBAL_CONTROL = 0x200`
- `NODE_CMD = 0x214`
- `NODE_STATUS = 0x284`

---

# Javasolt szerviz workflow

## Új node felvétele

1. `NODE_DISCOVER`
2. node válaszol:
   - `NODE_UID_REPORT_A`
   - `NODE_UID_REPORT_B`
3. `NODE_TEST_SPIN`
4. megnézed, hogy tényleg a jó motor mozdult
5. `NODE_ASSIGN`
6. opcionálisan `NODE_CMD_SOURCE_CFG`
7. `NODE_CFG_SAVE`

## ECU config állítás

1. `ECU_CFG_GET`
2. `ECU_CFG_SET`
3. `ECU_CFG_STATUS`
4. `ECU_CFG_SAVE`

## Motor diag

1. `ECU_DIAG_CONTROL`
2. ECU küldi:
   - `ECU_DIAG_SENSOR`
   - `ECU_DIAG_NODE_SUMMARY`
3. node küldi:
   - `NODE_DIAG_STATUS`
   - `NODE_DIAG_EXT`

---

# Javasolt web UI oldalak ehhez

## 1. ECU Config
- active sensor count
- configured row count
- holes per rev
- gear ratio
- upm scale
- trim limit
- position kp
- save/load

## 2. Node Provisioning
- discover
- UID lista
- test spin
- sensor channel kiválasztás
- node ID kiválasztás
- save

## 3. Motor Diag
- sensor selector
- node1..nodeN tábla
- online
- target rpm
- actual rpm
- trim rpm
- effective rpm
- actual pos
- sync error
- error code
- alive counter

---

# Megjegyzések

- A `trim_rpm_x10` mindenhol legyen előjeles `int16`
- A `test_rpm_x10` is legyen előjeles `int16`
- A node oldali UID-hez minimum 64 bit ajánlott
- Ha a teljes STM UID 96 bites, érdemes 2 vagy 3 frame-re bontani
- A diag stream kapcsolható legyen, hogy ne terhelje feleslegesen a buszt
- A `sensor_channel` és `node_id` logikai kiosztás jobb, mint a nyers CAN ID írás

# Javasolt első implementációs sorrend

1. `ECU_CFG_GET / SET / SAVE / STATUS`
2. `NODE_DISCOVER / UID_REPORT / ASSIGN / SAVE`
3. `NODE_TEST_SPIN`
4. `NODE_DIAG_STATUS / NODE_DIAG_EXT`
5. web UI
