# Vetőgép ECU CAN Szerviz PGN / CAN Tábla
## Javasolt V1 service/provisioning üzenetek

Ez a tábla a normál vezérlő CAN forgalomtól külön él.

## Cél
- node felderítés
- UID lekérés
- motor azonosítás
- `sensor_channel` és `node_id` kiosztás
- mentés flash-be
- diagnosztikai lekérés
- egyszeri szervizparancsok

## Fontos elv
- ezek az üzenetek nem periodikusak
- parancs/válasz jellegűek
- minden fontos parancsra legyen `ACK`
- a normál `GLOBAL_CONTROL` / `NODE_CMD` forgalom maradjon külön

---

# CAN ID tartomány

## Javasolt service CAN tartomány
- `0x500..0x52F`

## Miért jó ez
- jól elkülönül a mostani:
  - `0x080/0x100/0x180`
  - `0x200/0x210/0x280`
  - `0x300/0x310/0x380`
- könnyen szűrhető PCAN-ben
- nem keveredik a normál vezérléssel

---

# Szerepkörök

| Szereplő | Leírás |
|---|---|
| `ECU` | központi vezérlő vagy szerviz master |
| `Node` | motor modul |
| `Tool` | PC / web UI / setup tool |

Megjegyzés:
- a service busz mastere lehet az ECU vagy egy külső tool
- a node ugyanazokat a service üzeneteket érti bármelyik mestertől

---

# Üzenetlista

| CAN ID | Név | Irány | Funkció |
|---|---|---|---|
| 0x500 | `SRV_DISCOVER` | ECU/Tool -> broadcast | node keresés |
| 0x501 | `SRV_UID_A` | Node -> ECU/Tool | UID első rész |
| 0x502 | `SRV_UID_B` | Node -> ECU/Tool | UID második rész |
| 0x503 | `SRV_ASSIGN` | ECU/Tool -> node | csatorna + node ID kiosztás |
| 0x504 | `SRV_SAVE_CFG` | ECU/Tool -> node | config mentés |
| 0x505 | `SRV_ACK` | Node -> ECU/Tool | visszaigazolás |
| 0x506 | `SRV_TEST_SPIN` | ECU/Tool -> node | motor forgatás teszt |
| 0x507 | `SRV_DIAG_REQ` | ECU/Tool -> node | node diag lekérés |
| 0x508 | `SRV_DIAG_RESP_A` | Node -> ECU/Tool | alap diag |
| 0x509 | `SRV_DIAG_RESP_B` | Node -> ECU/Tool | bővített diag |
| 0x50A | `SRV_REBOOT` | ECU/Tool -> node | újraindítás |
| 0x50B | `SRV_IDENTIFY` | ECU/Tool -> node | LED villogás / rövid mozgatás |
| 0x50C | `SRV_CFG_READ` | ECU/Tool -> node | node config olvasás |
| 0x50D | `SRV_CFG_RESP` | Node -> ECU/Tool | node config válasz |
| 0x50E | `SRV_SET_CAN_SOURCE` | ECU/Tool -> node | melyik CAN bankot figyelje |

---

# Általános címzési szabály

Mivel standard 11 bites CAN ID-t használunk, a címzést a payloadban visszük.

## Célzás mezők
- `uid32`
- `sensor_channel`
- `node_id`

## Miért nem CAN ID-ba tesszük a címet
- a node kezdetben még lehet “ismeretlen”
- UID alapján stabilan címezhető
- egyszerűbb a provisioning

---

# Alap mezőformátumok

| Név | Típus | Skála | Megjegyzés |
|---|---|---|---|
| `uid32` | `uint32` | - | STM UID alsó 32 bit |
| `rpm_x10` | `int16` | `/10` | előjeles rpm |
| `sensor_channel` | `uint8` | - | 0..2 |
| `node_id` | `uint8` | - | 1..16 |
| `duration_s` | `uint8` | másodperc | teszt idő |
| `actual_pos_u16` | `uint16` | 0..65535 | körpozíció |
| `sync_error_x256rev` | `int8` | `/256 rev` | szinkron hiba |

---

# 0x500 - SRV_DISCOVER

## Irány
- `ECU/Tool -> broadcast`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `cmd` | `uint8` | `1 = discover` |
| 1 | `response_delay_slots` | `uint8` | válasz szórás |
| 2 | `request_uid` | `uint8` | 0/1 |
| 3 | `request_cfg` | `uint8` | 0/1 |
| 4..7 | `reserved` | - | 0 |

## Funkció
- minden aktív node válaszol
- ha kell, random késleltetéssel csökkenthető az ütközés

---

# 0x501 - SRV_UID_A

## Irány
- `Node -> ECU/Tool`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | UID alsó 32 bit |
| 4 | `sensor_channel` | `uint8` | aktuális csatorna |
| 5 | `node_id` | `uint8` | aktuális node ID |
| 6 | `fw_major` | `uint8` | fw major |
| 7 | `fw_minor` | `uint8` | fw minor |

---

# 0x502 - SRV_UID_B

## Irány
- `Node -> ECU/Tool`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32_hi` | `uint32` | UID következő 32 bit |
| 4 | `hw_rev` | `uint8` | hardware rev |
| 5 | `cap_flags` | `uint8` | képességek |
| 6 | `status_flags` | `uint8` | alap állapot |
| 7 | `reserved` | `uint8` | 0 |

## `cap_flags`
| Bit | Jelentés |
|---|---|
| 0 | test spin támogatott |
| 1 | config save támogatott |
| 2 | diag ext támogatott |
| 3 | local sensor támogatott |
| 4 | LED identify támogatott |
| 5..7 | tartalék |

---

# 0x503 - SRV_ASSIGN

## Irány
- `ECU/Tool -> node`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | cél node UID |
| 4 | `sensor_channel` | `uint8` | melyik “kocsi” |
| 5 | `node_id` | `uint8` | melyik sor |
| 6 | `apply_now` | `uint8` | 0/1 |
| 7 | `save_now` | `uint8` | 0/1 |

## Funkció
- a node eltárolja:
  - melyik csatornát figyelje
  - melyik `node_id` legyen
- ebből maga számolja a normál runtime CAN ID-ket

---

# 0x504 - SRV_SAVE_CFG

## Irány
- `ECU/Tool -> node`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | cél node UID |
| 4 | `magic` | `uint8` | `0xA5` |
| 5 | `scope` | `uint8` | 0=all |
| 6..7 | `reserved` | - | 0 |

---

# 0x505 - SRV_ACK

## Irány
- `Node -> ECU/Tool`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | válaszoló node UID |
| 4 | `ack_cmd` | `uint8` | melyik parancsra válasz |
| 5 | `result` | `uint8` | eredménykód |
| 6 | `sensor_channel` | `uint8` | aktuális csatorna |
| 7 | `node_id` | `uint8` | aktuális node ID |

## `result`
| Érték | Jelentés |
|---|---|
| 0 | OK |
| 1 | unknown command |
| 2 | bad payload |
| 3 | wrong uid |
| 4 | busy |
| 5 | save failed |
| 6 | not supported |

---

# 0x506 - SRV_TEST_SPIN

## Irány
- `ECU/Tool -> node`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | cél node UID |
| 4..5 | `test_rpm_x10` | `int16` | előjeles teszt rpm |
| 6 | `duration_s` | `uint8` | idő másodpercben |
| 7 | `mode` | `uint8` | 0=run, 1=coast, 2=brake |

## Funkció
- a node röviden megforgatja a motort
- szerviznél gyors azonosításra nagyon hasznos

---

# 0x507 - SRV_DIAG_REQ

## Irány
- `ECU/Tool -> node`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | cél node UID |
| 4 | `detail_level` | `uint8` | 0=basic, 1=extended |
| 5 | `include_runtime` | `uint8` | 0/1 |
| 6 | `include_cfg` | `uint8` | 0/1 |
| 7 | `reserved` | `uint8` | 0 |

---

# 0x508 - SRV_DIAG_RESP_A

## Irány
- `Node -> ECU/Tool`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `sensor_channel` | `uint8` | aktuális csatorna |
| 1 | `node_id` | `uint8` | aktuális node |
| 2..3 | `actual_rpm_x10` | `int16` | aktuális rpm |
| 4..5 | `actual_pos_u16` | `uint16` | aktuális pozíció |
| 6 | `error_code` | `uint8` | hiba |
| 7 | `status_flags` | `uint8` | online/run/fault |

---

# 0x509 - SRV_DIAG_RESP_B

## Irány
- `Node -> ECU/Tool`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | UID alsó rész |
| 4 | `sync_error_x256rev` | `int8` | szinkron hiba |
| 5 | `alive_counter` | `uint8` | alive |
| 6 | `temp_or_supply` | `uint8` | opcionális diag |
| 7 | `reserved` | `uint8` | 0 |

---

# 0x50A - SRV_REBOOT

## Irány
- `ECU/Tool -> node`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | cél UID |
| 4 | `magic` | `uint8` | `0x5A` |
| 5..7 | `reserved` | - | 0 |

---

# 0x50B - SRV_IDENTIFY

## Irány
- `ECU/Tool -> node`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | cél UID |
| 4 | `mode` | `uint8` | 0=LED blink, 1=short jog |
| 5 | `duration_s` | `uint8` | ideig fusson |
| 6..7 | `reserved` | - | 0 |

---

# 0x50C - SRV_CFG_READ

## Irány
- `ECU/Tool -> node`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | cél UID |
| 4 | `block_id` | `uint8` | melyik blokk |
| 5..7 | `reserved` | - | 0 |

## `block_id`
| Érték | Blokk |
|---|---|
| 0 | identitás |
| 1 | CAN source config |
| 2 | local motor config |
| 3 | diag config |

---

# 0x50D - SRV_CFG_RESP

## Irány
- `Node -> ECU/Tool`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0 | `block_id` | `uint8` | visszaküldött blokk |
| 1 | `node_id` | `uint8` | aktuális node ID |
| 2..7 | `data` | blokkfüggő | blokk tartalom |

## blokk 0 - identitás

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2 | `sensor_channel` | `uint8` | aktuális csatorna |
| 3 | `node_id_cfg` | `uint8` | aktuális node ID |
| 4 | `fw_major` | `uint8` | fw major |
| 5 | `fw_minor` | `uint8` | fw minor |
| 6 | `hw_rev` | `uint8` | hw rev |
| 7 | `flags` | `uint8` | capability/status |

## blokk 1 - CAN source config

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2 | `sensor_channel` | `uint8` | melyik csatorna |
| 3 | `node_id_cfg` | `uint8` | melyik node |
| 4 | `can_profile_index` | `uint8` | melyik CAN bank |
| 5 | `listen_enable` | `uint8` | 0/1 |
| 6..7 | `reserved` | - | 0 |

## blokk 2 - local motor config

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 2..3 | `local_trim_limit_rpm_x10` | `int16` | helyi trim limit |
| 4..5 | `rpm_limit_x10` | `uint16` | helyi max rpm |
| 6 | `invert_dir` | `uint8` | 0/1 |
| 7 | `use_local_sensor` | `uint8` | 0/1 |

---

# 0x50E - SRV_SET_CAN_SOURCE

## Irány
- `ECU/Tool -> node`

## DLC
- `8`

## Payload

| Byte | Mező | Típus | Jelentés |
|---|---|---|---|
| 0..3 | `uid32` | `uint32` | cél UID |
| 4 | `sensor_channel` | `uint8` | logikai csatorna |
| 5 | `node_id` | `uint8` | logikai node |
| 6 | `can_profile_index` | `uint8` | fizikai figyelt bank |
| 7 | `save_now` | `uint8` | 0/1 |

## Funkció
- akkor jó, ha külön akarjuk választani:
  - logikai szenzort
  - fizikai figyelt CAN bankot

---

# Normál runtime CAN bank számítás

A node a provisioning után ebből számolja a normál üzemi ID-ket.

## Profil 0
- `GLOBAL_CONTROL = 0x080`
- `NODE_CMD = 0x100 + node_id`
- `NODE_STATUS = 0x180 + node_id`

## Profil 1
- `GLOBAL_CONTROL = 0x200`
- `NODE_CMD = 0x210 + node_id`
- `NODE_STATUS = 0x280 + node_id`

## Profil 2
- `GLOBAL_CONTROL = 0x300`
- `NODE_CMD = 0x310 + node_id`
- `NODE_STATUS = 0x380 + node_id`

---

# Javasolt ACK stratégia

## Parancsok, amikre kötelező ACK
- `SRV_ASSIGN`
- `SRV_SAVE_CFG`
- `SRV_TEST_SPIN`
- `SRV_REBOOT`
- `SRV_SET_CAN_SOURCE`

## Ajánlott retry
- timeout: `100..300 ms`
- retry count: `3`

## Miért kell
- az “egyszer küldjük ki” önmagában nem elég megbízható
- CAN ugyan stabil, de provisioningnál fontos a biztos siker

---

# Javasolt workflow

## Új node felvétele
1. `SRV_DISCOVER`
2. node válaszol `SRV_UID_A`, `SRV_UID_B`
3. `SRV_TEST_SPIN`
4. ha a jó motor mozdult:
5. `SRV_ASSIGN`
6. opcionálisan `SRV_SET_CAN_SOURCE`
7. `SRV_SAVE_CFG`
8. `SRV_ACK`

## Node diagnosztika
1. `SRV_DIAG_REQ`
2. node válaszol:
   - `SRV_DIAG_RESP_A`
   - `SRV_DIAG_RESP_B`

## Azonosítás
1. `SRV_IDENTIFY`
2. node villog vagy röviden mozdul
3. `SRV_ACK`

---

# Megjegyzések

- A `trim_rpm_x10` és `test_rpm_x10` előjeles `int16`
- A provisioning külön service CAN ID-n menjen
- Nem érdemes a normál `NODE_CMD` mezőit tovább zsúfolni
- A node első körben elég, ha az UID alsó 32 bitjével azonosítható
- Ha később kell, bővíthető teljes 96 bites UID kezelésre

# Ajánlott első implementációs sorrend

1. `SRV_DISCOVER`
2. `SRV_UID_A`, `SRV_UID_B`
3. `SRV_ASSIGN`
4. `SRV_ACK`
5. `SRV_TEST_SPIN`
6. `SRV_SAVE_CFG`
7. `SRV_DIAG_REQ`, `SRV_DIAG_RESP_A`, `SRV_DIAG_RESP_B`
