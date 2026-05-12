# STM32G Motor Node Handoff

Ez a dokumentum a `firmware/stm32g-node` fejlesztési irányát foglalja ossze.

Cél:
- kulon workflow-ban lehessen fejleszteni a motor node firmware-t
- egyertelmu legyen, mit var tole jelenleg az ECU
- a node oldali fejlesztes ne keveredjen az ECU oldali `Legacy / SK21` Rate App kompatibilitassal

## 1. Rendszer topologia

A jelenlegi architektura:

- `Rate App -> UDP/Ethernet -> ECU -> CAN -> motor node`

A motor node:
- nem beszel kozvetlenul a Rate App-pal
- az ECU-tol kap CAN runtime parancsokat
- CAN-en kuld vissza allapotot, diagnosztikat, es kesobb seed/blockage adatokat

## 2. A node szerepe

A motor node feladata:

- `GLOBAL_CONTROL` fogadas
- `NODE_CMD` fogadas
- a cel motorfordulat kovetese
- sajat allapot, rpm, pozicio kuldese
- reszletes diag kuldese
- service/provisioning parancsok kezelese
- kesobb magszenzor / blockage logika futtatasa

Nem feladata:
- rate szamitas a Rate App helyett
- wheel speed support
- switchbox logika
- CAN Bridge / CommMode kezeles
- `Legacy / SK21` app valaszto kezeles

## 3. CAN profilok

4 szenzorprofil van. A node provisioning utan tudja, melyik profil ala tartozik.

### S0

- `GLOBAL_CONTROL = 0x080`
- `NODE_CMD = 0x100 + node_id`
- `NODE_PRESENCE = 0x140 + node_id`
- `NODE_CFG_ACK = 0x160 + node_id`
- `NODE_STATUS_FAST = 0x180 + node_id`
- `NODE_DIAG = 0x1C0 + node_id`

### S1

- `GLOBAL_CONTROL = 0x200`
- `NODE_CMD = 0x210 + node_id`
- `NODE_PRESENCE = 0x240 + node_id`
- `NODE_CFG_ACK = 0x260 + node_id`
- `NODE_STATUS_FAST = 0x280 + node_id`
- `NODE_DIAG = 0x2C0 + node_id`

### S2

- `GLOBAL_CONTROL = 0x300`
- `NODE_CMD = 0x310 + node_id`
- `NODE_PRESENCE = 0x340 + node_id`
- `NODE_CFG_ACK = 0x360 + node_id`
- `NODE_STATUS_FAST = 0x380 + node_id`
- `NODE_DIAG = 0x3C0 + node_id`

### S3

- `GLOBAL_CONTROL = 0x400`
- `NODE_CMD = 0x410 + node_id`
- `NODE_PRESENCE = 0x440 + node_id`
- `NODE_CFG_ACK = 0x460 + node_id`
- `NODE_STATUS_FAST = 0x480 + node_id`
- `NODE_DIAG = 0x4C0 + node_id`

## 4. Runtime uzenetek

### 4.1 `GLOBAL_CONTROL`

Fo mezo:

- `system_mode`
- `control_flags`
- `base_rpm`
- `sync_pos`
- `sequence`

`control_flags` bitek:

- bit0: `drive_enable`
- bit1: `sync_enable`
- bit2: `estop`
- bit3: `diag_enable`

Fontos:
- a `base_rpm` mar nem a regi `int16 * 0.1`
- hanem `uint16`, `1 rpm` felbontas
- vart tartomany akar `0..10000 rpm`

Ez motor oldali fordulat.

### 4.2 `NODE_CMD`

Fo mezo:

- `node_command`
- `node_flags`
- `trim_rpm`
- `section_mask`
- `sequence`

Fontos:
- a `4-5.` byte mar `section_mask`
- nem pozicio offset

A node feladata:
- eldonteni, hogy a sajat sora aktiv-e a `section_mask` alapjan
- a `base_rpm + trim_rpm` alapjan cel motorfordulatot kepezni
- ha a sor nem aktiv, ne hajtson

### 4.3 `NODE_STATUS_FAST`

Ezt a node kuldi az ECU fele.

Tartalom:

- `status_flags`
- `error_code`
- `actual_rpm`
- `actual_pos`
- `alive_counter`
- `sync_error`

Fontos:
- a visszajelzett `actual_rpm` motor oldali fordulat legyen
- nem tarcsaoldali

### 4.4 `NODE_DIAG`

Részletes node diagnosztika:

- `bus_voltage`
- `motor_current`
- `controller_temp`
- `motor_temp`
- `fault_flags`
- `warning_flags`

Elvart viselkedes:
- ha `GLOBAL_CONTROL.ctrl_diag_enable = 0`, ne kuldjon periodikus diagot
- ha `GLOBAL_CONTROL.ctrl_diag_enable = 1`, kuldhet periodikus diagot

### 4.5 `NODE_PRESENCE`

Ez mar nem csak jelenletjelzo, hanem seed/blockage kiegeszito adatcsatorna is.

Jelenlegi vart payload:

- byte0 `seed_flags`
- byte1 `blockage_pct`
- byte2 `slowdown_pct`
- byte3 `skip_pct`
- byte4 `double_pct`
- byte5 `singulation_pct`
- byte6-7 `population_x1k_u16`

`seed_flags` bitek:

- bit0 `seed_valid`
- bit1 `seed_blocked`
- bit2 `seed_slowed`
- bit3 `seed_sensor_fault`

Ez a kesobbi planter / blockage monitor integracio alapja.

## 5. Mechanikai modell

Fontos fogalmi pont:

- a node motor oldali rpm-et kap celkent
- a node motor oldali rpm-et kuld vissza

Az ECU oldali logika:

- `disc_rpm` -> tarcsa fordulat
- `motor_rpm = disc_rpm * drive_ratio * motor_ratio`

Ezert a node:
- mindig motor oldali fordulatban gondolkodjon
- a sajat visszajelzese is motor oldali legyen

## 6. Elvart node viselkedes

Minimum elvart funkcionalitas:

1. `GLOBAL_CONTROL` fogadas
2. `NODE_CMD` fogadas
3. sajat section bit figyelese
4. `target_motor_rpm = base_rpm + trim_rpm`
5. leallas, ha section off / drive off / estop
6. `NODE_STATUS_FAST` kuldes
7. `NODE_DIAG` kuldes `diag_enable` alapjan
8. `NODE_PRESENCE` kuldes seed/blockage adatokkal

## 7. Service CAN retegek

Kulon service/provisioning ID tartomany van:

- `0x500..0x50E`

Parancsok:

- `SRV_DISCOVER`
- `SRV_ASSIGN`
- `SRV_SAVE_CFG`
- `SRV_TEST_SPIN`
- `SRV_DIAG_REQ`
- `SRV_REBOOT`
- `SRV_IDENTIFY`
- `SRV_CFG_READ`
- `SRV_SET_CAN_SOURCE`

Valaszok:

- `SRV_UID_A`
- `SRV_UID_B`
- `SRV_ACK`
- `SRV_DIAG_RESP_A`
- `SRV_DIAG_RESP_B`
- `SRV_CFG_RESP`

Minimum node oldali service feladatok:

- sajat UID visszaadasa
- `sensor_channel`, `node_id`, `can_profile` eltárolása
- mentes flash-be
- test spin
- identify

## 8. Seed / blockage fejlesztes iranya

Hosszabb tavon a magszenzoros / blockage logika a motor node-ba kerul.

Miert jo ez:

- a node van legkozelebb a sorhoz
- lokalisan tud magszenzort kezelni
- az ECU mar most is tudja fogadni az extra adatokat a `NODE_PRESENCE` frame-ben

Varhato jovobeli node oldali adatok:

- `blockage_pct`
- `slowdown_pct`
- `skip_pct`
- `double_pct`
- `singulation_pct`
- `population_x1k`

Jelenleg ezeket a motor-szimulator mar tudja utanozni, ez jo referencia.

## 9. Mi nincs keszen a node oldalon

Ezeket kell fejleszteni:

- runtime CAN parser a fenti formatumokhoz
- `GLOBAL_CONTROL.base_rpm` uj formatumanak kezelese
- `NODE_CMD.section_mask` helyes ertelmezese
- `NODE_STATUS_FAST` kuldes az ECU vart formatumban
- `NODE_DIAG` periodikus kuldes `diag_enable` szerint
- `NODE_PRESENCE` seed/blockage payload kuldes
- service/provisioning parancsok implementalasa
- config mentes

## 10. Mihez ne nyuljon a node workflow

Nem node feladat:

- Rate App `Legacy / SK21` kompatibilitas
- `32400 / 32401 / 32700` UDP oldali kompatibilitas
- ECU service tool UI
- planter / blockage PC-s megjelenito appok

Ez ECU oldali tema.

## 11. Javasolt fejlesztesi sorrend

1. `GLOBAL_CONTROL` fogadas
2. `NODE_CMD` fogadas
3. motor celrpm kovetes
4. `NODE_STATUS_FAST`
5. `NODE_DIAG`
6. `NODE_PRESENCE`
7. service CAN provisioning
8. flash save/load
9. valodi seed / blockage szenzor logika

## 12. Fontos referencia

A legjobb aktualis referencia:

- ECU firmware CAN matrix
- motor simulator viselkedese
- ECU altal jelenleg vart frame-strukturak

A motor-szimulator kulonosen fontos, mert jol mutatja, mit var el az ECU a node-tol.
