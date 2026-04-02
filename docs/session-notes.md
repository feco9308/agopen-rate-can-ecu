# Vetogep ECU Session Notes

## Aktualis allapot

- Repo: `agopen-rate-can-ecu`
- Fokusz: `firmware/teensy-ecu/ecu-v1/rate_ecu`
- Kulso tool: `tools/pyside-ecu-service/ecu_service_tool.py`

## Mi keszult el

### 1. Multi-szenzor ECU

- Az ECU most mar 3 szenzorcsatornat tud kezelni.
- A csatornak kulon CAN profillal mennek:
  - `sensor 0` -> `0x080 / 0x100 / 0x180 ...`
  - `sensor 1` -> `0x200 / 0x210 / 0x280 ...`
  - `sensor 2` -> `0x300 / 0x310 / 0x380 ...`

### 2. Vetogepes UPM -> RPM szamitas

- A jelenlegi keplet:

```text
totalUPM = target_upm * upmScale
rowUPM   = totalUPM / activeSections
discRPM  = rowUPM / holes
motorRPM = discRPM * gearRatio
```

- A visszajelzett `actualUpm` mar a visszajovo motor RPM-ekbol szamolodik.
- A motor pozicio atlagolasa koratlaggal megy.

### 3. Trim RPM

- A `NODE_CMD.trim_rpm_x10` most mar elojeles `int16`.
- A trim poziciohibabol szamolodik:
  - cel pozicio = `syncAxis`
  - aktualis pozicio = node `actual_pos`
- A node figyelmen kivul hagyhatja, de debugban mar latszik.

### 4. Runtime config az ECU-ban

- Nem csak fix `constexpr` ertekek vannak.
- EEPROM-os runtime config kerult be:
  - aktiv szenzorszam
  - rows
  - holes
  - gear ratio
  - upm scale
  - trim limit
  - position kp
  - diag enable
  - diag stream
  - diag period
  - module id
  - ECU IP utolso oktett

### 5. Custom UDP PGN-ek

- ECU sajat UDP PGN-ek:
  - `32800` `ECU_CFG_GET`
  - `32801` `ECU_CFG_SET`
  - `32802` `ECU_CFG_SAVE`
  - `32803` `ECU_CFG_LOAD`
  - `32804` `ECU_CFG_STATUS`
  - `32805` `ECU_DIAG_CONTROL`
  - `32806` `ECU_DIAG_STATUS`
  - `32807` `ECU_DIAG_SENSOR`
  - `32808` `ECU_DIAG_NODE_SUMMARY`

### 6. CAN service/provisioning reteget

- Kulon service CAN ID-k kerultek be:
  - `0x500` `DISCOVER`
  - `0x501` `UID_A`
  - `0x502` `UID_B`
  - `0x503` `ASSIGN`
  - `0x504` `SAVE_CFG`
  - `0x505` `ACK`
  - `0x506` `TEST_SPIN`
  - `0x507` `DIAG_REQ`
  - `0x508` `DIAG_RESP_A`
  - `0x509` `DIAG_RESP_B`
  - `0x50A` `REBOOT`
  - `0x50B` `IDENTIFY`
  - `0x50C` `CFG_READ`
  - `0x50D` `CFG_RESP`
  - `0x50E` `SET_CAN_SOURCE`

- Az ECU ezeket mar ki tudja kuldeni.
- A node service valaszok az ECU soros debugjaban mar latszhatnak.

### 7. PySide6 tool

- Helye:
  - `tools/pyside-ecu-service/ecu_service_tool.py`
- Tud:
  - ECU config get/set/save/load
  - diag control
  - ECU diag megjelenites
  - node service parancsok kuldese

## Halozati portok

### Legacy Rate App

- ECU receive:
  - `28888`
- ECU valasz a Rate App fele:
  - `29999`

### Custom service tool

- Tool kuld az ECU fele:
  - `28888`
- ECU custom service valasz a tool fele:
  - `30001`

## Jelenlegi IP-k

- ECU:
  - `192.168.1.200`
- PC:
  - `192.168.1.6`

## Fontos javitasok a toolban

- A tool mar nem all le, ha a fogadoport foglalt.
- A custom valaszok kulon portra mentek at: `30001`
- Az ECU mar unicastban valaszol a tool IP-jere.
- A PySide UDP fogadasnal volt egy `bytes`/CRC hiba, ez javitva lett.

## Mi latszott a soros logban

- Az ECU mar tenylegesen kuldi a custom PGN valaszokat a PC-re:

```text
[ETH TX CUSTOM] PGN 32808 -> 192.168.1.6:30001
[ETH TX CUSTOM] PGN 32806 -> 192.168.1.6:30001
[ETH TX CUSTOM] PGN 32807 -> 192.168.1.6:30001
```

Ez azt jelenti, hogy:
- a firmware oldali custom TX mar mukodik
- a tool fogadasat kell nezni, ha valami meg mindig hianyzik

## Mi lehet a kovetkezo lepes

### Opcio 1

- Megnezni, hogy a PySide tool most mar tenylegesen kirajzolja-e a bejovo `32806/32807/32808` PGN-eket.

### Opcio 2

- Node firmware oldalon implementalni a service CAN fogadast:
  - `DISCOVER`
  - `UID_A`
  - `UID_B`
  - `ASSIGN`
  - `SAVE_CFG`
  - `TEST_SPIN`
  - `CFG_READ`
  - `SET_CAN_SOURCE`
  - `ACK`

### Opcio 3

- A PySide toolt tovabb boviteni:
  - node lista tabla
  - ACK/UID megjelenites
  - jobb diag tablazat

## Hasznos parancsok

### Firmware build

```powershell
C:\Users\User2\.platformio\penv\Scripts\platformio.exe run
```

### Python tool inditas

```powershell
python tools\pyside-ecu-service\ecu_service_tool.py
```

### Python syntax check

```powershell
python -m py_compile tools\pyside-ecu-service\ecu_service_tool.py
```

## Javasolt commit

```powershell
git add docs\session-notes.md firmware\teensy-ecu\ecu-v1\rate_ecu\src\can_bus.cpp firmware\teensy-ecu\ecu-v1\rate_ecu\src\can_bus.h firmware\teensy-ecu\ecu-v1\rate_ecu\src\config.h firmware\teensy-ecu\ecu-v1\rate_ecu\src\custom_pgn_protocol.h firmware\teensy-ecu\ecu-v1\rate_ecu\src\ethernet_link.cpp firmware\teensy-ecu\ecu-v1\rate_ecu\src\ethernet_link.h firmware\teensy-ecu\ecu-v1\rate_ecu\src\main.cpp firmware\teensy-ecu\ecu-v1\rate_ecu\src\node_manager.cpp firmware\teensy-ecu\ecu-v1\rate_ecu\src\rate_math.h firmware\teensy-ecu\ecu-v1\rate_ecu\src\runtime_config.cpp firmware\teensy-ecu\ecu-v1\rate_ecu\src\runtime_config.h firmware\teensy-ecu\ecu-v1\rate_ecu\src\service_can_protocol.h tools\pyside-ecu-service\ecu_service_tool.py tools\pyside-ecu-service\README.md
git commit -m "Add ECU service PGNs, CAN provisioning, and PySide tool"
```
