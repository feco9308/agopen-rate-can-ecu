# Motor Node Simulator - PySide

Magyar leírás a `tools/Motor-Simulator/motor_simulator_pyside.py` programhoz.

## Mire való

Ez a program motor node-okat szimulál a CAN buszon.

Fontos:
- ez nem ECU szimulátor
- ez nem Rate App szimulátor
- ez a motor modulok viselkedését utánozza

Röviden:
- az ECU küldi a `GLOBAL_CONTROL` és `NODE_CMD` frame-eket
- a szimulátor ezekre válaszol `NODE_STATUS`, `NODE_DIAG`, `NODE_PRESENCE` frame-ekkel
- emellett service CAN parancsokra is tud reagálni

## Jelenlegi szenzor / CAN bankok

- `Sensor 0` -> `0x080 / 0x100 / 0x180 / 0x1C0`
- `Sensor 1` -> `0x200 / 0x210 / 0x280 / 0x2C0`
- `Sensor 2` -> `0x300 / 0x310 / 0x380 / 0x3C0`
- `Sensor 3` -> `0x400 / 0x410 / 0x480 / 0x4C0`

## Felső sáv

### `Channel`

A használt CAN interfész neve.

Példa:
- `PCAN_USBBUS1`

### `Bitrate`

A CAN busz sebessége.

Nálatok tipikusan:
- `250000`

### `Connect`

Megnyitja a CAN interfészt.

### `Start`

Elindítja a szimulációs loopot.

### `Stop`

Leállítja a szimulációs loopot.

### `Status TX`

Ha be van kapcsolva, a szimulátor periodikusan küld `NODE_STATUS` frame-eket.

Ez a legfontosabb visszajelzés az ECU felé.

### `Diag TX`

Ha be van kapcsolva, a szimulátor periodikusan küld `NODE_DIAG` frame-eket.

Ez több buszforgalmat jelent.

### `Presence TX`

Ha be van kapcsolva, a szimulátor periodikusan küld `NODE_PRESENCE` frame-eket.

Ez is növeli a buszterhelést.

## `Service / Provisioning` blokk

Ez a node provisioning és service CAN tesztekhez való.

### `UID32 Hex`

A kiválasztott szimulált node azonosítója hexában.

### `Sensor`

Melyik logikai szenzorhoz tartozzon a node.

### `Node`

Melyik sor / node legyen.

### `Profile`

Melyik CAN bankot figyelje.

### `Test RPM`

A service `Test Spin` parancs fordulata.

### `Duration`

A teszt időtartama másodpercben.

### Gombok

#### `Discover`

Service discover kérés küldése.

#### `Assign`

UID alapján node szerep kiosztása.

#### `Save`

Config mentés szimulált node-ra.

#### `Cfg Read`

Node config lekérése.

#### `Diag Req`

Node diag lekérése.

#### `Identify`

Azonosítás.

A GUI-ban ez általában kiemeléssel / blink állapottal jelenik meg.

#### `Test Spin`

Rövid motorforgatás teszt.

#### `Set CAN Source`

Beállítja a szenzor / node / profile párost.

## Szenzorfülek

Van 4 fül:
- `Sensor 0`
- `Sensor 1`
- `Sensor 2`
- `Sensor 3`

Mindegyik fül ugyanazt a szerkezetet mutatja.

## `Sensor X Summary`

Ez a kiválasztott szenzorhoz tartozó utolsó `GLOBAL_CONTROL` állapot rövid nézete.

### `Mode`

Az ECU által küldött rendszer mód.

Például:
- `OFF`
- `MANUAL`
- `AUTO`

### `Flags`

A `GLOBAL_CONTROL.control_flags` mező.

### `Base RPM`

Az ECU által küldött alap fordulat.

### `Sync Pos`

Az ECU által küldött szinkron pozíció.

### `Sequence`

Az utolsó látott `GLOBAL_CONTROL` sequence.

### `Last RX age`

Mennyi idő telt el azóta, hogy a szimulátor ehhez a szenzorhoz utoljára érvényes vezérlő frame-et kapott.

Ez nagyon fontos hibakereséshez:
- ha ez nagy, akkor a szimulátor nem kap friss ECU vezérlést
- ha kicsi, akkor jó buszon van és él a kommunikáció

## Node tábla

Szenzoronként a node-ok aktuális állapotát mutatja.

### `Node`

A node sorszáma.

### `UID32`

A szimulált node rövid UID-je.

### `Profile`

Melyik CAN profilt figyeli.

### `Enabled`

Kapott-e engedélyező parancsot.

### `Section`

Az adott node-hoz tartozó szakasz aktív-e a `sectionMask` alapján.

### `Target RPM`

Az ECU vezérlés és a trim alapján számolt célfordulat.

### `Actual RPM`

A szimulált tényleges motorfordulat.

### `Trim RPM`

Az ECU-tól jövő trim korrekció.

### `Pos`

A szimulált motor pozíció.

### `Bus V`

A szimulált tápfeszültség.

### `Current A`

A szimulált motoráram.

### `Ctrl C`

A vezérlő hőmérséklete.

### `Motor C`

A motor hőmérséklete.

### `Warnings`

Warning bitek.

### `Faults`

Fault bitek.

### `Identify`

Mutatja, hogy az adott node éppen identify állapotban van-e.

## Alsó log ablak

Itt látható:
- bejövő `GLOBAL_CONTROL`
- bejövő `NODE_CMD`
- service CAN parancsok
- küldési hibák
- buszhibák

Hibakeresésnél ez a legfontosabb rész.

## Fontos működési elv

A szimulátor jelenleg úgy van visszafogva, hogy:
- alapból a `Status TX` megy
- a `Diag TX` és `Presence TX` kapcsolható
- csak a releváns, tényleg megszólított node-ok válaszoljanak

Ez azért kell, hogy ne terhelje túl a CAN buszt.

## Tipikus használat

### Egyszerű ECU teszt

1. `Connect`
2. `Start`
3. hagyd csak a `Status TX`-et bekapcsolva
4. nézd a `Last RX age` mezőt
5. ha az kicsi és a `Base RPM` változik, akkor az ECU vezérli a szimulátort

### Service teszt

1. írd be a kívánt `UID32` értéket
2. állítsd be a `Sensor`, `Node`, `Profile` mezőket
3. `Assign`
4. ha kell `Save`
5. utána `Identify`, `Test Spin`, `Diag Req`

### Ha gond van

Először ezt nézd:
- jó CAN csatorna van-e megadva
- jó bitrate van-e
- `Last RX age` frissül-e
- a logban van-e bus-off vagy send hiba

## Futtatás

```powershell
python tools\Motor-Simulator\motor_simulator_pyside.py
```

## Függőségek

- `PySide6`
- `python-can`
- megfelelő PCAN / CAN driver
