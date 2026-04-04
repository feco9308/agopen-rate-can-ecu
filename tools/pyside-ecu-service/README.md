# PySide ECU Service Tool

Magyar leírás a `tools/pyside-ecu-service/ecu_service_tool.py` programhoz.

## Mire való

Ez a program az ECU-val beszél UDP-n keresztül.

Fő céljai:
- ECU konfiguráció olvasása és írása
- ECU beállítások mentése / visszatöltése
- diagnosztikai stream kérése az ECU-tól
- node service parancsok küldése az ECU-n keresztül a CAN buszra

Röviden:
- ez a program az ECU beállító és szerviz eszköze
- nem közvetlenül a motor node-okkal beszél Etherneten
- hanem az ECU custom UDP PGN-jeit használja

## Hálózati működés

Alapértékek:
- küldés az ECU felé: `UDP 28888`
- fogadás az ECU custom válaszaira: `UDP 30001`
- alap cél IP: `192.168.1.255`

Ha nem broadcast kell, a felső sorban átírható a cél IP, például:
- `192.168.1.200`

## Felső sáv

### `ECU IP / Broadcast`

Ide kerül az ECU IP-je vagy a broadcast cím.

Példák:
- `192.168.1.200`
- `192.168.1.255`

### `Apply Target`

Elmenti a cél IP-t a programban.

### `RX`

Mutatja, hogy a tool figyeli-e a `30001` portot.

Jó állapot például:
- `RX: listening on *:30001`

### `Retry RX Bind`

Újra megpróbálja megnyitni a figyelő UDP portot.

Ez akkor kellhet, ha:
- a port foglalt volt
- a program indulásakor valamiért nem tudott bindolni

## `ECU Config` fül

Ez az ECU saját beállításaira való.

### `Machine Config`

#### `Active Sensors`

Hány szenzorcsatornát használjon az ECU.

Jelenlegi firmware oldali tipikus érték:
- `1..4`

#### `Configured Rows`

Hány sor / szakasz legyen aktív a gépben.

Ez hatással van például:
- hány `NODE_CMD` megy ki CAN-en
- hány node-ot vesz figyelembe az ECU
- UPM -> RPM számításra

#### `Holes / Rev`

A vetőtárcsa furatszáma.

#### `UPM Scale`

Az ECU oldali skála a Rate App `target_upm` értékéhez.

### `Get`

Lekéri az adott blokk aktuális ECU értékeit.

### `Set`

Elküldi a blokkban látható értékeket az ECU-nak.

Fontos:
- ez még nem feltétlenül menti EEPROM-ba
- futás közben átírja a runtime configot

### `Drive Config`

#### `Gear Ratio`

Motor / tárcsa áttétel.

#### `Trim Limit RPM`

A pozíció alapú finomkorrekció maximális RPM eltérése.

#### `Position Kp`

A pozícióhibából számolt trim arányossági tényezője.

### `Diag Config`

#### `Enable Diagnostics`

Bekapcsolja az ECU diag funkciót.

#### `Enable Stream`

Folyamatosan küldi a diagnosztikai adatokat.

#### `Diag Period ms`

Milyen gyakran menjen a diag stream.

#### `Diag Detail`

Alap vagy bővített részletesség.

### `Network Config`

#### `IP Last Octet`

Az ECU IP címének utolsó tagja.

Példa:
- `200` -> `192.168.1.200`

#### `Module ID`

A Rate App felé használt modulazonosító.

### `Persistence`

#### `Save To ECU`

Elmenti az aktuális configot az ECU nem felejtő tárába.

#### `Load From ECU`

Visszatölti a korábban mentett beállításokat.

## `Diag` fül

Ez az ECU összesített állapotának figyelésére való.

### `Enable`

Bekapcsolja a diagnosztikát.

### `Stream`

Folyamatos diagnosztikai küldést kér.

### `S0`, `S1`, `S2`, `S3`

Kiválasztja, mely szenzorcsatornákról kérjen adatot.

### `Node Mask Hex`

Hexadecimális node maszk.

Példa:
- `FFFF` = az első 16 node mind engedélyezett

### `Period ms`

A diag stream periódusa.

### `Detail`

Basic vagy Extended részletesség.

### `Send Diag Control`

Elküldi a fenti diag beállításokat az ECU-nak.

### `Request ECU Status`

Lekéri a fő ECU blokkokat és elküldi a diag controlt.

Ez jó első tesztgomb.

### `Request Node Details`

Nem folyamatos streamet kér, hanem egyszeri részletes node-diag lekérést a kiválasztott:
- szenzorra
- node-ra

Ez azért fontos, mert a részletes node-diag csomagok sokkal nagyobb Ethernet forgalmat okoznak, ezért már nem megyenek folyamatos streamben.

### Alsó tábla

Szenzoronként mutatja:
- mód
- base RPM
- target UPM
- online node darabszám
- átlagos RPM
- összegzett RPM

### Node diag tábla

Az összesített szenzor sorok alatt egy külön node-szintű diagnosztikai tábla is van.

Ebben node-onként látszik:
- szenzor
- node
- status flags
- error code
- actual RPM
- pozíció
- buszfeszültség
- motoráram
- vezérlő hőmérséklet
- motor hőmérséklet
- warning bitek
- fault bitek

Ez már az ECU által a CAN-ről összegyűjtött és UDP-n továbbadott részletes node diag adat.

## `Node Service` fül

Ez a motor node provisioning / szerviz rész.

Itt a tool nem közvetlenül a node-hoz beszél Etherneten, hanem:
- UDP-n szól az ECU-nak
- az ECU service CAN frame-eket küld a buszra

### `Discover`

#### `Delay Slots`

A discover válaszok késleltetésének slot értéke.

#### `Request UID`

Kérjen UID választ a node-októl.

#### `Request CFG`

Kérjen config választ is.

#### `Send Discover`

Broadcast discover a service CAN-en.

### `Target Node`

#### `UID32 Hex`

A kiválasztott node 32 bites UID azonosítója hexában.

#### `Sensor Channel`

Melyik logikai szenzorhoz tartozzon a node.

#### `Node ID`

Melyik sor / node legyen.

#### `CAN Profile`

Melyik CAN bankot figyelje.

### `Assign`

UID alapján kiosztja a node szerepét.

### `Save CFG`

Eltároltatja a node configját.

### `Service Actions`

#### `Test RPM`

Teszt forgatási fordulat.

#### `Duration s`

Teszt időtartama másodpercben.

#### `Identify Mode`

Például:
- `LED Blink`
- `Short Jog`

#### `Test Spin`

Rövid forgatás teszt.

#### `Diag Req`

Node diag kérés.

#### `CFG Read`

Node config visszaolvasás.

#### `Set CAN Source`

Beállítja, melyik szenzor / node / CAN profil alapján figyeljen.

#### `Identify`

Azonosító parancs a node-nak.

#### `Reboot`

Node újraindítás.

## Alsó log ablak

Ide kerülnek:
- TX PGN üzenetek
- RX ECU diag válaszok
- node service válaszok
- hibák

Ha valami nem működik, ez az első hely, amit nézni érdemes.

## Tipikus használat

### ECU beállítás

1. cél IP beállítása
2. `Apply Target`
3. `ECU Config` fülön `Get`
4. értékek átírása
5. `Set`
6. ha jó, `Save To ECU`

### Node keresés és kiosztás

1. `Node Service`
2. `Send Discover`
3. UID beírása
4. `Sensor Channel`, `Node ID`, `CAN Profile` beállítása
5. `Assign`
6. `Save CFG`

### Diagnosztika

1. `Diag` fül
2. `Enable`
3. opcionálisan `Stream`
4. szenzorok kijelölése
5. `Send Diag Control`
6. `Request ECU Status`

## Futtatás

```powershell
python tools\pyside-ecu-service\ecu_service_tool.py
```
