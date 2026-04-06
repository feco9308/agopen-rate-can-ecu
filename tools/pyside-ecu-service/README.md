# PySide ECU Service Tool

Magyar leiras a `tools/pyside-ecu-service/ecu_service_tool.py` programhoz.

## Mire valo

Ez a program az ECU-val beszel UDP-n keresztul.

Fo feladatai:
- ECU konfiguracio olvasasa es irasa
- ECU beallitasok mentese / visszatoltese
- diagnosztikai stream kerese az ECU-tol
- node service parancsok kuldese az ECU-n keresztul a CAN buszra
- monitor output mod valasztasa (`Off / Planter / Blockage`)

Roviden:
- ez a program az ECU beallito es szerviz eszkoze
- nem kozvetlenul a motor node-okkal beszel Etherneten
- hanem az ECU custom UDP PGN-jeit hasznalja

## Halozati mukodes

Alapertekek:
- kuldes az ECU fele: `UDP 28888`
- fogadas az ECU custom valaszaira: `UDP 30001`
- alap cel IP: `192.168.1.255`

Ha nem broadcast kell, a felso sorban atirhato a cel IP, peldaul:
- `192.168.1.200`

## Felso sav

### `ECU IP / Broadcast`

Ide kerul az ECU IP-je vagy a broadcast cim.

Peldak:
- `192.168.1.200`
- `192.168.1.255`

### `Apply Target`

Elmenti a cel IP-t a programban.

### `RX`

Mutatja, hogy a tool figyeli-e a `30001` portot.

Jo allapot peldaul:
- `RX: listening on *:30001`

### `Retry RX Bind`

Ujra megprobalja megnyitni a figyelo UDP portot.

## `ECU Config` ful

Ez az ECU sajat beallitasaira valo.

### `Machine Config`

#### `Active Sensors`

Hany szenzorcsatornat hasznaljon az ECU.

Jelenlegi firmware oldali tartomany:
- `1..4`

#### `Configured Rows`

Hany sor / szakasz legyen aktiv a gepben.

Ez hatassal van peldaul:
- hany `NODE_CMD` megy ki CAN-en
- hany node-ot vesz figyelembe az ECU
- UPM -> RPM szamitasra
- diag auto node-detail maximumara

#### `Holes / Rev`

A vetotarcsa furatszama.

#### `UPM Scale`

Az ECU oldali skala a Rate App `target_upm` ertekehez.

### `Drive Config`

Ez valojaban nem attetelt jelent, hanem szinkron / trim konfiguraciot.

#### `Trim Limit RPM`

A soronkenti korrekcio maximalis RPM elterese.

Ekkora `+/- trim_rpm` mehet ki egy node-ra.

#### `Position Kp`

A poziciohibabol szamolt trim erossege.

Minel nagyobb, annal agresszivebben korrigalja a node-okat.

### `Channel Ratio Config`

Ez a szenzoronkenti attetel konfig.

#### `Sensor Channel`

Melyik csatorna attetelet szerkeszted:
- `0`
- `1`
- `2`
- `3`

#### `Drive Ratio`

Szenzoronkenti gep / hajtas oldali szorzo.

#### `Motor Ratio`

Szenzoronkenti motor oldali szorzo.

A vegso szamitasban a ketto szorzata megy:
- `combined_ratio = drive_ratio * motor_ratio`

### `Diag Config`

Ez az ECU sajat diagnosztikai viselkedeset szabalyzza.

#### `Enable Diagnostics`

Bekapcsolja az ECU diag modjat.

Ez kapcsolja a `GLOBAL_CONTROL.ctrl_diag_enable` bitet is, tehat a motor node-ok ezt latjak CAN-en.

#### `Enable Stream`

Folyamatosan kuldje-e az ECU a summary diag adatokat UDP-n a service tool fele.

#### `Diag Period ms`

Milyen gyakran menjen a summary diag stream.

#### `Diag Detail`

Basic vagy Extended reszletesseg.

Jelenleg a fo kulonbseg inkabb a tool oldali kezelesben van, nem a teljes rendszer logikajaban.

### `Network Config`

#### `IP Last Octet`

Az ECU IP cimenek utolso tagja.

Pelda:
- `200` -> `192.168.1.200`

#### `Module ID`

A Rate App fele hasznalt modulazonosito.

### `Monitor Output Config`

Ez a kulso monitor alkalmazasok fele mennyo opcionális UDP kimenet.

#### `Enable Monitor Output`

Ha ki van kapcsolva:
- az ECU nem kuld monitor outputot

Ha be van kapcsolva:
- az ECU a kivalasztott mod szerint kuld kimeneti csomagokat

#### `Mode`

Valaszthato mod:
- `Off`
- `Planter`
- `Blockage`

Jelentes:
- `Off`: semmit nem kuld
- `Planter`: AOGPlanterV2 fele kuld kompatibilis UDP csomagokat
- `Blockage`: BlockageMonitor fele kuld kompatibilis UDP csomagokat

#### `Monitor Rows`

Hany logikai sort kuldjon ki a monitor output modul.

Ez nem feltetlenul ugyanaz, mint a `Configured Rows`, de altalaban ahhoz igazodik.

#### `Rows / Module`

Blockage modban hasznalt felosztas.

Megadja, hogy egy logikai module-ra hany sor essen.

#### `Row Width cm`

Planter modban hasznalt sortav.

#### `Target Population`

A planter / blockage becsles alapja.

#### `Doubles Factor`

Planter modban konfiguracios parameter.

#### `Metric Units`

Planter modban metrikus megjeleniteshez hasznalt flag.

#### `Blockage Threshold`

Blockage modhoz tartozik.

Az ECU csak tovabbitja a beallitast; a blockage app sajat logikaja is hasznalhatja.

### `Persistence`

#### `Save To ECU`

Elmenti az aktualis configot az ECU nem felejto taraba.

#### `Load From ECU`

Visszatolti a korabban mentett beallitasokat.

## `Diag` ful

Ez az ECU osszesitett allapotanak figyelesere valo.

### `Enable`

Bekapcsolja a diagnosztikat az ECU-ban.

### `Stream`

Folyamatos summary diagnosztikai kuldest ker az ECU-tol.

### `S0`, `S1`, `S2`, `S3`

Kivalasztja, mely szenzorcsatornakrol kerjen adatot a tool.

### `Node Mask Hex`

Hexadecimalis node maszk a reszletes node-diag lekereshez.

Bitek:
- bit0 = node1
- bit1 = node2
- ...
- bit15 = node16

Peldak:
- `0001` = csak node1
- `0020` = csak node6
- `003F` = node1..6
- `FFFF` = node1..16, illetve a `Configured Rows` maximumaig

Mit csinal:
- az `Auto Node Details` ezen a maszkon lepked vegig
- a reszletes also tablaba ezekrol a node-okrol ker adatot

Mit nem csinal:
- nem tilt le motorokat
- nem valtoztatja a Rate App mukodeset
- nem maszkolja a `GLOBAL_CONTROL` vagy `NODE_CMD` kuldest

### `Period ms`

A summary diag stream periodusa.

### `Detail`

Basic vagy Extended reszletesseg.

### `Detail Sensor`

Kezileg melyik szenzorrol kerj reszletes node adatot.

### `Detail Node`

Kezileg melyik node-rol kerj reszletes adatot.

### `Auto Node Details`

Lassu, korbejaro node-diag lekeres.

A kivalasztott:
- szenzor pipak
- `Node Mask Hex`
- `Configured Rows`

alapjan vegigmegy a node-okon, es nem kell kezzel gombot nyomogatni.

### `Auto ms`

Az automatikus node-detail lekeres periodusa.

### `Send Diag Control`

Elkuldi a fenti diag beallitasokat az ECU-nak.

### `Request ECU Status`

Lekeri a fo ECU blokkokat es elkuldi a diag controlt.

Ez jo elso tesztgomb.

### `Request Node Details`

Egyszeri reszletes node-diag lekeres a kivalasztott:
- szenzorra
- node-ra

### Felso summary tabla

Szenzoronkent mutatja:
- mod
- base RPM
- target UPM
- online node darabszam
- atlagos RPM
- osszegzett RPM

### Also node diag tabla

Node-onkent latszik:
- szenzor
- node
- status flags
- error code
- actual RPM
- pozicio
- buszfeszultseg
- motoraram
- vezerlo homerseklet
- motor homerseklet
- warning bitek
- fault bitek

## `Node Service` ful

Ez a motor node provisioning / szerviz resz.

Itt a tool nem kozvetlenul a node-hoz beszel Etherneten, hanem:
- UDP-n szol az ECU-nak
- az ECU service CAN frame-eket kuld a buszra

Fo funkciok:
- discover
- assign
- save cfg
- test spin
- diag req
- cfg read
- identify
- reboot
- set CAN source

## Monitor output megjegyzes

A `Planter` es `Blockage` mod jelenleg becsult adatokat is hasznal.

Ez azt jelenti:
- a rate / population jellegu ertekek motor RPM alapon vannak becsulve
- nem tenyleges magszenzoros meresbol jonnek

Ez kompatibilitasi modnak jo, de nem teljesen azonos egy valodi seed sensor rendszerrel.
