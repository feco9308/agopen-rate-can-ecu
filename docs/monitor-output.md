# Monitor Output

Ez a dokumentum az ECU opcionális monitor-kimeneti funkciojat irja le.

## Cel

A rate ECU a normal rate es motorvezerles mellett kulso monitor alkalmazasok fele is tud UDP adatot kuldeni.

Jelenlegi tamogatott modok:
- `Off`
- `Planter`
- `Blockage`

A valasztas az ECU configbol tortenik, es az ECU Service Tool tudja allitani.

## Alapelv

A monitor output:
- kulon kapcsolhato
- nem resze a Rate App alap PGN forgalmanak
- opcionális kompatibilitasi reteg

Ha ki van kapcsolva:
- az ECU nem kuld kulso monitor adatot

Ha be van kapcsolva:
- a kivalasztott mod szerint kuld UDP csomagokat

## Modok

### `Off`

Semmilyen monitor output nem megy ki.

### `Planter`

Az ECU AOGPlanterV2-szeru UDP csomagokat kuld.

Hasznalt planter PGN-ek:
- `E0` config
- `E1` row population `9..16`
- `E2` row population `1..8`
- `E3` doubles
- `E4` skips
- `E5` summary
- `E6` row status

UDP port:
- `15555`

### `Blockage`

Az ECU BlockageMonitor kompatibilis UDP csomagokat kuld.

Jelenlegi hasznalt csomag:
- `32100`

UDP port:
- `25600`

## Mi becsult es mi valos

Fontos:
- a jelenlegi rendszer nem valodi magszenzoros seed monitor
- a monitor output a mostani ECU altal ismert adatokbol dolgozik

### Valos forrasok

Az ECU ezekhez rendelkezik valos adattal:
- node online / offline
- node fault / warning
- section on / off
- target motor RPM
- actual motor RPM

### Becsult adatok

Ezek jelenleg becsultek:
- planter population
- blockage rate

Ez a becsles a target es actual motor RPM aranyabol keszul.

## Planter mod lekepzese

### Population

Soronkent becsult population:

```text
estimated_population = target_population * (actual_rpm / target_rpm)
```

Ha:
- `target_rpm <= 0`
- vagy `actual_rpm <= 0`

akkor az eredmeny `0`.

### Summary population

Az aktiv sorok atlagolt becsult populationje.

### Skip

Jelenleg nincs valodi magszenzoros forras.

Ezert:
- `skip = 0`

### Double

Jelenleg nincs valodi magszenzoros forras.

Ezert:
- `double = 0`

### Singulation

Mivel nincs valodi skip / double meres:
- `singulation = 100`

### Row status

A sor allapota ECU es node allapotbol kepezodik.

Javasolt logika:
- section off -> normal / szurke
- node offline vagy fault -> piros
- warning vagy nagy RPM hiba -> sarga
- egyebkent normal / zold

## Blockage mod lekepzese

Blockage modban az ECU soronkent egy kompatibilis `32100` csomagot kuld.

### Rate byte

A kuldott rate byte becsult:

```text
estimated_population = target_population * (actual_rpm / target_rpm)
rate_byte = estimated_population / 1000
```

Majd `0..255` tartomanyra clampelve megy ki.

### Module / row kiosztas

A blockage kimenet a logikai sorokat module + row bontasban kuldi.

A fo beallitasok:
- `Monitor Rows`
- `Rows / Module`

Pelda:
- `Monitor Rows = 24`
- `Rows / Module = 8`

akkor:
- module 0 -> rows 0..7
- module 1 -> rows 8..15
- module 2 -> rows 16..23

## ECU Config mezok

### `Enable Monitor Output`

Altalanos fo kapcsolo.

### `Mode`

Valaszthato:
- `Off`
- `Planter`
- `Blockage`

### `Monitor Rows`

Hany logikai sort kuldjon ki a monitor output.

### `Rows / Module`

Blockage modban hasznalt felosztas.

### `Row Width cm`

Planter modhoz hasznalt sortav.

### `Target Population`

A becsles alapja planter es blockage modban.

### `Doubles Factor`

Planter konfiguracios parameter.

### `Metric Units`

Planter mod metrikus flagje.

### `Blockage Threshold`

Blockage modhoz tartozo parameter.

## Fajlok

Firmware:
- `src/ethernet_link.cpp`
- `src/ethernet_link.h`
- `src/planter_output.cpp`
- `src/planter_output.h`
- `src/blockage_output.cpp`
- `src/blockage_output.h`
- `src/runtime_config.cpp`
- `src/runtime_config.h`

Tool:
- `tools/pyside-ecu-service/ecu_service_tool.py`

## Korlatozasok

Jelenleg:
- a planter es blockage kimenet becsult rate / population adatot hasznal
- nem valodi seed pulse vagy blockage sensor adatot

Ez kompatibilitasi modnak jo, de nem teljes erteku magszenzoros rendszer.

## Javasolt tovabblepes

Ha kesobb valodi seed monitor funkcio kell, akkor uj adatforras szukseges:
- magszenzor
- soronkénti impulzus
- vagy kulon blockage / seed sensor ECU

Ekkor a `skip`, `double`, `singulation` es a blockage rate mar valodi meresbol szamolhato.
