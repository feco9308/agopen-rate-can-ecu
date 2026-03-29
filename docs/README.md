# Dokumentáció

Ez a mappa tartalmazza a vetőgép CAN-alapú ECU rendszerének teljes műszaki dokumentációját.

## Tartalom

- `architecture.md`  
  → rendszer felépítés, komponensek és működés

- `can-matrix.md`  
  → CAN kommunikációs protokoll (üzenetek, struktúrák)

## Cél

A dokumentáció célja, hogy:

- egyértelmű legyen a rendszer működése
- a firmware fejlesztés egységes logika szerint történjen
- a CAN kommunikáció stabil és bővíthető legyen
- több fejlesztő is tudjon dolgozni a projekten

## Fő koncepció

A rendszer:

- központi ECU (Teensy)
- CAN buszon kommunikáló motor node-ok (ESP32)
- LAN kapcsolat a Windows / AOG felé

A hagyományos PWM vezérlés helyett:

👉 CAN-alapú referencia vezérlés van használva

- sebesség (RPM)
- pozíció (szinkronizáció)
