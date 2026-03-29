# Rendszer architektúra

## Áttekintés

A rendszer egy több soros vetőgép elektromos hajtásvezérlésére szolgál, ahol minden vetősor külön motorral rendelkezik.

A vezérlés célja:

* pontos vetési ráta tartása
* sorok közötti szinkron biztosítása
* megbízható, bővíthető rendszer kialakítása

---

## Fő komponensek

### Teensy ECU (központi vezérlő)

Feladatai:

* rate (vetési ráta) számítás
* központi állapotgép (system mode)
* CAN busz vezérlés
* LAN kommunikáció a Windows / AOG felé
* referencia sebesség és pozíció generálása

---

### STM32G motor node (soronként)

Minden vetősorhoz tartozik egy külön node.

Feladatai:

* BLDC motor vezérlés (FOC / PID)
* aktuális fordulatszám mérése
* aktuális pozíció mérése (encoder / hall)
* helyi szabályozás végrehajtása
* CAN kommunikáció a Teensy ECU-val

---

### Windows / AOG

Feladatai:

* felhasználói interfész
* sebesség és vetési ráta meghatározása
* monitorozás (RPM, pozíció, hibák)

---

## Kommunikációs struktúra

```
Windows / AOG
      │
      ▼
   (LAN)
      │
      ▼
  Teensy ECU
      │
      ▼
    (CAN)
      │
      ▼
  STM32G node-ok
```

---

## Vezérlési elv

A rendszer nem használ közvetlen PWM vezérlést a központból.

### Helyette:

A Teensy ECU egy közös referencia jelet küld minden node számára:

* `base_rpm` → kívánt fordulatszám
* `sync_pos` → referencia pozíció

Ez egy **master–slave szinkronizált rendszer**.

---

## Virtuális referencia tengely

A Teensy ECU egy virtuális tengelyt futtat:

```
sync_pos(t+dt) = sync_pos(t) + base_rpm * dt
```

Ez a rendszer közös időalapja és pozíció referenciája.

---

## Node működési logika

A STM32G node minden ciklusban:

1. fogadja a `base_rpm` és `sync_pos` értékeket
2. méri a saját pozícióját (`actual_pos`)
3. kiszámolja az eltérést:

```
position_error = sync_pos - actual_pos
```

4. fordulatszám korrekciót számol:

```
final_target_rpm = base_rpm + trim_rpm + Kp * position_error
```

5. helyben szabályozza a motort (FOC / PID)

---

## Üzemmódok (system_mode)

A rendszer több működési módot támogat:

* **OFF**

  * minden hajtás tiltva

* **MANUAL**

  * kézi vezérlés
  * teszteléshez használható

* **AUTO**

  * normál működés
  * AOG által vezérelt

* **CALIBRATION**

  * kalibráció
  * encoder nullázás

* **FAULT_HOLD**

  * hiba után letiltott állapot
  * csak kézi reset után indul

---

## Szakaszolás (Section control)

A szakaszolás nem globális broadcast szinten történik.

👉 Node szinten van kezelve:

* sor engedélyezése / tiltása
* egyedi beállítások
* külön vezérlés soronként

---

## Biztonság

A rendszer több szinten védi magát:

### CAN timeout

* ha a node nem kap friss adatot → motor leáll

### E-STOP

* broadcast üzenet
* azonnali leállás minden node-on

### Fault kezelés

* túláram
* túlmelegedés
* szenzor hiba
* kommunikációs hiba

---

## Visszajelzések (telemetria)

A node folyamatosan visszaküldi:

### Gyors állapot

* aktuális RPM
* aktuális pozíció
* állapot flag-ek
* hiba kód

### Diagnosztika

* tápfeszültség
* motor áram
* vezérlő hőmérséklet
* motor hőmérséklet
* fault és warning flag-ek

---

## Hardver mérési megjegyzés

A node által küldött diagnosztikai adatok forrása:

* shunt ellenállás + ADC → motor áram
* feszültségosztó + ADC → bus feszültség
* driver fault jelek → hibák
* hőmérséklet szenzorok

---

## Rendszer célja

* pontos vetési ráta
* sorok közötti szinkron
* stabil működés
* moduláris felépítés
* könnyű bővíthetőség (több sor)

---

## Bővíthetőség

A rendszer egyszerűen skálázható:

* több node hozzáadható CAN buszon
* node ID alapú címzés
* broadcast alapú közös vezérlés

---

## Összefoglalás

A rendszer egy modern, CAN-alapú, elosztott vezérlés:

* központi ECU (Teensy)
* intelligens node-ok (STM32G)
* referencia alapú vezérlés (RPM + pozíció)
* valós idejű szinkronizáció

Ez a felépítés alkalmas precíziós mezőgazdasági alkalmazásokhoz.
