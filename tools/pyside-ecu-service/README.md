# PySide ECU Service Tool

Egyszeru PySide6 alkalmazas az ECU custom UDP PGN-jeihez.

## Funkciok

- ECU config olvasas/iras
- ECU config save/load
- diag control kuldes
- ECU diag fogadas
- node service parancsok kuldese:
  - discover
  - assign
  - save cfg
  - test spin
  - diag req
  - cfg read
  - identify
  - reboot
  - set can source

## Futtatas

```powershell
python tools\pyside-ecu-service\ecu_service_tool.py
```

## Halozati alapertelmezes

- kuldes ECU fele: `28888`
- fogadas ECU service valaszokra: `30001`
- alap cel IP: `192.168.1.255`

Ha nem broadcast kell, a felso savban atirhato a cel IP.
