# 🚜 Vetőgép kalibrációs táblázat (10 km/h)

Paraméterek:
- Sebesség: 10 km/h
- Sortáv: 0.75 m
- Sorok száma: 6
- Tárcsa: 26 lyuk
- Áttétel: 1:2
- ECU scale: 100

## 📊 Cél ráta → UPM → RPM → tőtáv

| Cél ráta | target_upm | Motor RPM | Tárcsa RPM | Tőtáv (cm) |
|---------:|-----------:|----------:|-----------:|-----------:|
| 40 | 27.7 | 35.5 | 17.8 | 36.5 |
| 50 | 34.7 | 44.4 | 22.2 | 29.2 |
| 54 | 37.4 | 48.0 | 24.0 | 26.7 |
| 60 | 41.6 | 53.3 | 26.6 | 24.1 |
| 70 | 48.5 | 62.2 | 31.1 | 20.6 |
| 80 | 55.4 | 71.1 | 35.5 | 18.0 |
| 90 | 62.4 | 80.0 | 40.0 | 16.0 |
| 100 | 69.3 | 88.9 | 44.4 | 14.4 |

---

## 🧠 Gyors összefüggések

motorRPM ≈ cél_ráta × 0.888  
target_upm ≈ cél_ráta × 0.693  

---

## 🎯 Gyakorlati célértékek

| Tőtáv | Ajánlott cél ráta |
|------|------------------|
| 15 cm | ~95–100 |
| 20 cm | ~70 |
| 25 cm | ~60 |
| 30 cm | ~50 |

---

## ⚠️ Megjegyzés

- Az értékek a jelenlegi AOG Rate logikára vannak kalibrálva  
- Függ:
  - sebességtől
  - munkaszélességtől
  - szakaszoktól
- Más sebességnél újrakalibrálás szükséges
