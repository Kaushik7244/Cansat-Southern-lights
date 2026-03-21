# CanSat SouthernLights 2026 — Power Delivery

---

## Power Budget (30-Minute Flight)

### Component Current Draw

| Component | Rail | Current (avg) | Power |
|-----------|------|--------------|-------|
| Portenta H7 — both cores active, WiFi off | 3.3 V | 135 mA | 445 mW |
| Nicla Sense ME — BLE + BHI260 + BMP390 | 3.3 V | 12 mA | 40 mW |
| BME688 — with periodic gas heater pulses | 3.3 V | 3 mA | 10 mW |
| Grove Air530 GPS — tracking mode | 3.3 V | 28 mA | 92 mW |
| SD card — periodic 10 s writes | 3.3 V | 8 mA | 26 mW |
| LoRa Vision Shield — intermittent TX | 3.3 V | 10 mA | 33 mW |
| Camera — periodic captures | 3.3 V | 10 mA | 33 mW |
| APC220 @ 13 dBm — ~50% TX duty | 5 V | 45 mA | 225 mW |
| Servo × 2 — active steering under load | 5 V | 150 mA each | 1500 mW |
| Regulator losses (~12%) | — | — | ~120 mW |
| **3.3 V rail total** | | **~206 mA** | **~679 mW** |
| **5 V rail total (with servos)** | | **~345 mA** | **~1725 mW** |
| **System total — descent (worst case)** | | | **~2.5 W** |
| **System total — ascent (no servos)** | | | **~1.1 W** |

> Note: APC220 is currently disabled in `hw_config.h`. Budget includes it as active.
> Servo current is load-dependent. 150 mA/servo is a mid-load estimate; stall current is ~600 mA each.

### Flight Phase Energy

| Phase | Duration | Power | Energy |
|-------|----------|-------|--------|
| Ascent | 10 min | 1.1 W | 0.18 Wh |
| Descent | 20 min | 2.5 W | 0.83 Wh |
| **Total** | **30 min** | | **~1.0 Wh** |
| **With 25% safety margin** | | | **~1.4 Wh** |

---

## Option A — 2× 18650 (series, 7.4 V → 5 V) + 1× LiPo directly to Portenta

**Architecture**

```
2× 18650 in series (7.4 V, ~2500 mAh)
└── Buck converter → 5 V ──── Servos (×2)
                           └── APC220

1× LiPo 3.7 V (~1000 mAh)
└── Direct to Portenta VBAT ── Portenta internal reg → 3.3 V
                                ├── BME688
                                ├── GPS (Air530)
                                ├── Nicla Sense ME
                                ├── SD card
                                ├── LoRa Vision Shield
                                └── Camera
```

| | Value |
|--|-------|
| 5 V rail energy available | 2500 mAh × 7.4 V × 0.87 (buck eff.) = **16 Wh** |
| 3.3 V rail energy available | 1000 mAh × 3.7 V × 0.88 (LDO eff.) = **3.3 Wh** |
| Total available | **~19 Wh** |
| Required | ~1.4 Wh |
| Margin | **13×** |

**Pros**
- Completely isolated servo rail — servo stall cannot brownout the MCU
- High-current headroom on the 5 V rail; 18650s in series are well-suited for buck conversion
- Portenta VBAT accepts 3.5–4.2 V directly — no regulator needed on the logic side
- Maximum robustness for the competition mission

**Cons**
- Heaviest and largest option (3 cells total)
- Two separate battery systems to charge and monitor
- Overkill capacity — 13× margin means most of the battery is never used

---

## Option B — Single 18650 for Everything

**Architecture**

```
1× 18650 (3.6 V nominal, ~2500 mAh)
├── Direct to Portenta VBAT ──── Portenta internal reg → 3.3 V
│                                 ├── BME688, GPS, Nicla, SD, LoRa, Camera
└── Boost converter → 5 V ─────── Servos (×2)
                               └── APC220
```

| | Value |
|--|-------|
| Total energy available | 2500 mAh × 3.6 V × 0.87 = **7.8 Wh** |
| Required | ~1.4 Wh |
| Margin | **5.5×** |

**Pros**
- Lightest and simplest — one cell, one boost converter
- 18650 can easily source 600 mA+ without sag (low internal resistance ~100–150 mΩ)
- Portenta VBAT accepts raw 18650 voltage directly during discharge (4.2 V → 3.0 V)
- More than sufficient capacity for a 30-minute flight

**Cons**
- Shared battery — a servo stall (peak ~1.2 A) will cause a brief voltage dip on the same rail the Portenta is drawing from. A low-ESR cell and careful PCB layout mitigate this
- Boost converter adds a component and ~10–13% efficiency loss on the 5 V path
- No redundancy — a single cell failure ends the mission

**Recommended boost converter:** MT3608 or PAM2401 (both handle 2 A output, compact footprint)

---

## Option C — 2× 3.7 V Flat LiPo (Square Pack), Split Rails

Intended for builds where cylindrical 18650s don't fit the enclosure geometry. Flat LiPo pouch cells (e.g. 103450, 604060, or similar) can be sourced in the 500–2000 mAh range in a thin rectangular form factor.

The 5 V boost and charging for LiPo A is handled by a **Seeed LiPo Rider Plus** — a 25 × 41 mm module that combines boost conversion, USB-C charging input, and battery protection in one board.

**Architecture**

```
LiPo A (3.7 V, ≥800 mAh — servo/radio cell)
└── Seeed LiPo Rider Plus (25×41 mm)
    ├── USB-C in  ◄─── external charging cable (5V/2A)
    └── USB-A out → 5 V / 2.4 A ── Servos (×2)
                                └── APC220

LiPo B (3.7 V, ≥500 mAh — logic cell)
├── Direct to Portenta VBAT ── Portenta internal reg → 3.3 V
│                               ├── BME688, GPS, Nicla, SD, LoRa, Camera
└── TP4056 module ◄─── separate USB charging cable
```

> Do not use the LiPo Rider Plus onboard 3.3 V / 250 mA pin header for the sensor load —
> it is insufficient. Keep LiPo B directly on Portenta VBAT.

### LiPo Rider Plus — Key Specs

| Spec | Value |
| ---- | ----- |
| Charge input | USB-C, 5 V / 2 A (10 W) |
| Boost output | USB-A, 5 V / 2.4 A |
| Cell connection | JST2.0, single port |
| Onboard 3.3 V header | 250 mA (too marginal — do not use for sensors) |
| Size | 25 × 41 mm |
| Pass-through charging | Yes — charges cell and powers load simultaneously |

### Energy Budget

| | Value |
|--|-------|
| 5 V rail (LiPo A, 800 mAh) | 800 mAh × 3.7 V × 0.87 (LiPo Rider eff.) = **2.6 Wh** |
| 3.3 V rail (LiPo B, 500 mAh) | 500 mAh × 3.7 V × 0.88 (LDO eff.) = **1.6 Wh** |
| Total available | **~4.2 Wh** |
| Required | ~1.4 Wh |
| Margin | **3×** |

**Minimum recommended cell sizes:** LiPo A ≥ 800 mAh, LiPo B ≥ 500 mAh.

**Pros**

- LiPo Rider Plus replaces a discrete boost converter + charging circuit with one small module
- USB-C charging port on LiPo A can be routed to the enclosure wall — charge without opening the cansat
- Isolated servo rail — servo stall cannot brownout the MCU
- Flat form factor fits tight enclosures where 18650 cylinders don't
- Pass-through charging means the system can run tethered on USB during ground testing

**Cons**

- LiPo B (Portenta rail) still needs a separate TP4056 — two charge ports on the enclosure
- Smallest energy margin of the three options — verify actual cell capacity matches label
- Flat LiPo cells are more fragile than 18650s; need mechanical protection in the enclosure
- LiPo Rider Plus output is USB-A; requires a short USB-A breakout cable or pigtail to reach servo/APC220 terminals

---

## Comparison Summary

| | Option A | Option B | Option C |
|--|----------|----------|----------|
| Configuration | 2× 18650 (series) + 1× LiPo | 1× 18650 | 2× flat LiPo |
| Total energy | ~19 Wh | ~7.8 Wh | ~4.2 Wh |
| Margin over requirement | 13× | 5.5× | 3× |
| Isolated servo rail | Yes | No | Yes |
| Weight | Heaviest | Lightest | Medium |
| Form factor | Cylindrical | Cylindrical | Flat / flexible |
| Complexity | Medium | Low | Medium |
| Best for | Maximum robustness | Weight/space savings | Tight enclosures |

---

## Regulator Recommendations

| Need | Suggested part | Notes |
|------|---------------|-------|
| Buck 7.4 V → 5 V (Option A) | LM2596 or MP1584 | 3 A rated, ~87% eff. at 350 mA |
| Boost 3.6 V → 5 V (Options B, C) | MT3608 or PAM2401 | 2 A rated, compact, ~87% eff. |
| LiPo charging | TP4056 module | One per cell; includes over-discharge protection |

---

## Open Items

- [ ] Decide enclosure geometry — determines whether cylindrical 18650 or flat LiPo is viable
- [ ] Perform battery endurance test (Emily / Aurora) — run full system for 30 min, log voltage at start and end
- [ ] Confirm servo stall current under real paraglider brake load
- [ ] Select and test boost converter module before final assembly
