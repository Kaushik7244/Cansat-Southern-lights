# APC220 / RF7020 Radio Module Status

## Module Inventory

| Module | PCB Marking | Type | Frequency | Net ID | Status |
|--------|-------------|------|-----------|--------|--------|
| A | RF7020 v4.0 | Dorji DRF7020D13 clone | unknown | unknown | ❌ Dead — no response to any command |
| B | APC220 v4.0 | Genuine APPCON APC220 | 433920 kHz | 0 | ✅ Working |
| C | RF7020 v4.0 | Dorji DRF7020D13 clone | 433920 kHz | 0 | ✅ Working |
| D | RF7020 v4.0 | Dorji DRF7020D13 clone | 433920 kHz | 0 | ✅ Working |

### Notes
- All working modules configured to **433.920 MHz**, **9600 bps UART**, **9600 bps RF**, **power level 9**, **no parity**
- Modules A, C, D are Dorji DRF7020D13 clones — compatible with APC220 at RF level but require hardware UART for reliable configuration (SoftwareSerial on Arduino Uno too unreliable for write operation)
- Module B is a genuine APPCON APC220
- Factory default frequencies were all different (433.0, 434.0, 434.5 MHz) — modules must be explicitly configured to match
- APC220 and RF7020 clones **may not be cross-compatible** at RF level — use matched pairs where possible
- Verified working pair: **C + D** (both RF7020 clones)

---

## Current Deployment

| Module | Connected To | Interface |
|--------|-------------|-----------|
| D | Portenta H7 | UART1 (D14/D13) |
| C | PC via CP2102 | COM1 |

---

## Portenta H7 Wiring

### Normal Operation (Bridge Mode)

| Portenta H7 Pin | Signal | RF7020 Pin |
|-----------------|--------|------------|
| D14 / PA_9 | UART1 TX | RXD (pin 4) |
| D13 / PA_10 | UART1 RX | TXD (pin 5) |
| 3.3V | Power | VCC (pin 2) + EN (pin 3) |
| GND | Ground | GND (pin 1) |
| — | Not connected | AUX (pin 6) |
| — | Floating | SET (pin 7) — internal pull-up holds HIGH |

### Configuration / Programming Mode

Same as above, plus:

| Portenta H7 Pin | Signal | RF7020 Pin |
|-----------------|--------|------------|
| D10 / PC_2 | SET control (GPIO) | SET (pin 7) |

> SET must be connected to a GPIO pin for programming. Tie SET HIGH (3.3V) or leave floating for normal operation.

---

## RF7020 Pin Reference

| Pin | Name | Function |
|-----|------|----------|
| 1 | GND | Ground |
| 2 | VCC | Power 3.4–5.5V |
| 3 | EN | Enable — HIGH for normal operation, LOW = sleep |
| 4 | RXD | UART input (connect to host TX) |
| 5 | TXD | UART output (connect to host RX) |
| 6 | AUX | Data in/out indication — not used |
| 7 | SET | LOW = config mode, HIGH = normal operation |

---

## Configuration Parameters

| Parameter | Value | Notes |
|-----------|-------|-------|
| Frequency | 433920 | 433.920 MHz in kHz units |
| DRFSK | 3 | 9600 bps RF air rate |
| POUT | 9 | 13 dBm max output power |
| DRIN | 3 | 9600 bps UART rate |
| Parity | 0 | No parity (8N1) |

**Write command:** `WR 433920 3 9 3 0`
**Read command:** `RD`

### Frequency Range
- DRF7020D13: 421–444 MHz
- APC220: 418–455 MHz

---

## CP2102 USB-Serial Adapter Notes

- Must run terminal software and RF-MAGIC **as Administrator** on Windows
- Set COM port to **COM1** for RF-MAGIC compatibility
- Flow control must be set to **None**
- RF7020 slots directly into CP2102 breakout header — but SET and EN land on wrong pins:
  - Bridge **VCC → EN** (adjacent pins, short wire)
  - Bridge **VCC → SET** (short wire)
- RF-MAGIC v1.2 can read but **cannot write** to RF7020 clones reliably
- Use Portenta H7 with hardware UART for reliable configuration
