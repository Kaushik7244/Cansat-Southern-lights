# Portenta H7 — CanSat Pin Assignment Map

**Board:** Arduino Portenta H7 + Vision Shield LoRa
**As of:** 2026-03-16

---

## UART / Serial — all 3 hardware ports occupied

| Port | STM32 pins | Arduino | Role | Device | Baud |
|------|-----------|---------|------|--------|------|
| Serial1 | PA9 (TX) / PA10 (RX) | — | APC220 433 MHz radio | telemetry.cpp | 9600 |
| Serial2 | PA15 (TX) / PF6 (RX) | — | Grove Air530 GPS | gps.h `GPS_SERIAL` | 9600 |
| Serial3 / SerialLoRa | PJ8 (TX) / PJ9 (RX) | — | Vision Shield Murata LoRa (HD connector) | MKRWAN library | 19200 8E1 |

> **Serial3/SerialLoRa is on the HD (High-Density) connector** — not on the breakout header.
> PA15 and PF6 are available on the H7 breakout header for GPS wiring tomorrow.

---

## Vision Shield — control pins (managed by MKRWAN library)

| Pin | Role |
|-----|------|
| PC7 | `LORA_RESET` |
| PG7 | `LORA_BOOT0` |
| PJ11 | `LORA_IRQ_DUMB` |

These are driven automatically by `lora.begin()` — do not use for anything else.

---

## I2C

| Bus | STM32 pins | Arduino object | Role | Device |
|-----|-----------|---------------|------|--------|
| I2C3 | PH7 (SCL) / PH8 (SDA) | `Wire` | BME688 environmental sensor | M4 core, addr 0x77 |
| I2C1 | — (HD connector) | `Wire1` | Reserved (was Nicla ESLOV) | **Nicla now BLE — Wire1 free in practice** |

---

## PWM / Servo

| Arduino pin | Role |
|------------|------|
| D4 | `SERVO_PIN_LEFT` — left brake servo |
| D5 | `SERVO_PIN_RIGHT` — right brake servo |

---

## Wireless (no GPIO allocation)

| Protocol | Device | Notes |
|----------|--------|-------|
| BLE | Nicla Sense ME | Connects via BHY2Host BLE — no wires from H7 |
| WiFi | Internal (H7 onboard) | Used at boot for NTP; powered down after |

---

## Free pins — available for new wiring

All standard breakout header pins **not listed above** are unassigned.
Confirmed free candidates for the MKR WAN 1310 UART bridge:

| STM32 pin | Arduino pin | Notes |
|-----------|------------|-------|
| PA0 | D0 | UART4_TX — mbed `BufferedSerial(PA_0, PA_1)` |
| PA1 | D1 | UART4_RX |
| PA2 | D2 | UART2_TX alternative |
| PA3 | D3 | UART2_RX alternative |

> PA0/PA1 are **not exposed as `Serial4`** in the Arduino framework for Portenta H7.
> Access them via mbed: `mbed::BufferedSerial lora_uart(PA_0, PA_1, 9600);`
> Or use SoftwareSerial as a fallback (lower throughput, but sufficient for LoRa packet rate).

---

## 4th UART — MKR WAN 1310 bridge (design option)

Goal: H7 → UART → 1310 (CanSat) → LoRa → 1310 (ground station)

**Option A — mbed UART4 (recommended)**
- PA0 (TX from H7) → RX on 1310
- PA1 (RX on H7) → TX from 1310
- Use `mbed::BufferedSerial` or `arduino::UART` at 9600 baud
- 1310 firmware: receive UART packet from H7, transmit over LoRa (sandeepmistry/LoRa)
- H7 side: `lora_uart.write(frame, len)` after framing telemetry packet

**Option B — SoftwareSerial on any free GPIO**
- Works, but max ~19200 baud, no DMA
- Sufficient for 1 Hz telemetry packets

---

## Physical wiring to add tomorrow

| Wire | From | To | Notes |
|------|------|----|-------|
| GPS TX | Air530 TX | PA15 on H7 header | Yellow |
| GPS RX | Air530 RX | PF6 on H7 header | White |
| GPS VCC | 3.3 V | Air530 VCC | Red |
| GPS GND | GND | Air530 GND | Black |

> Serial2 uses PA15 (TX from H7) and PF6 (RX into H7).
> Grove Air530 is a 3.3 V device — do not connect to 5 V.

---

## Summary — resource budget

| Resource | Used | Free |
|----------|------|------|
| UART | 3 / 3 standard | 1× via mbed UART4 (PA0/PA1) |
| I2C | 1 active (Wire, M4) | Wire1 free |
| SPI | 0 (Vision Shield SPI not routed to H7 SPI pins) | Full SPI available on header |
| PWM | D4, D5 (servos) | Many remaining |
| BLE | 1 (Nicla) | — |
| WiFi | 1 (boot NTP) | — |
