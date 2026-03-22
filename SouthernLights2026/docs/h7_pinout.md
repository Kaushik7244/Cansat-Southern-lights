# Portenta H7 — CanSat Pin Assignment Map

**Board:** Arduino Portenta H7 + Vision Shield LoRa
**As of:** 2026-03-22

---

## UART / Serial

| Port | STM32 pins | Role | Device | Baud | Notes |
|------|-----------|------|--------|------|-------|
| SerialLoRa | PJ8 (TX) / PJ9 (RX) | Vision Shield Murata LoRa | MKRWAN library (M7) | 19200 8E1 | HD connector only — modem responds to `lora.begin()` but AT TX commands unsupported; LoRa TX disabled in flight build |

> APC220 and GPS are **no longer on hardware UARTs**. Both are routed through the DFRobot IIC-UART module (WK2132) on I2C — see section below.

---

## IIC-UART — DFRobot WK2132 (DFR0627) @ I2C addr 0x10

Both UART channels share one WK2132 chip on Wire (I2C3, ESLOV/Qwiic connector).
DIP switch: A1=GND, A0=GND → address 0x10.

| Channel | Object | Role | Device | Baud |
|---------|--------|------|--------|------|
| UART_1 | `iic_apc` | Telemetry downlink | APC220 433 MHz radio | 9600 |
| UART_2 | `iic_gps` | GNSS | Grove Air530 GPS | 9600 |

---

## I2C

| Bus | STM32 pins | Arduino object | Devices | Notes |
|-----|-----------|---------------|---------|-------|
| I2C3 | PH7 (SCL) / PH8 (SDA) | `Wire` | WK2132 (DFR0627) @ 0x10 — 2× UART bridge (APC220 + GPS) | ESLOV / Qwiic connector |
| I2C1 | — (internal) | `Wire1` | PMIC, fuel gauge, crypto | No external pins — not accessible from breakout header |

---

## SPI — M4 core only

SPI is used exclusively by M4. Both devices share the same bus with independent CS lines.

| Arduino pin | STM32 pin | Role | Device |
|------------|----------|------|--------|
| D7 | PI_0 | `BME_CS` — BME688 chip select | BME688 environmental sensor |
| D6 | PA_8 | `LORA_NSS` — RFM95W chip select | RFM95W 868 MHz LoRa |
| D5 | PC_6 | `LORA_DIO0` | RFM95W |
| D4 | PC_7 | `LORA_RESET` | RFM95W |

LoRa config: 868 MHz, SF9, BW125, CR4/5, +17 dBm, sync word 0x12.

> MOSI/MISO/SCK are the standard H7 SPI pins on the breakout header.

---

## Vision Shield — control pins (managed by MKRWAN library, M7)

| Pin | Role |
|-----|------|
| PC7 | `LORA_RESET` |
| PG7 | `LORA_BOOT0` |
| PJ11 | `LORA_IRQ_DUMB` |

These are driven automatically by `lora.begin()` — do not use for anything else.

---

## PWM / Servo — M7 core

| STM32 pin | Role |
|----------|------|
| PH_15 | `SERVO_PIN_LEFT` — left brake servo |
| PK_1 | `SERVO_PIN_RIGHT` — right brake servo |

Both servos default to 90° neutral at boot.

---

## Wireless (no GPIO allocation)

| Protocol | Device | Notes |
|----------|--------|-------|
| BLE | Nicla Sense ME | BHY2Host BLE — no wires from H7; barometer, temperature, orientation |
| WiFi | Internal (H7 onboard) | Disabled in flight build (`WIFI_ENABLED = false`) |

---

## Summary — resource budget

| Resource | Used | Free |
|----------|------|------|
| UART (hardware) | 1 (SerialLoRa — Vision Shield, TX disabled) | Serial1, Serial2, Serial3 on breakout header |
| IIC-UART | 2 channels (APC220, GPS) via WK2132 | — |
| I2C | Wire: WK2132 @ 0x10 (2× UART bridge) | Wire1 internal only |
| SPI | D4–D7 (BME688 + RFM95W on M4) | Full SPI available on M7 side |
| PWM | PH_15, PK_1 (servos) | Many remaining |
| BLE | 1 (Nicla Sense ME) | — |
| WiFi | Disabled | — |
