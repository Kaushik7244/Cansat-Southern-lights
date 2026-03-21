/**
 * iic_uart.h — SouthernLights CanSat 2026
 *
 * DFRobot IIC to Dual UART module (DFR0627, WK2132 chip) on Wire (I2C3).
 *
 * Both UART channels share one I2C base address 0x10 (A1=OFF, A0=OFF on DIP).
 * SUBUART_CHANNEL_1 → APC220 radio  (9600 bps)
 * SUBUART_CHANNEL_2 → GPS Air530    (9600 bps)
 *
 * I2C bus: Wire (I2C3, ESLOV/Qwiic connector, SDA=PH_8, SCL=PH_7).
 * Wire1 (I2C1) is the internal-only bus (PMIC/fuel gauge/crypto) — no external pins.
 * Wire is also used by M4 for BME688 (0x77) — shared bus, collision risk is low.
 */

#pragma once
#include <Wire.h>
#include <DFRobot_IICSerial.h>

// Sub-address 0 → A1=GND, A0=GND → WK2132 I2C address 0x10
extern DFRobot_IICSerial iic_apc;   // UART_1 — APC220
extern DFRobot_IICSerial iic_gps;   // UART_2 — GPS

/**
 * Initialise both IIC-UART channels. Call once in setup(), before gpsInit()
 * and telemetryInit(). Returns true if both channels respond.
 */
bool iicUartInit();
