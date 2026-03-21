#pragma once
/**
 * hw_config.h — SouthernLights CanSat 2026
 *
 * Single source of truth for all hardware port assignments.
 * Change here to remap a peripheral without touching any driver file.
 *
 * ── GPS ────────────────────────────────────────────────────────────────────
 * DFRobot IIC to Dual UART (DFR0627) channel 2 (UART_2).
 */
#include "iic_uart.h"
#define GPS_SERIAL  iic_gps   // DFRobot IICSerial UART_2
#define GPS_BAUD    9600

/**
 * ── APC220 radio ────────────────────────────────────────────────────────────
 * DFRobot IIC to Dual UART (DFR0627) channel 1 (UART_1), 9600 bps.
 */
#define APC_ENABLED
#define APC_SERIAL  iic_apc   // DFRobot IICSerial UART_1
#define APC_BAUD    9600

/**
 * ── I2C buses ───────────────────────────────────────────────────────────────
 * Wire  (I2C3, SDA=PH_8 / SCL=PH_7) — ESLOV + Qwiic (same physical bus)
 *   BME688 @ 0x77 (M4), WK2132 @ 0x10 (M7) — shared bus, low collision risk
 * Wire1 (I2C1, SDA=PB_7 / SCL=PB_6) — internal only: PMIC/fuel gauge/crypto
 *   No external pins accessible. BHY2Host uses BLE, not Wire1.
 */
