/**
 * hw_config.h — SouthernLights CanSat 2026
 *
 * Single source of truth for all hardware port assignments.
 * Change here to remap a peripheral without touching any driver file.
 *
 * ── GPS ────────────────────────────────────────────────────────────────────
 * Normal wiring : Serial2 (D18 RX / D19 TX)
 * Temporary swap: Serial1 — used when APC220 is disconnected for bench testing.
 */
#define GPS_SERIAL  Serial1   // UART connected to Grove Air530 RX/TX
#define GPS_BAUD    9600

/**
 * ── APC220 radio ────────────────────────────────────────────────────────────
 * Normal port: Serial1 (D13 RX / D14 TX), 9600 bps.
 *
 * Comment out APC_ENABLED to disable the APC220 entirely — e.g. when GPS is
 * temporarily sharing Serial1 for bench testing.
 */
// #define APC_ENABLED
#define APC_SERIAL  Serial1
#define APC_BAUD    9600

/**
 * ── I2C buses ───────────────────────────────────────────────────────────────
 * Wire  (I2C3, SDA=D11 / SCL=D12) — BME688 on M4 (primed by M7 before M4 release)
 * Wire1 (I2C1, SDA=D0  / SCL=D1 ) — Nicla Sense ME BLE (managed by BHY2Host)
 *
 * No defines needed here — Wire and Wire1 are used directly in code.
 * Listed for reference only.
 */
