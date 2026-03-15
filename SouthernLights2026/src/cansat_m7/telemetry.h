/**
 * telemetry.h — SouthernLights CanSat 2026
 *
 * Binary radio telemetry over APC220 (433 MHz) and LoRa (868 MHz).
 *
 * Both radios transmit the same framed TxPacket — Tier 1 + Tier 2 data only.
 * Tier 3 (tertiary) data is never transmitted; it is SD-only.
 *
 * Wire frame format:
 *   [0xAA][0x55][LEN:1][...TxPacket...][CRC16_HI:1][CRC16_LO:1]
 *
 *   SYNC bytes  — allow receiver to find packet boundary in a noisy stream
 *   LEN         — payload byte count (sizeof TxPacket), fixed but included for safety
 *   CRC16       — covers LEN + payload, big-endian, poly 0x1021 (CCITT)
 *                 Receiver discards silently on mismatch — no retransmit
 *
 * Usage:
 *   telemetryInit()        once in setup()
 *   telemetrySend(packet)  called by main loop after stamping sequence + timestamp
 *
 * APC220 is primary. LoRa is secondary — skipped if not available.
 * Call telemetryLoRaAvailable() anywhere in the main sketch to check LoRa status.
 * Both are attempted on each send; failure of one does not block the other.
 */

#pragma once
#include "data_types.h"

/**
 * Initialise APC220 UART. Initialise LoRa modem if Vision Shield is present.
 * Increments m7_errors if LoRa init fails.
 * Returns true if at least one radio initialised successfully.
 */
bool telemetryInit(SensorPacket& g_packet);

/**
 * Returns true if LoRa modem initialised successfully.
 * Use this instead of a separate gLoRaAvailable flag in the main sketch —
 * telemetry.cpp is the single owner of LoRa state.
 */
bool telemetryLoRaAvailable();

/**
 * Build a TxPacket from the current SensorPacket, frame it, and transmit
 * on all available radios. Increments m7_errors on any send failure.
 * Non-blocking — does not wait for transmission acknowledgement.
 */
void telemetrySend(const SensorPacket& pkt, SensorPacket& g_packet);
