/**
 * telemetry.h — SouthernLights CanSat 2026
 *
 * APC220 (433 MHz) binary telemetry via WK2132 IIC-UART bridge.
 * LoRa (868 MHz) is handled by M4 core via shared memory.
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
 */

#pragma once
#include "data_types.h"

/**
 * Initialise APC220 UART channel on WK2132.
 * Returns true if APC220 channel is available.
 */
bool telemetryInit(SensorPacket& g_packet);

/**
 * Build a TxPacket from the current SensorPacket, frame it, and transmit
 * via APC220. Increments m7_errors on I2C write failure.
 * Non-blocking — does not wait for transmission acknowledgement.
 */
void telemetrySend(const SensorPacket& pkt, SensorPacket& g_packet);
