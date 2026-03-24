/**
 * telemetry.cpp — SouthernLights CanSat 2026
 *
 * See telemetry.h for interface documentation and frame format.
 *
 * APC220 TX via WK2132 IIC-UART bridge (DFR0627) on Wire (I2C3).
 * Direct Wire writes to FIFO address 0x11 in 32-byte chunks —
 * bypasses DFRobot IICSerial library (buggy writeFIFO with delay(10)).
 *
 * LoRa TX is handled by M4 core via shared memory (see cansat_m4/).
 *
 * CRC16-CCITT (poly 0x1021, init 0xFFFF) — covers LEN byte + TxPacket payload.
 */

#include "telemetry.h"
#include "hw_config.h"
#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------------
// State
// ---------------------------------------------------------------------------

static bool g_apc_ok = false;

// ---------------------------------------------------------------------------
// CRC16-CCITT
// Poly 0x1021, init 0xFFFF. Covers every byte between sync and CRC fields.
// The same algorithm must be used on the ground station receiver.
// ---------------------------------------------------------------------------

static uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Frame builder
// Writes the complete wire frame into buf.
// buf must be at least FRAME_PAYLOAD_LEN + 5 bytes.
// Returns total frame length.
// ---------------------------------------------------------------------------

static size_t buildFrame(const SensorPacket& pkt, uint8_t* buf) {
    // Build TxPacket from SensorPacket — copy only Tier 1 + Tier 2
    TxPacket tx;
    tx.sequence     = pkt.sequence;
    tx.timestamp_ms = pkt.timestamp_ms;
    tx.boot_epoch   = pkt.boot_epoch;
    tx.state        = pkt.state;
    tx.m4_errors    = pkt.m4_errors;
    tx.m7_errors    = pkt.m7_errors;
    tx.flags        = pkt.flags;
    tx.primary      = pkt.primary;
    tx.secondary    = pkt.secondary;

    // Frame: [SYNC1][SYNC2][LEN][...payload...][CRC_HI][CRC_LO]
    size_t idx = 0;
    buf[idx++] = FRAME_SYNC_1;
    buf[idx++] = FRAME_SYNC_2;
    buf[idx++] = FRAME_PAYLOAD_LEN;

    memcpy(&buf[idx], &tx, FRAME_PAYLOAD_LEN);
    idx += FRAME_PAYLOAD_LEN;

    // CRC covers LEN byte + payload (everything after sync, before CRC)
    uint16_t crc = crc16(&buf[2], 1 + FRAME_PAYLOAD_LEN);
    buf[idx++] = (crc >> 8) & 0xFF;   // big-endian
    buf[idx++] =  crc       & 0xFF;

    return idx;   // total frame length
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool telemetryInit(SensorPacket& g_packet) {

    // APC220 — port and baud set in hw_config.h
#ifdef APC_ENABLED
    APC_SERIAL.begin(APC_BAUD);
    g_apc_ok = true;
    g_packet.m7_errors += 0;   // APC220 has no init handshake to verify
#else
    Serial.println("[APC220] disabled via hw_config.h (APC_ENABLED not set)");
#endif

    return g_apc_ok;
}

void telemetrySend(const SensorPacket& pkt, SensorPacket& g_packet) {
    // Build frame once, send on all available radios
    uint8_t frame[sizeof(TxPacket) + 5];
    size_t  frame_len = buildFrame(pkt, frame);

    // APC220 — write to WK2132 FIFO via direct Wire in 32-byte chunks.
    // Bypasses DFRobot IICSerial library (its writeFIFO has unnecessary delay(10)
    // between chunks). WK2132 FIFO address for sub-UART ch1: 0x11.
#ifdef APC_ENABLED
    if (g_apc_ok) {
        static const uint8_t WK2132_FIFO_APC = 0x11;
        static const size_t  CHUNK = 32;

        const uint8_t* p   = frame;
        size_t         left = frame_len;
        bool           ok   = true;

        while (left) {
            size_t n = (left > CHUNK) ? CHUNK : left;
            Wire.beginTransmission(WK2132_FIFO_APC);
            Wire.write(p, n);
            if (Wire.endTransmission() != 0) { ok = false; break; }
            p    += n;
            left -= n;
        }
        if (!ok) {
            g_packet.m7_errors++;
        }
    }
#endif

}
