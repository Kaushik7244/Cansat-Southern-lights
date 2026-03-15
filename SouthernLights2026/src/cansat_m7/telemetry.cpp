/**
 * telemetry.cpp — SouthernLights CanSat 2026
 *
 * See telemetry.h for interface documentation and frame format.
 *
 * APC220  — Serial1 (UART1, D14/D13), 9600 bps, confirmed working pair C+D.
 * LoRa    — Vision Shield Murata module via MKRWAN + AT commands over SerialLoRa.
 *           Requires MKRWAN.h patched for SERIAL_8E1 (see cansat_lora_p2p_notes.md).
 *
 * CRC16-CCITT (poly 0x1021, init 0xFFFF) — covers LEN byte + TxPacket payload.
 */

#include "telemetry.h"
#include <MKRWAN.h>

// ---------------------------------------------------------------------------
// Radio objects
// ---------------------------------------------------------------------------

static LoRaModem lora(SerialLoRa);
static bool g_apc_ok  = false;
static bool g_lora_ok = false;

// Single public accessor — main sketch uses this instead of a parallel global flag
bool telemetryLoRaAvailable() { return g_lora_ok; }

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

    // APC220 — UART1, D14 TX / D13 RX
    // Modules C+D pre-configured to 433.920 MHz, 9600 bps (see apc220_rf7020_status.md)
    Serial1.begin(9600);
    g_apc_ok = true;
    g_packet.m7_errors += 0;   // APC220 has no init handshake to verify

    // LoRa — Vision Shield Murata module via MKRWAN AT commands
    // Requires MKRWAN.h patched with SERIAL_8E1 for Portenta (see notes)
    if (lora.begin(EU868)) {
        // Switch from LoRaWAN mode to raw P2P test mode
        // AT+TCONF sets: 868 MHz | 14 dBm | BW125 | SF7 | CR4/5 | LNA off | PABoost off
        //TODO: Verify SF7 vs SF9 for range vs data rate tradeoff in real conditions.
        SerialLoRa.print("AT+TOFF\r");          delay(500);
        SerialLoRa.print("AT+TCONF=868:14:125:9:4/5:0:0\r"); delay(500);
        g_lora_ok = true;
    } else {
        g_lora_ok = false;
        g_packet.m7_errors++;
    }

    return g_apc_ok || g_lora_ok;
}

void telemetrySend(const SensorPacket& pkt, SensorPacket& g_packet) {
    // Build frame once, send on all available radios
    uint8_t frame[sizeof(TxPacket) + 5];
    size_t  frame_len = buildFrame(pkt, frame);

    // APC220 — write raw bytes to UART1, radio handles RF transparently
    if (g_apc_ok) {
        size_t written = Serial1.write(frame, frame_len);
        if (written != frame_len) {
            g_packet.m7_errors++;
        }
    }

    // LoRa — AT+UTX sends a raw payload over the air without LoRaWAN overhead
    // Format: AT+UTX=<len>\r then write <len> raw bytes
    if (g_lora_ok) {
        // Flush any pending RX data before sending
        while (SerialLoRa.available()) SerialLoRa.read();

        SerialLoRa.print("AT+UTX=");
        SerialLoRa.print((int)frame_len);
        SerialLoRa.print("\r");
        delay(50);   // brief pause for modem to switch to receive-payload state

        size_t written = SerialLoRa.write(frame, frame_len);
        if (written != frame_len) {
            g_packet.m7_errors++;
        }

        // Wait briefly for +OK or +ERR from modem (non-blocking with timeout)
        unsigned long t = millis();
        while (millis() - t < 500) {
            if (SerialLoRa.available()) break;
        }
        // Drain response — we log errors via m7_errors, not by parsing the response
        while (SerialLoRa.available()) SerialLoRa.read();
    }
}
