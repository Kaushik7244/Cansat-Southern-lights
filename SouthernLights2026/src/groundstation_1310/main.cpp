/**
 * cansat_basestation_1310 — SouthernLights CanSat 2026
 *
 * Ground station receiver for MKR WAN 1310.
 * Uses arduino-LoRa (sandeepmistry) which handles MKR WAN 1310 dumb mode
 * automatically — no manual pin manipulation required.
 *
 * Listens for binary-framed TxPackets transmitted by the CanSat.
 * Outputs two lines per received packet to Serial (115200 baud):
 *   1. Human-readable summary  — for live monitoring in Serial Monitor
 *   2. CSV line prefixed "CSV," — pipe Serial output to file for logging
 *
 * ============================================================
 * FIRST TIME SETUP — verify RF link
 * ============================================================
 * 1. Upload this sketch, open Serial Monitor at 115200 baud.
 * 2. Power the CanSat. Packets should appear within a few seconds.
 * 3. If nothing arrives after 30s, try changing LORA_SYNC_WORD to 0x34 —
 *    the Murata firmware may default to LoRaWAN sync word rather than private.
 * 4. RSSI better than -120 dBm and SNR > -10 dB = healthy link.
 * ============================================================
 *
 * RF settings must match cansat_m7/telemetry.cpp exactly:
 *   868.0 MHz | BW 125 kHz | SF 9 | CR 4/5 | sync word 0x12
 */

#include <SPI.h>
#include <LoRa.h>
#include "data_types.h"

static const long  LORA_FREQ      = 868000000L;
static const int   LORA_SF        = 9;
static const long  LORA_BW        = 125000L;
static const int   LORA_CR        = 5;            // 4/5
static const int   LORA_SYNC_WORD = 0x12;  // private LoRa — matches future dumb-mode TX
static const int   LORA_PREAMBLE  = 8;

// ---------------------------------------------------------------------------
// CRC16-CCITT — identical to telemetry.cpp
// Poly 0x1021, init 0xFFFF, covers LEN byte + TxPacket payload
// ---------------------------------------------------------------------------
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Frame parser
// ---------------------------------------------------------------------------
static bool parseFrame(const uint8_t *buf, size_t len, TxPacket &tx)
{
    const size_t MIN_FRAME = 5 + FRAME_PAYLOAD_LEN;
    if (len < MIN_FRAME)            return false;
    if (buf[0] != FRAME_SYNC_1)     return false;
    if (buf[1] != FRAME_SYNC_2)     return false;
    uint8_t payload_len = buf[2];
    if (payload_len != FRAME_PAYLOAD_LEN)         return false;
    if (len < (size_t)(3 + payload_len + 2))      return false;

    uint16_t expected = crc16(&buf[2], 1 + payload_len);
    uint16_t received = ((uint16_t)buf[3 + payload_len] << 8)
                      |  (uint16_t)buf[4 + payload_len];
    if (expected != received) return false;

    memcpy(&tx, &buf[3], sizeof(TxPacket));
    return true;
}

// ---------------------------------------------------------------------------
// Packet loss tracking
// ---------------------------------------------------------------------------
static uint16_t g_last_seq   = 0;
static uint32_t g_last_ts    = 0;
static uint32_t g_total_rx   = 0;
static uint32_t g_total_dup  = 0;
static uint32_t g_crc_errors = 0;

// ---------------------------------------------------------------------------
// Output helpers
// ---------------------------------------------------------------------------
static const char *stateToStr(FlightState s)
{
    switch (s)
    {
    case STATE_BOOT:    return "BOOT";
    case STATE_PAD:     return "PAD";
    case STATE_ASCENT:  return "ASCENT";
    case STATE_APOGEE:  return "APOGEE";
    case STATE_DESCENT: return "DESCENT";
    case STATE_LANDED:  return "LANDED";
    default:            return "UNKNOWN";
    }
}

static void printCSVHeader()
{
    Serial.println(
        "CSV,seq,ts_ms,utc_ts,utc_time,state,"
        "temp1_C,press1_hPa,hum_pct,alt_baro_m,temp2_C,press2_hPa,"
        "lat,lon,alt_gps_m,speed_ms,heading_deg,gps_sats,gps_fix,"
        "m4_errors,m7_errors,rssi_dbm,snr_db");
}

static void printPacket(const TxPacket &tx, float rssi, float snr)
{
    uint32_t utc_ts = 0;
    if (tx.boot_epoch > 0)
        utc_ts = tx.boot_epoch + tx.timestamp_ms / 1000;

    // Human-readable summary
    Serial.print("--- Pkt #"); Serial.print(tx.sequence);
    Serial.print("  "); Serial.print(tx.timestamp_ms); Serial.print("ms  UTC ");
    if (utc_ts > 0) {
        uint32_t h = (utc_ts % 86400) / 3600;
        uint32_t m = (utc_ts % 3600)  / 60;
        uint32_t s =  utc_ts % 60;
        if (h < 10) Serial.print('0'); Serial.print(h); Serial.print(':');
        if (m < 10) Serial.print('0'); Serial.print(m); Serial.print(':');
        if (s < 10) Serial.print('0'); Serial.print(s);
    } else {
        Serial.print("--:--:--");
    }
    Serial.print("  "); Serial.print(stateToStr(tx.state));
    Serial.print("  RSSI "); Serial.print(rssi, 0); Serial.print("dBm SNR ");
    Serial.print(snr, 1); Serial.print("dB  [RX:"); Serial.print(g_total_rx);
    Serial.print(" DUP:"); Serial.print(g_total_dup);
    Serial.print(" CRC_ERR:"); Serial.print(g_crc_errors); Serial.println("] ---");

    Serial.print("  T1:"); Serial.print(tx.primary.temperature, 2);
    Serial.print("C  P1:"); Serial.print(tx.primary.pressure, 2);
    Serial.print("hPa  Hum:"); Serial.print(tx.primary.humidity, 1);
    Serial.print("%  Alt:"); Serial.print(tx.primary.altitude_baro, 1); Serial.println("m");

    Serial.print("  T2:"); Serial.print(tx.primary.temperature2, 2);
    Serial.print("C  P2:"); Serial.print(tx.primary.pressure2, 2);
    Serial.print("hPa");
    if (!(tx.flags & FLAG_NICLA_BLE_OK)) Serial.print("  [Nicla BLE lost]");
    Serial.println();

    if (tx.secondary.gps_fix) {
        Serial.print("  GPS:");
        Serial.print(tx.secondary.latitude, 6);  Serial.print("  ");
        Serial.print(tx.secondary.longitude, 6); Serial.print("  ");
        Serial.print(tx.secondary.altitude_gps, 1); Serial.print("m  ");
        Serial.print(tx.secondary.speed_ms, 2);  Serial.print("m/s  ");
        Serial.print(tx.secondary.heading, 1);   Serial.print("deg  ");
        Serial.print(tx.secondary.gps_satellites); Serial.println("sats");
    } else {
        Serial.print("  GPS: no fix  (");
        Serial.print(tx.secondary.gps_satellites); Serial.println(" sats visible)");
    }

    if (tx.m4_errors || tx.m7_errors) {
        Serial.print("  *** ERRORS  M4:"); Serial.print(tx.m4_errors);
        Serial.print("  M7:"); Serial.print(tx.m7_errors); Serial.println(" ***");
    }

    // CSV line
    Serial.print("CSV,");
    Serial.print(tx.sequence); Serial.print(',');
    Serial.print(tx.timestamp_ms); Serial.print(',');
    Serial.print(utc_ts); Serial.print(',');
    if (utc_ts > 0) {
        uint32_t h = (utc_ts % 86400) / 3600;
        uint32_t m = (utc_ts % 3600)  / 60;
        uint32_t s =  utc_ts % 60;
        if (h < 10) Serial.print('0'); Serial.print(h); Serial.print(':');
        if (m < 10) Serial.print('0'); Serial.print(m); Serial.print(':');
        if (s < 10) Serial.print('0'); Serial.print(s);
    } else {
        Serial.print("--:--:--");
    }
    Serial.print(','); Serial.print(stateToStr(tx.state)); Serial.print(',');
    Serial.print(tx.primary.temperature, 2);  Serial.print(',');
    Serial.print(tx.primary.pressure, 2);     Serial.print(',');
    Serial.print(tx.primary.humidity, 2);     Serial.print(',');
    Serial.print(tx.primary.altitude_baro, 1); Serial.print(',');
    Serial.print(tx.primary.temperature2, 2); Serial.print(',');
    Serial.print(tx.primary.pressure2, 2);    Serial.print(',');
    Serial.print(tx.secondary.latitude, 6);   Serial.print(',');
    Serial.print(tx.secondary.longitude, 6);  Serial.print(',');
    Serial.print(tx.secondary.altitude_gps, 1); Serial.print(',');
    Serial.print(tx.secondary.speed_ms, 2);   Serial.print(',');
    Serial.print(tx.secondary.heading, 1);    Serial.print(',');
    Serial.print(tx.secondary.gps_satellites); Serial.print(',');
    Serial.print(tx.secondary.gps_fix ? 1 : 0); Serial.print(',');
    Serial.print(tx.m4_errors);  Serial.print(',');
    Serial.print(tx.m7_errors);  Serial.print(',');
    Serial.print(rssi, 0);       Serial.print(',');
    Serial.println(snr, 1);
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    while (!Serial);

    Serial.println("=== SouthernLights Base Station ===");
    Serial.print("Initialising LoRa (MKR WAN 1310)... ");

    if (!LoRa.begin(LORA_FREQ))
    {
        Serial.println("FAILED");
        while (true);
    }

    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.setPreambleLength(LORA_PREAMBLE);

    Serial.println("OK");
    Serial.print("868MHz BW125 SF"); Serial.print(LORA_SF);
    Serial.print(" CR4/5 sync=0x"); Serial.println(LORA_SYNC_WORD, HEX);
    Serial.print("sizeof(TxPacket)="); Serial.print(sizeof(TxPacket));
    Serial.print("  FRAME_PAYLOAD_LEN="); Serial.println(FRAME_PAYLOAD_LEN);
    Serial.println("Listening...\n");

    printCSVHeader();
}

// ---------------------------------------------------------------------------
// Main loop — polling receive, print stats every 10 s when idle
// ---------------------------------------------------------------------------
static uint32_t g_last_stats_ms = 0;

void loop()
{
    int packetSize = LoRa.parsePacket();

    if (packetSize > 0)
    {
        // Read raw bytes
        uint8_t raw[packetSize];
        for (int i = 0; i < packetSize; i++)
            raw[i] = (uint8_t)LoRa.read();

        TxPacket tx;
        if (parseFrame(raw, packetSize, tx))
        {
            float rssi = LoRa.packetRssi();
            float snr  = LoRa.packetSnr();

            // SX1276 continuous RX can trigger RX_DONE twice for the same
            // packet.  Skip exact duplicates (same seq AND same timestamp).
            if (g_total_rx > 0 && tx.sequence == g_last_seq
                && tx.timestamp_ms == g_last_ts)
            {
                g_total_dup++;
                g_total_rx++;
                return;   // back to top of loop()
            }

            g_total_rx++;

            if (g_total_rx > 1)
            {
                uint16_t gap = (uint16_t)(tx.sequence - g_last_seq - 1);
                if (gap > 0 && gap < 1000)
                {
                    Serial.print("*** SEQUENCE GAP ");
                    Serial.print(gap);
                    Serial.println(" packet(s) missed ***");
                }
            }
            g_last_seq = tx.sequence;
            g_last_ts  = tx.timestamp_ms;
            printPacket(tx, rssi, snr);
        }
        else
        {
            g_crc_errors++;
            Serial.print("CRC/frame error (");
            Serial.print(packetSize);
            Serial.print(" bytes) hex: ");
            // Dump first 16 bytes + last 4 bytes for debugging
            int dump_n = (packetSize < 16) ? packetSize : 16;
            for (int i = 0; i < dump_n; i++) {
                if (raw[i] < 0x10) Serial.print('0');
                Serial.print(raw[i], HEX);
                Serial.print(' ');
            }
            if (packetSize > 20) {
                Serial.print("... ");
                for (int i = packetSize - 4; i < packetSize; i++) {
                    if (raw[i] < 0x10) Serial.print('0');
                    Serial.print(raw[i], HEX);
                    Serial.print(' ');
                }
            }
            Serial.println();
        }
    }
    else
    {
        // Print alive stats + channel RSSI every 5 s (RSSI shows if any RF energy present)
        if (millis() - g_last_stats_ms >= 5000)
        {
            Serial.print("=== alive | RX:");
            Serial.print(g_total_rx);
            Serial.print(" DUP:");
            Serial.print(g_total_dup);
            Serial.print(" CRC_ERR:");
            Serial.print(g_crc_errors);
            Serial.println(" ===");
            g_last_stats_ms = millis();
        }
    }
}
