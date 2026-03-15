/**
 * cansat_basestation_1310.ino — SouthernLights CanSat 2026
 *
 * Ground station receiver for MKR WAN 1310.
 * MKR WAN 1310 has direct SPI access to SX1276 — RadioLib used here directly.
 * (Unlike the Portenta Vision Shield which requires UART/AT commands.)
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
 *   868.0 MHz | BW 125 kHz | SF 7 | CR 4/5 | sync word 0x12
 */

#include <RadioLib.h>
#include "data_types.h"

// ---------------------------------------------------------------------------
// Radio — SX1276 on MKR WAN 1310 (pins fixed by board hardware)
// ---------------------------------------------------------------------------
SX1276 radio = new Module(7, 1, 2, -1); // CS, IRQ, RST, no BUSY on SX1276

static const float LORA_FREQ = 868.0;
static const float LORA_BW = 125.0;
static const uint8_t LORA_SF = 9;
static const uint8_t LORA_CR = 5;           // 4/5
static const uint8_t LORA_SYNC_WORD = 0x12; // try 0x34 if no link established
static const int8_t LORA_POWER = 10;        // required by begin(), not used in RX
static const uint16_t LORA_PREAMBLE = 8;

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
        {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Frame parser
// Validates sync bytes, LEN, and CRC16. Copies payload into tx on success.
// ---------------------------------------------------------------------------
static bool parseFrame(const uint8_t *buf, size_t len, TxPacket &tx)
{
    const size_t MIN_FRAME = 5 + FRAME_PAYLOAD_LEN;
    if (len < MIN_FRAME)
        return false;
    if (buf[0] != FRAME_SYNC_1)
        return false;
    if (buf[1] != FRAME_SYNC_2)
        return false;
    uint8_t payload_len = buf[2];
    if (payload_len != FRAME_PAYLOAD_LEN)
        return false;
    if (len < (size_t)(3 + payload_len + 2))
        return false;

    uint16_t expected = crc16(&buf[2], 1 + payload_len);
    uint16_t received = ((uint16_t)buf[3 + payload_len] << 8) | (uint16_t)buf[4 + payload_len];
    if (expected != received)
        return false;

    memcpy(&tx, &buf[3], sizeof(TxPacket));
    return true;
}

// ---------------------------------------------------------------------------
// Output
// ---------------------------------------------------------------------------
static const char *stateToStr(FlightState s)
{
    switch (s)
    {
    case STATE_BOOT:
        return "BOOT";
    case STATE_PAD:
        return "PAD";
    case STATE_ASCENT:
        return "ASCENT";
    case STATE_APOGEE:
        return "APOGEE";
    case STATE_DESCENT:
        return "DESCENT";
    case STATE_LANDED:
        return "LANDED";
    default:
        return "UNKNOWN";
    }
}

static void printCSVHeader()
{
    Serial.println(
        "CSV,seq,ts_ms,utc_ts,state,"
        "temp1,press1,hum,alt_baro,temp2,press2,"
        "lat,lon,alt_gps,speed_ms,heading,gps_sats,gps_fix,"
        "m4_err,m7_err,rssi_dbm,snr_db");
}

static void printPacket(const TxPacket &tx, float rssi, float snr)
{
    // Human-readable summary
    Serial.print("--- Pkt ");
    Serial.print(tx.sequence);
    Serial.print(" | ");
    Serial.print(tx.timestamp_ms);
    Serial.print("ms");
    if (tx.boot_epoch > 0)
    {
        uint32_t utc = tx.boot_epoch + tx.timestamp_ms / 1000;
        // Print as HH:MM:SS (UTC) — simple modulo arithmetic, no library needed
        uint32_t h = (utc % 86400) / 3600;
        uint32_t m = (utc % 3600) / 60;
        uint32_t s = utc % 60;
        Serial.print(" UTC ");
        if (h < 10)
            Serial.print('0');
        Serial.print(h);
        Serial.print(':');
        if (m < 10)
            Serial.print('0');
        Serial.print(m);
        Serial.print(':');
        if (s < 10)
            Serial.print('0');
        Serial.print(s);
    }
    else
    {
        Serial.print(" (no time sync)");
    }
    Serial.print(" | ");
    Serial.print(stateToStr(tx.state));
    Serial.print(" | RSSI ");
    Serial.print(rssi, 0);
    Serial.print("dBm SNR ");
    Serial.print(snr, 1);
    Serial.println("dB ---");

    Serial.print("  T1:");
    Serial.print(tx.primary.temperature, 2);
    Serial.print("C  P1:");
    Serial.print(tx.primary.pressure, 2);
    Serial.print("hPa  Hum:");
    Serial.print(tx.primary.humidity, 1);
    Serial.print("%  Alt:");
    Serial.print(tx.primary.altitude_baro, 1);
    Serial.println("m");

    Serial.print("  T2:");
    Serial.print(tx.primary.temperature2, 2);
    Serial.print("C  P2:");
    Serial.print(tx.primary.pressure2, 2);
    Serial.println("hPa");

    if (tx.secondary.gps_fix)
    {
        Serial.print("  GPS:");
        Serial.print(tx.secondary.latitude, 6);
        Serial.print("  ");
        Serial.print(tx.secondary.longitude, 6);
        Serial.print("  ");
        Serial.print(tx.secondary.altitude_gps, 1);
        Serial.print("m  ");
        Serial.print(tx.secondary.speed_ms, 2);
        Serial.print("m/s  ");
        Serial.print(tx.secondary.heading, 1);
        Serial.print("deg  ");
        Serial.print(tx.secondary.gps_satellites);
        Serial.println("sats");
    }
    else
    {
        Serial.println("  GPS: no fix");
    }

    if (tx.m4_errors || tx.m7_errors)
    {
        Serial.print("  *** ERRORS  M4:");
        Serial.print(tx.m4_errors);
        Serial.print("  M7:");
        Serial.print(tx.m7_errors);
        Serial.println(" ***");
    }

    // CSV line — prefix "CSV," lets a logging script filter these lines
    Serial.print("CSV,");
    Serial.print(tx.sequence);
    Serial.print(',');
    Serial.print(tx.timestamp_ms);
    Serial.print(',');
    uint32_t utc_ts = (tx.boot_epoch > 0) ? tx.boot_epoch + tx.timestamp_ms / 1000 : 0;
    Serial.print(utc_ts);
    Serial.print(',');
    Serial.print(stateToStr(tx.state));
    Serial.print(',');
    Serial.print(tx.primary.temperature, 2);
    Serial.print(',');
    Serial.print(tx.primary.pressure, 2);
    Serial.print(',');
    Serial.print(tx.primary.humidity, 2);
    Serial.print(',');
    Serial.print(tx.primary.altitude_baro, 1);
    Serial.print(',');
    Serial.print(tx.primary.temperature2, 2);
    Serial.print(',');
    Serial.print(tx.primary.pressure2, 2);
    Serial.print(',');
    Serial.print(tx.secondary.latitude, 6);
    Serial.print(',');
    Serial.print(tx.secondary.longitude, 6);
    Serial.print(',');
    Serial.print(tx.secondary.altitude_gps, 1);
    Serial.print(',');
    Serial.print(tx.secondary.speed_ms, 2);
    Serial.print(',');
    Serial.print(tx.secondary.heading, 1);
    Serial.print(',');
    Serial.print(tx.secondary.gps_satellites);
    Serial.print(',');
    Serial.print(tx.secondary.gps_fix ? 1 : 0);
    Serial.print(',');
    Serial.print(tx.m4_errors);
    Serial.print(',');
    Serial.print(tx.m7_errors);
    Serial.print(',');
    Serial.print(rssi, 0);
    Serial.print(',');
    Serial.println(snr, 1);
}

// ---------------------------------------------------------------------------
// Packet loss tracking
// ---------------------------------------------------------------------------
static uint16_t g_last_seq = 0;
static uint32_t g_total_rx = 0;
static uint32_t g_crc_errors = 0;

static void printStats()
{
    Serial.print("=== alive | RX:");
    Serial.print(g_total_rx);
    Serial.print(" CRC_ERR:");
    Serial.print(g_crc_errors);
    Serial.println(" ===");
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    Serial.println("=== SouthernLights Base Station ===");
    Serial.print("Initialising SX1276 on MKR WAN 1310... ");

    int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                            LORA_SYNC_WORD, LORA_POWER, LORA_PREAMBLE);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("FAILED code ");
        Serial.println(state);
        while (true)
            ;
    }
    Serial.println("OK");
    Serial.print("868MHz BW125 SF");
    Serial.print(LORA_SF);
    Serial.print(" CR4/5 sync=0x");
    Serial.println(LORA_SYNC_WORD, HEX);
    Serial.println("Listening...\n");

    printCSVHeader();
}

// ---------------------------------------------------------------------------
// Main loop — blocking receive with timeout, print stats when idle
// ---------------------------------------------------------------------------
void loop()
{
    uint8_t raw[sizeof(TxPacket) + 8]; // frame size + headroom
    int state = radio.receive(raw, sizeof(raw));

    if (state == RADIOLIB_ERR_NONE)
    {
        TxPacket tx;
        if (parseFrame(raw, radio.getPacketLength(), tx))
        {
            float rssi = radio.getRSSI();
            float snr = radio.getSNR();
            g_total_rx++;

            if (g_total_rx > 1)
            {
                uint16_t gap = (uint16_t)(tx.sequence - g_last_seq - 1);
                if (gap > 0 && gap < 1000)
                { // guard against first-packet wrap
                    Serial.print("*** SEQUENCE GAP ");
                    Serial.print(gap);
                    Serial.println(" packet(s) missed ***");
                }
            }
            g_last_seq = tx.sequence;
            printPacket(tx, rssi, snr);
        }
        else
        {
            g_crc_errors++;
            Serial.print("CRC/frame error (");
            Serial.print(radio.getPacketLength());
            Serial.println(" bytes)");
        }
    }
    else if (state == RADIOLIB_ERR_RX_TIMEOUT)
    {
        printStats(); // normal idle — shows base station is alive
    }
    else
    {
        Serial.print("RX error code ");
        Serial.println(state);
    }
}
