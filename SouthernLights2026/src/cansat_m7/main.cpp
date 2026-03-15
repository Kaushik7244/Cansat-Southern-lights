/**
 * cansat_m7/main.cpp — SouthernLights CanSat 2026
 *
 * M7 main core: WiFi/NTP, GPS, telemetry, SD storage.
 * M4 pushes sensor data via RPC → UpdateSensorPackage() → g_packet.primary.
 * M7 owns g_packet, g_ring, transmission timing, and flight state.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "arduino_secrets.h"
#include <RPC.h>
#include <time.h>
#include "data_types.h"
#include "storage.h"
#include "telemetry.h"
#include "gps.h"

#ifdef CORE_CM7

// Uncomment for verbose serial output during bench testing.
// Must be OFF for flight builds — Serial.print() wastes CPU with no USB listener.
#define SLDEBUG

// LED polarity — Portenta: LOW = on
const int ON  = LOW;
const int OFF = HIGH;

const bool WIFI_ENABLED = true;

// WiFi
char ssid[]     = SECRET_SSID;
char pass[]     = SECRET_PASS;
long WiFirssi   = -1;
int  WiFistatus = WL_IDLE_STATUS;

// RPC bookkeeping
long   mainCoreLoop      = 0;
long   rtCoreLoop        = 0;
time_t timeForLastH4tick = 0;

// Global live packet — single source of truth on M7.
// M4 fills primary fields via RPC; M7 fills secondary (GPS) directly.
SensorPacket g_packet = {};

// Ring buffer — declared extern in data_types.h, defined here.
SensorPacket     g_ring[RING_SIZE];
volatile uint8_t g_ring_head = 0;
uint8_t          g_ring_tail = 0;

// Loop timing
static uint32_t g_last_tx_ms    = 0;
static uint32_t g_last_drain_ms = 0;
static const uint32_t TX_INTERVAL_MS    = 1000;   // transmit every 1 s
static const uint32_t DRAIN_INTERVAL_MS = 10000;  // flush SD every 10 s

// Forward declarations
void printPacketToSerial();
void setH4CoreLoop(long mcl);
void UpdateSensorPackage(float temperature, float pressure, float humidity,
                         float gasresistance, float altitude);
void M4Error();
void printWifiStatus();
static void configTime(int gmtOffset_sec, int daylightOffset_sec, const char *ntpServer);

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup()
{
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, ON);   // red = booting
    digitalWrite(LEDG, OFF);
    digitalWrite(LEDB, OFF);

    Serial.begin(115200);
    while (!Serial) ;
    Serial.println("USB Serial up at 115200");

    // RPC must start before WiFi so M4 can begin sending sensor data
    Serial.println("Starting RPC");
    RPC.begin();
    RPC.bind("setH4CoreLoop",       setH4CoreLoop);
    RPC.bind("UpdateSensorPackage", UpdateSensorPackage);
    RPC.bind("M4Error",             M4Error);

    digitalWrite(LEDR, OFF);
    digitalWrite(LEDB, ON);   // blue = network/NTP in progress

    if (WIFI_ENABLED) {
        Serial.println("Starting WiFi");
        if (WiFi.status() == WL_NO_SHIELD) {
            Serial.println("WiFi module not found — halting");
            while (true) ;
        }
        while (WiFistatus != WL_CONNECTED) {
            Serial.print("Connecting to ");
            Serial.println(ssid);
            WiFistatus = WiFi.begin(ssid, pass);
            delay(3000);
        }
        Serial.println("WiFi connected");
        printWifiStatus();

        // NTP time sync — best-effort, 10 s timeout
        Serial.print("NTP sync... ");
        configTime(0, 0, "pool.ntp.org");
        time_t ntp_now = 0;
        unsigned long ntp_start = millis();
        while (ntp_now < 1700000000UL && millis() - ntp_start < 10000) {
            delay(500);
            time(&ntp_now);
        }
        if (ntp_now > 1700000000UL) {
            storageSetBootEpoch((uint32_t)(ntp_now - millis() / 1000), "NTP");
            g_packet.boot_epoch = storageGetBootEpoch();
            Serial.print("OK — epoch ");
            Serial.println((uint32_t)ntp_now);
        } else {
            Serial.println("no response — will retry via GPS or APC220 command");
        }
    } else {
        Serial.println("WiFi disabled");
    }

    // SD card
    Serial.print("SD init... ");
    if (storageInit()) {
        Serial.print("OK — flight ");
        Serial.println(storageFlightNumber());
    } else {
        Serial.println("FAILED — no SD logging this flight");
        g_packet.m7_errors++;
    }

    // Radios — APC220 UART + LoRa Vision Shield
    Serial.print("Telemetry init... ");
    if (telemetryInit(g_packet)) {
        Serial.print("OK");
        if (telemetryLoRaAvailable()) Serial.print(" (APC220 + LoRa)");
        else                          Serial.print(" (APC220 only)");
        Serial.println();
    } else {
        Serial.println("WARNING — no radio available");
    }

    // GPS
    gpsInit();

    g_packet.state = STATE_PAD;

    digitalWrite(LEDR, OFF);
    digitalWrite(LEDG, ON);   // green = ready
    digitalWrite(LEDB, OFF);

    Serial.println("Setup complete — entering loop");
}

// ---------------------------------------------------------------------------
// Main loop — non-blocking
// ---------------------------------------------------------------------------

void loop()
{
    uint32_t now_ms = millis();

    // APC220 time sync command from ground station.
    // Ground operator sends: TIME:<unix_epoch>   e.g. TIME:1742123456
    // python -c "import time,serial; s=serial.Serial('COM1',9600); s.write(b'TIME:'+str(int(time.time())).encode()+b'\n'); s.close()"
    if (Serial1.available()) {
        String cmd = Serial1.readStringUntil('\n');
        cmd.trim();
        if (cmd.startsWith("TIME:") && storageGetBootEpoch() == 0) {
            uint32_t epoch = (uint32_t)cmd.substring(5).toInt();
            if (epoch > 1700000000UL) {
                storageSetBootEpoch(epoch - now_ms / 1000, "APC");
                g_packet.boot_epoch = storageGetBootEpoch();
                Serial.print("Time synced via APC220 — epoch ");
                Serial.println(epoch);
            }
        }
    }

    // GPS — non-blocking poll; fills g_packet.secondary when fix is valid
    gpsPoll(g_packet);

    // Transmit on interval
    if (now_ms - g_last_tx_ms >= TX_INTERVAL_MS) {
        g_packet.sequence++;
        g_packet.timestamp_ms = now_ms;
        telemetrySend(g_packet, g_packet);
        g_last_tx_ms = now_ms;
        mainCoreLoop++;

#ifdef SLDEBUG
        printPacketToSerial();
#endif
    }

    // Drain SD ring buffer
    if (now_ms - g_last_drain_ms >= DRAIN_INTERVAL_MS) {
        storageDrain(g_packet);
        g_last_drain_ms = now_ms;
    }
}

// ---------------------------------------------------------------------------
// RPC callbacks — called from M4
// ---------------------------------------------------------------------------

void setH4CoreLoop(long mcl)
{
    rtCoreLoop = mcl;
    time(&timeForLastH4tick);
}

void UpdateSensorPackage(float temperature, float pressure, float humidity,
                         float gasresistance, float altitude)
{
    g_packet.primary.temperature   = temperature;
    g_packet.primary.pressure      = pressure;
    g_packet.primary.humidity      = humidity;
    g_packet.primary.altitude_baro = altitude;
    // gasresistance: no PrimaryData field — will move to TertiaryData when Nicla Sense is added
    (void)gasresistance;
}

void M4Error()
{
    g_packet.m4_errors++;
    Serial.print("ERROR from M4 (total: ");
    Serial.print(g_packet.m4_errors);
    Serial.println(")");
}

// ---------------------------------------------------------------------------
// Utilities
// ---------------------------------------------------------------------------

void printPacketToSerial()
{
    Serial.print("TX #");
    Serial.print(g_packet.sequence);
    Serial.print("  M4:");
    Serial.print(rtCoreLoop);
    Serial.print("  T:");
    Serial.print(g_packet.primary.temperature, 1);
    Serial.print("C  P:");
    Serial.print(g_packet.primary.pressure, 1);
    Serial.print("hPa  Alt:");
    Serial.print(g_packet.primary.altitude_baro, 0);
    Serial.print("m");
    if (gpsHasFix()) {
        Serial.print("  GPS:");
        Serial.print(g_packet.secondary.latitude, 5);
        Serial.print(",");
        Serial.print(g_packet.secondary.longitude, 5);
    } else {
        Serial.print("  GPS:no-fix");
    }
    Serial.println();
}

void printWifiStatus()
{
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("IP:   ");
    Serial.println(WiFi.localIP());
    WiFirssi = WiFi.RSSI();
    Serial.print("RSSI: ");
    Serial.print(WiFirssi);
    Serial.println(" dBm");
}

// ---------------------------------------------------------------------------
// configTime — Portenta H7 replacement for the ESP32 API.
// Queries NTP over UDP and sets the mbed OS system clock via settimeofday().
// Called once in setup() after WiFi is connected.
// ---------------------------------------------------------------------------
static void configTime(int gmtOffset_sec, int daylightOffset_sec, const char *ntpServer)
{
    static const uint16_t NTP_PORT = 123;
    static const uint32_t NTP_EPOCH_OFFSET = 2208988800UL; // seconds 1900→1970

    uint8_t packet[48] = {};
    packet[0] = 0b11100011; // LI=3, VN=4, Mode=3 (client request)

    WiFiUDP udp;
    udp.begin(2390);
    udp.beginPacket(ntpServer, NTP_PORT);
    udp.write(packet, sizeof(packet));
    udp.endPacket();

    unsigned long start = millis();
    while (udp.parsePacket() == 0 && millis() - start < 5000)
        delay(100);

    if (udp.available())
    {
        udp.read(packet, sizeof(packet));
        uint32_t hi = ((uint32_t)packet[40] << 24) | ((uint32_t)packet[41] << 16) | ((uint32_t)packet[42] << 8) | (uint32_t)packet[43];
        uint32_t epoch = hi - NTP_EPOCH_OFFSET + gmtOffset_sec + daylightOffset_sec;
        struct timeval tv = {(time_t)epoch, 0};
        settimeofday(&tv, nullptr);
    }
    udp.stop();
}

#endif  // CORE_CM7
