/**
 * cansat_m7/main.cpp — SouthernLights CanSat 2026
 *
 * M7 main core: WiFi/NTP, GPS, Nicla Sense, navigation, telemetry, SD storage.
 * M4 pushes BME688 sensor data via RPC → UpdateSensorPackage() → g_packet.primary.
 * M7 owns g_packet, g_ring, flight state, servo control, and transmission timing.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include "arduino_secrets.h"
#include <RPC.h>
#include <time.h>
#include <Arduino_BHY2Host.h>
#include <Servo.h>
#include "data_types.h"
#include "storage.h"
#include "telemetry.h"
#include "gps.h"
#include "Altitude.h"
#include "Go_to_checkpoint.h"

#ifdef CORE_CM7

// Uncomment for verbose serial output during bench testing.
// Must be OFF for flight builds — Serial.print() wastes CPU with no USB listener.
#define SLDEBUG

// Uncomment to strip all peripherals except Portenta H7 + Nicla Sense ME.
// Disables WiFi, SD, telemetry, GPS, and M4 (M4 idles after RPC.begin()).
// Use when physically disconnecting APC220, BME688, and Vision Shield.
// NOTE: Nicla communicates via BLE in all modes — ESLOV I2C is no longer used.
//#define ISOLATION_TEST

// ---------------------------------------------------------------------------
// LED polarity — Portenta: LOW = on
// ---------------------------------------------------------------------------
const int ON  = LOW;
const int OFF = HIGH;

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
const bool WIFI_ENABLED = true;
char ssid[]     = SECRET_SSID;
char pass[]     = SECRET_PASS;
long WiFirssi   = -1;
int  WiFistatus = WL_IDLE_STATUS;

// ---------------------------------------------------------------------------
// Navigation — hardware
// ---------------------------------------------------------------------------
#define SERVO_PIN_LEFT  PH_15
#define SERVO_PIN_RIGHT PK_1
#define LEFT_NEUTRAL   180
#define RIGHT_NEUTRAL  0
#define SERVO_FOLDED     90
#define SERVO_MAX_PULL 90
static const float TURNING_RATE  = 0.8f;

Servo break_left;
Servo break_right;

// Nicla Sense ME sensors via BHY2Host
Sensor          nicla_barometer(SENSOR_ID_BARO);
Sensor          nicla_temp(SENSOR_ID_TEMP);
SensorQuaternion nicla_ori(SENSOR_ID_ORI);

// ---------------------------------------------------------------------------
// Navigation — state
// ---------------------------------------------------------------------------

// Checkpoints — lat/lon pairs. {0,0} entries are ignored.
// UPDATE THESE BEFORE EACH FLIGHT.
struct Checkpoint { float x; float y; };
static const Checkpoint arr[100] = {
    {59.80796f, 10.44519f},
    {59.80634f, 10.45074f},
    {59.80694f, 10.45174f},
    // remaining entries zero-initialised — treated as end-of-list
};

Go_to_checkpoint Target(arr[0].x, arr[0].y);
int              checkpoints_counter = 0;

Altitude alt;
float    nicla_altitude = 0.0f;
float    yaw_deg        = 0.0f;
bool     nicla_active   = false;   // set true only when BHY2Host init succeeded
bool     alt_init       = false;
bool     crossed_500    = false;
bool     parachute_deployed = false;
bool     flight_end     = false;

// ---------------------------------------------------------------------------
// RPC bookkeeping
// ---------------------------------------------------------------------------
long   mainCoreLoop      = 0;
long   rtCoreLoop        = 0;
time_t timeForLastH4tick = 0;

// ---------------------------------------------------------------------------
// Global live packet + ring buffer
// ---------------------------------------------------------------------------
SensorPacket     g_packet    = {};
SensorPacket     g_ring[RING_SIZE];
volatile uint8_t g_ring_head = 0;
uint8_t          g_ring_tail = 0;

// ---------------------------------------------------------------------------
// Loop timing
// ---------------------------------------------------------------------------
static uint32_t g_last_tx_ms    = 0;
static uint32_t g_last_drain_ms = 0;
static uint32_t g_last_nicla_ms = 0;
static const uint32_t TX_INTERVAL_MS    = 1000;   // transmit every 1 s
static const uint32_t DRAIN_INTERVAL_MS = 10000;  // flush SD every 10 s
static const uint32_t NICLA_INTERVAL_MS = 200;    // poll Nicla at 5 Hz — limits I2C bus usage

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void navigationSetup();
void niclaUpdate();
void navigationUpdate();
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
    Serial.print("RPC init...");
    RPC.begin();
    RPC.bind("setH4CoreLoop",       setH4CoreLoop);
    RPC.bind("UpdateSensorPackage", UpdateSensorPackage);
    RPC.bind("M4Error",             M4Error);
    Serial.println("OK");

    digitalWrite(LEDR, OFF);
    digitalWrite(LEDB, ON);   // blue = network/NTP in progress

#ifndef ISOLATION_TEST
    if (WIFI_ENABLED) {
        Serial.println("WiFi init...");
        if (WiFi.status() == WL_NO_SHIELD) {
            Serial.println("WiFi module not found — halting");
            while (true) ;
        }
        while (WiFistatus != WL_CONNECTED) {
            Serial.print("  Connecting to ");
            Serial.println(ssid);
            WiFistatus = WiFi.begin(ssid, pass);
            delay(3000);
        }
        Serial.println("  WiFi connected");
        printWifiStatus();

        // NTP time sync — best-effort, 10 s timeout
        Serial.print("  NTP sync... ");
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
            Serial.print("  OK — epoch ");
            Serial.println((uint32_t)ntp_now);
        } else {
            Serial.println("  no response — will retry via GPS or APC220 command");
        }
    } else {
        Serial.println("  WiFi disabled");
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
#endif  // ISOLATION_TEST

    // Nicla Sense + servos
    navigationSetup();

    g_packet.state = STATE_PAD;

#ifndef ISOLATION_TEST
    // Prime Wire (I2C3) for M4's BME688 access.
    // EslovHandler now uses Wire1 (I2C1) for Nicla, so Wire is not touched by
    // Nicla init. M7 must prime Wire before releasing M4 regardless of Nicla state.
    {
        Wire.begin();
        Wire.setClock(400000);
        delay(50);
        for (uint8_t i = 0; i < 20; i++) {
            Wire.beginTransmission(0x77);
            Wire.write(0xD0);
            Wire.endTransmission();
            Wire.requestFrom((uint8_t)0x77, (uint8_t)1);
            while (Wire.available()) Wire.read();
            delay(5);
        }
        Serial.println("Wire primed for M4 (BME688)");
    }

    Serial.println("Signaling M4 Core to start sensor operations");
    RPC.call("M7Ready"); // release M4 to start I2C / BME688 init
#else
    Serial.println("ISOLATION_TEST: M4 kept idle, Wire priming skipped");
#endif

    digitalWrite(LEDR, OFF);
    digitalWrite(LEDG, ON);   // green = ready
    digitalWrite(LEDB, OFF);



#ifdef SLDEBUG
    // RF link verification — sends plain ASCII so PuTTY can confirm data arrives.
    // If "APC220 TEST" appears on COM14, the radio link is good.
    // Remove before flight build (SLDEBUG must be off).
    Serial1.println("APC220 TEST");
    Serial.println("APC220 TEST sent on Serial1");
#endif

    Serial.println("Setup complete — entering loop");
    
}

// ---------------------------------------------------------------------------
// Main loop — non-blocking
// ---------------------------------------------------------------------------

void loop()
{
    uint32_t now_ms = millis();

#ifndef ISOLATION_TEST
    // Forward M4 RPC.print() output to USB Serial
    while (RPC.available()) Serial.write(RPC.read());

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
        } else if (cmd == "DUMP") {
            // Ground operator command — dumps full M4 in-memory log to USB Serial as CSV
            Serial.println("DUMP requested — triggering M4 log dump");
            RPC.call("DumpM4Log");
        }
    }
#endif  // ISOLATION_TEST

    // Nicla Sense — poll BLE at 5 Hz. Sensor data arrives at 10 Hz via BLE notifications;
    // 200 ms polling interval is sufficient for navigation updates.
    if (nicla_active && now_ms - g_last_nicla_ms >= NICLA_INTERVAL_MS) {
        niclaUpdate();
        g_last_nicla_ms = now_ms;
    }

#ifndef ISOLATION_TEST
    // GPS — non-blocking poll; fills g_packet.secondary when fix is valid
    gpsPoll(g_packet);
#endif

    // Navigation — flight state machine and servo control
    navigationUpdate();

    // Transmit on interval
    if (now_ms - g_last_tx_ms >= TX_INTERVAL_MS) {
        g_packet.sequence++;
        g_packet.timestamp_ms = now_ms;

        // Push snapshot to ring buffer before transmitting — storageDrain() reads from here
        g_ring[g_ring_head % RING_SIZE] = g_packet;
        g_ring_head++;

#ifndef ISOLATION_TEST
        telemetrySend(g_packet, g_packet);
#endif
        g_last_tx_ms = now_ms;
        mainCoreLoop++;

#ifdef SLDEBUG
        printPacketToSerial();
#endif
    }

#ifndef ISOLATION_TEST
    // Drain SD ring buffer
    if (now_ms - g_last_drain_ms >= DRAIN_INTERVAL_MS) {
        storageDrain(g_packet);
        g_last_drain_ms = now_ms;
    }
#endif
}

// ---------------------------------------------------------------------------
// Navigation
// ---------------------------------------------------------------------------

void navigationSetup()
{
    // BHY2Host connects to the Nicla Sense ME over BLE.
    // The Nicla must be running cansat_nicla firmware (NICLA_STANDALONE mode),
    // advertising as "NICLA". begin() blocks until the device is found — power
    // the Nicla before the Portenta reaches this point.
    Serial.print("Nicla Sense (BLE) connecting... ");
    if (BHY2Host.begin(false, NICLA_VIA_BLE)) {
        nicla_barometer.begin(10, 0);
        nicla_temp.begin(10, 0);
        nicla_ori.begin(10, 0);
        // Wait up to 5 s for first data to arrive over BLE
        Serial.print("waiting for data... ");
        unsigned long t0 = millis();
        while (nicla_temp.value() == 0.0f && millis() - t0 < 5000) {
            BHY2Host.update();
            delay(50);
        }
        nicla_active = (nicla_temp.value() != 0.0f);
        Serial.println(nicla_active ? "OK" : "connected but no data yet — continuing");
    } else {
        Serial.println("FAILED — Nicla not found over BLE");
    }

    // Servos start with brakes open (awaiting parachute deployment trigger)
    Serial.print("Servos init... ");
    break_left.attach(SERVO_PIN_LEFT);
    break_right.attach(SERVO_PIN_RIGHT);
    break_left.write(SERVO_FOLDED);
    break_right.write(SERVO_FOLDED);
    Serial.println("OK");
}

void niclaUpdate()
{
    // update() calls BLE.poll() which processes any pending BLE notifications
    // from the Nicla. Sensor values are updated by the notification callbacks —
    // read .value() immediately after.
    BHY2Host.update();

    // Always read latest values — returns 0.0 until Nicla sends its first packet.
    float t2 = nicla_temp.value();
    float p2 = nicla_barometer.value() / 100.0f;  // Pa → hPa

    // Outlier rejection — same bounds as BME688. A single corrupt BLE packet
    // can produce physically impossible values that would falsely trigger the
    // flight state machine (ascent/apogee/parachute deploy).
    if (t2 >= -50.0f && t2 <= 85.0f)   g_packet.primary.temperature2 = t2;
    if (p2 >= 300.0f  && p2 <= 1100.0f) g_packet.primary.pressure2    = p2;

#ifdef SLDEBUG
    static uint8_t nicla_dbg_count = 0;
    if (nicla_dbg_count < 3) {
        char buf[60];
        snprintf(buf, sizeof(buf), "Nicla: T2=%.2f P2=%.2f w=%.2f\r\n",
                 g_packet.primary.temperature2, g_packet.primary.pressure2, nicla_ori.w());
        Serial.print(buf);
        nicla_dbg_count++;
    }
#endif

    // Orientation quaternion → yaw angle
    float w = nicla_ori.w();
    float x = nicla_ori.x();
    float y = nicla_ori.y();
    float z = nicla_ori.z();
    float num = 2.0f * (w * z + x * y);
    float den = 1.0f - 2.0f * (y * y + z * z);
    yaw_deg = atan2(num, den) * 180.0f / PI;

    // Initialise relative altitude reference on first valid Nicla barometer reading
    if (!alt_init && g_packet.primary.pressure2 > 0.0f) {
        alt      = Altitude(g_packet.primary.temperature2, 0, g_packet.primary.pressure2);
        alt_init = true;
        Serial.println("NAV: altitude reference set");
    }

    if (alt_init) {
        nicla_altitude = alt.get_alt(g_packet.primary.temperature2, g_packet.primary.pressure2);
    }
}

void navigationUpdate()
{
    if (!alt_init) return;   // altitude reference not yet set — nothing to do

    // Ascent detection
    if (g_packet.state == STATE_PAD && nicla_altitude > 10.0f) {
        g_packet.state = STATE_ASCENT;
        Serial.println("NAV: ascent detected");
    }

    // Apogee / parachute deploy
    if (!crossed_500 && nicla_altitude > 480.0f) {
        crossed_500    = true;
        g_packet.state = STATE_APOGEE;
        Serial.println("NAV: apogee threshold crossed");
    }

    if (crossed_500 && !parachute_deployed && nicla_altitude < 60.0f) {
        parachute_deployed = true;
        g_packet.state     = STATE_DESCENT;
        break_left.write(LEFT_NEUTRAL);
        break_right.write(RIGHT_NEUTRAL);
        Serial.println("NAV: parachute deployed — servos to neutral");
    }

    // Checkpoint navigation — only active during descent with valid GPS
    if (parachute_deployed && !flight_end && g_packet.secondary.gps_fix) {
        double longitude = g_packet.secondary.longitude;
        double latitude  = g_packet.secondary.latitude;

        if (checkpoints_counter < 99 &&
            arr[checkpoints_counter].x != 0 && arr[checkpoints_counter].y != 0) {

            float dist = Target.Calc_dist(longitude, latitude);
            if (dist <= 5.0f) {
                checkpoints_counter++;
                Target = Go_to_checkpoint(arr[checkpoints_counter].x, arr[checkpoints_counter].y);
            }

            int leftServo  = LEFT_NEUTRAL;
            int rightServo = RIGHT_NEUTRAL;

            float target_heading = Target.Calc_desiered_heading(yaw_deg, longitude, latitude);
            target_heading = target_heading * TURNING_RATE;
            target_heading = constrain(target_heading, -100.0f, 100.0f);

            // Convert heading magnitude to brake amount (0 → 90 degrees)
            int brake = map(abs(target_heading), 0, 100, 0, 90);

            if (target_heading > 0) {
                rightServo = RIGHT_NEUTRAL + brake;   
            }

            if (target_heading < 0) {
                leftServo = LEFT_NEUTRAL - brake;   
            }

            break_left.write(leftServo);
            break_right.write(rightServo);

        } else {
            // Checkpoint list exhausted — hold last checkpoint
            if (checkpoints_counter > 0) checkpoints_counter--;
            Target     = Go_to_checkpoint(arr[checkpoints_counter].x, arr[checkpoints_counter].y);
            flight_end = true;
            Serial.println("NAV: all checkpoints reached");
        }
    }

    // Landing
    if (flight_end && nicla_altitude < 6.0f && g_packet.state != STATE_LANDED) {
        g_packet.state = STATE_LANDED;
        Serial.println("NAV: landed — triggering M4 log dump");
        RPC.call("DumpM4Log");
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
    // Discard the first 2 samples — BME688 first reading after bme.begin() is
    // unreliable on STM32H7 until the heater and compensation settle.
    static uint8_t m4_samples = 0;
    if (m4_samples < 2) { m4_samples++; return; }

    // Outlier rejection — I2C bus collisions (shared Wire between M7 Nicla and M4 BME688)
    // occasionally produce impossible values. Drop any reading outside physical bounds.
    if (temperature < -50.0f || temperature > 85.0f) return;   // BME688 operating range
    if (pressure    < 300.0f || pressure    > 1100.0f) return;  // sea level to ~9000 m

    g_packet.primary.temperature   = temperature;
    g_packet.primary.pressure      = pressure;
    g_packet.primary.humidity      = humidity;
    g_packet.primary.altitude_baro = altitude;
    // gasresistance: no PrimaryData field — will move to TertiaryData when Nicla gas sensor is wired
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
    Serial.print("  millis:");
    Serial.print(g_packet.timestamp_ms);
    Serial.print("  state:");
    switch (g_packet.state) {
        case STATE_PAD:     Serial.print("PAD");     break;
        case STATE_ASCENT:  Serial.print("ASCENT");  break;
        case STATE_APOGEE:  Serial.print("APOGEE");  break;
        case STATE_DESCENT: Serial.print("DESCENT"); break;
        case STATE_LANDED:  Serial.print("LANDED");  break;
        default:            Serial.print("BOOT");    break;
    }
    Serial.print("  M4:");
    Serial.print(rtCoreLoop);
    Serial.print("  T:");
    Serial.print(g_packet.primary.temperature, 1);
    Serial.print("C  T2:");
    Serial.print(g_packet.primary.temperature2, 1);
    Serial.print("C  P:");
    Serial.print(g_packet.primary.pressure, 1);
    Serial.print("hPa  Alt_baro:");
    Serial.print(g_packet.primary.altitude_baro, 0);
    Serial.print("m  Alt_nav:");
    Serial.print(nicla_altitude, 0);
    Serial.print("m  yaw:");
    Serial.print(yaw_deg, 1);
    if (g_packet.secondary.gps_fix) {
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
    Serial.print("  SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("  IP:   ");
    Serial.println(WiFi.localIP());
    WiFirssi = WiFi.RSSI();
    Serial.print("  RSSI: ");
    Serial.print(WiFirssi);
    Serial.println(" dBm");
}

// ---------------------------------------------------------------------------
// configTime — Portenta H7 replacement for the ESP32 API.
// Queries NTP over UDP and sets the mbed OS system clock via settimeofday().
// ---------------------------------------------------------------------------
static void configTime(int gmtOffset_sec, int daylightOffset_sec, const char *ntpServer)
{
    static const uint16_t NTP_PORT         = 123;
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

    if (udp.available()) {
        udp.read(packet, sizeof(packet));
        uint32_t hi    = ((uint32_t)packet[40] << 24) | ((uint32_t)packet[41] << 16)
                       | ((uint32_t)packet[42] <<  8) |  (uint32_t)packet[43];
        uint32_t epoch = hi - NTP_EPOCH_OFFSET + gmtOffset_sec + daylightOffset_sec;
        struct timeval tv = { (time_t)epoch, 0 };
        settimeofday(&tv, nullptr);
    }
    udp.stop();
}

#endif  // CORE_CM7
