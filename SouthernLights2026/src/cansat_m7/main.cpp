/**
 * cansat_m7/main.cpp — SouthernLights CanSat 2026
 *
 * M7 main core: WiFi/NTP, GPS, Nicla Sense, navigation, telemetry, SD storage.
 * M4 writes BME688 sensor data to shared SRAM (M4_SHARED) → M7 polls → g_packet.primary.
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
#include "hw_config.h"
#include "gps.h"
#include "Altitude.h"
#include "Go_to_checkpoint.h"
#include "shared_memory.h"
#include "cam_capture.h"

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

static void ledSet(bool r, bool g, bool b) {
    digitalWrite(LEDR, r ? ON : OFF);
    digitalWrite(LEDG, g ? ON : OFF);
    digitalWrite(LEDB, b ? ON : OFF);
}

// ---------------------------------------------------------------------------
// WiFi
// ---------------------------------------------------------------------------
const bool WIFI_ENABLED = false;
char ssid[]     = SECRET_SSID;
char pass[]     = SECRET_PASS;
long WiFirssi   = -1;
int  WiFistatus = WL_IDLE_STATUS;

// ---------------------------------------------------------------------------
// Navigation — hardware
// ---------------------------------------------------------------------------
#define SERVO_PIN_LEFT  PH_15
#define SERVO_PIN_RIGHT PG_7    // moved from PK_1 (now used by camera TIM1_CH1)
#define LEFT_NEUTRAL 90 
#define RIGHT_NEUTRAL 90
#define altitude_sim false
static const float TURNING_RATE  = 0.8f;

Servo break_left;
Servo break_right;

// Nicla Sense ME sensors via BHY2Host
Sensor          nicla_barometer(SENSOR_ID_BARO);
Sensor          nicla_temp(SENSOR_ID_TEMP);
SensorQuaternion nicla_ori(SENSOR_ID_GEORV);  // Geo-magnetic rotation vector

// Camera — see cam_capture.h
static bool g_cam_ok = false;

// ---------------------------------------------------------------------------
// Navigation — state
// ---------------------------------------------------------------------------

// Checkpoints — lat/lon pairs. {0,0} entries are ignored.
// UPDATE THESE BEFORE EACH FLIGHT.
struct Checkpoint { float x; float y; };
static const Checkpoint arr[100] = {
    {59.896537f, 10.522882f},
    {59.896694f, 10.523529f},
    {59.896924f, 10.523528f},
    // remaining entries zero-initialised — treated as end-of-list
};

Go_to_checkpoint Target(arr[0].y, arr[0].x);
int              checkpoints_counter = 0;

// Landing loop — 3 waypoints 10m apart to circle last checkpoint
bool             landing_phase       = false;   // true when flying landing loop
struct Checkpoint landing_waypoints[3] = {};
int              landing_wp_counter  = 0;      // loops 0→1→2→0→1→...

Altitude alt;
float    nicla_altitude = 0.0f;
float    yaw_deg        = 0.0f;
bool     nicla_active   = false;
bool     alt_init       = false;
bool     flight_end     = false;

// M4 loop counter — read from shared SRAM and printed in printPacketToSerial()
long   rtCoreLoop        = 0;

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
static bool          g_iic_uart_ok        = false;  // false → skip GPS poll / telemetry to avoid I2C timeout stalls
static uint32_t g_last_tx_ms         = 0;
static uint32_t g_last_drain_ms      = 0;
static uint32_t g_last_nicla_ms      = 0;
static uint32_t g_last_gps_ms        = 0;
static uint32_t g_last_gps_status_ms = 0;
static const uint32_t TX_INTERVAL_MS         = 1000;   // transmit every 1 s
static const uint32_t DRAIN_INTERVAL_MS      = 10000;  // flush SD every 10 s
static const uint32_t NICLA_INTERVAL_MS      = 1000;   // poll Nicla at 1 Hz (each update blocks ~700 ms)
static const uint32_t GPS_INTERVAL_MS        = 100;    // poll GPS at 10 Hz (9600 baud ≈ 5 NMEA sentences/s)
static const uint32_t GPS_STATUS_INTERVAL_MS = 5000;   // GPS acquisition status every 5 s

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
void navigationSetup();
void niclaUpdate();
void navigationUpdate();
void printPacketToSerial();
// setH4CoreLoop, UpdateSensorPackage, M4Error, getGPSDataForM4 removed —
// M4 data now arrives via shared SRAM (see shared_memory.h), not RPC callbacks.
void printWifiStatus();
static void configTime(int gmtOffset_sec, int daylightOffset_sec, const char *ntpServer);


// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup()
{
    // -----------------------------------------------------------------------
    // Enable Backup SRAM clock — required before any access to 0x38800000.
    // RCC is shared between cores; M4 inherits the clock enable from M7.
    // -----------------------------------------------------------------------
    RCC->AHB4ENR |= RCC_AHB4ENR_BKPRAMEN;
    __DSB();

    // -----------------------------------------------------------------------
    // MPU Region 7: mark Backup SRAM (0x38800000, 4 KB) as non-cacheable.
    // MUST be region 7 (highest ARMv7-M priority) to override mbed's regions.
    // -----------------------------------------------------------------------
    {
        MPU->RNR  = 7;
        MPU->RBAR = 0x38800000UL;           // Backup SRAM base
        // Normal memory, Outer+Inner Non-cacheable (TEX=001 C=0 B=0),
        // Shareable, Full RW access, Execute-Never.
        MPU->RASR = (1UL << 28)    // XN  — no execute
                  | (3UL << 24)    // AP  — full access
                  | (1UL << 19)    // TEX = 001
                  | (1UL << 18)    // S   — shareable
                  | (11UL << 1)    // SIZE = 11 → 2^12 = 4 KB
                  | (1UL << 0);    // ENABLE
        MPU->CTRL = MPU_CTRL_ENABLE_Msk | MPU_CTRL_PRIVDEFENA_Msk;
        __DSB();
        __ISB();
    }

    // Zero the shared SRAM struct.  SRAM4 holds unknown garbage at power-on.
    // Zeroing guarantees seq_write == seq_done == 0 so M7's poll block waits
    // for M4's first real write before accepting any data.
    // SRAM4 is now non-cacheable (MPU region 7), so memset writes go directly
    // to physical SRAM — no cache flush needed.
    memset((void *)SHARED_SRAM_BASE, 0, sizeof(M4SharedData));
    __DMB();

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    ledSet(true, false, false);  // RED: RPC init

    Serial.begin(115200);
    { unsigned long _t = millis(); while (!Serial && millis() - _t < 2000) ; }
    Serial.println("USB Serial up at 115200");

    // Backup SRAM self-test: write a known pattern, read it back, verify.
    {
        volatile uint32_t *test_addr = (volatile uint32_t *)SHARED_SRAM_BASE;
        *test_addr = 0xDEADBEEF;
        __DMB();
        uint32_t readback = *test_addr;
        *test_addr = 0;  // restore to zero for seq_write
        __DMB();
        Serial.print("BKPSRAM self-test @0x");
        Serial.print((uint32_t)SHARED_SRAM_BASE, HEX);
        Serial.print(": ");
        if (readback == 0xDEADBEEF) {
            Serial.println("OK");
        } else {
            Serial.print("FAIL — read 0x");
            Serial.println(readback, HEX);
        }
    }

    // RPC must start before WiFi so M4 can begin sending sensor data
    Serial.print("RPC init...");
    RPC.begin();
    // UpdateSensorPackage, setH4CoreLoop, M4Error bindings removed —
    // M4 now writes directly to shared SRAM (no RPC needed for M4→M7 data).
    // RPC.begin() is still needed: M7 calls RPC.call("DumpM4Log") on landing,
    // and M7 drains M4's RPC.print() debug output via RPC.available().
    Serial.println("OK");

    ledSet(true, true, false);   // YELLOW: peripheral init (SD, radios, GPS)

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

    // DFRobot IIC to Dual UART — must init before GPS and telemetry
    Serial.print("IIC-UART init... ");
    g_iic_uart_ok = iicUartInit();
    if (!g_iic_uart_ok) {
        Serial.println("WARNING — one or both IIC-UART channels failed");
        g_packet.m7_errors++;
    }

    // Radios — APC220 on IIC-UART ch1
    Serial.print("Telemetry init... ");
    if (telemetryInit(g_packet)) {
        Serial.println("OK (APC220)");
    } else {
        Serial.println("WARNING — no radio available");
    }

    // GPS on IIC-UART ch2
    gpsInit();

    // -----------------------------------------------------------------------
    // Wire timeout — MUST be set AFTER all DFRobot_IICSerial::begin() calls.
    // Each begin() calls Wire.begin() internally, which resets the timeout.
    // iicUartInit(), telemetryInit(), and gpsInit() all trigger Wire.begin().
    // Setting it here guarantees the timeout is active for the entire loop.
    // -----------------------------------------------------------------------
    Wire.setTimeout(200);  // 200 ms — prevents permanent I2C hangs
    Serial.println("Wire timeout set to 200ms");
#endif  // ISOLATION_TEST

    // Wait for Nicla to boot before scanning I2C — BHI260AP needs time after power-on.
    delay(4000);

    // I2C bus scan — detect Nicla BHI260AP (0x28/0x29) and any other devices.
    // Scans Wire (I2C3, shared with WK2132) and Wire1 (ESLOV/I2C1).
    {
        Serial.println("I2C scan — Wire (I2C3):");
        Wire.begin();
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire.beginTransmission(addr);
            if (Wire.endTransmission() == 0) {
                Serial.print("  0x");
                if (addr < 0x10) Serial.print('0');
                Serial.print(addr, HEX);
                if (addr == 0x10) Serial.print(" (WK2132)");
                if (addr == 0x28 || addr == 0x29) Serial.print(" (BHI260AP / Nicla)");
                if (addr == 0x77 || addr == 0x76) Serial.print(" (BME688)");
                Serial.println();
            }
        }
        Wire.setTimeout(200);  // restore after scan

        Serial.println("I2C scan — Wire1 (ESLOV/I2C1):");
        Wire1.begin();
        for (uint8_t addr = 1; addr < 127; addr++) {
            Wire1.beginTransmission(addr);
            if (Wire1.endTransmission() == 0) {
                Serial.print("  0x");
                if (addr < 0x10) Serial.print('0');
                Serial.print(addr, HEX);
                if (addr == 0x28 || addr == 0x29) Serial.print(" (BHI260AP / Nicla)");
                Serial.println();
            }
        }
    }

    // Nicla Sense + servos
    navigationSetup();

    g_packet.state = STATE_PAD;

#ifdef ISOLATION_TEST
    Serial.println("ISOLATION_TEST: M4 kept idle");
#endif

    ledSet(false, true, false);  // GREEN: boot complete — all systems go



    // Camera — Vision Shield HM0360 (ext clock on PK_1 via TIM1_CH1)
    Serial.print("Camera init... ");
    g_cam_ok = camInit();
    Serial.println(g_cam_ok ? "OK (HM0360 QVGA grayscale)" : "FAIL");

#if defined(SLDEBUG) && defined(APC_ENABLED)
    // RF link verification — sends plain ASCII so PuTTY can confirm data arrives.
    // If "APC220 TEST" appears on COM14, the radio link is good.
    // Remove before flight build (SLDEBUG must be off).
    APC_SERIAL.println("APC220 TEST");
    Serial.println("APC220 TEST sent on APC_SERIAL");
#endif

    Serial.println("Setup complete — entering loop");
    
}

// ---------------------------------------------------------------------------
// Main loop — non-blocking
// ---------------------------------------------------------------------------

void loop()
{
    uint32_t now_ms = millis();

    // First-loop camera burst — capture several images to SD for verification.
    // Runs once, before any other loop work, so the SD bus is uncontested.
    {
        static bool s_done = false;
        if (!s_done && g_cam_ok) {
            s_done = true;
            static const int NUM_CAPTURES = 5;
            Serial.print("Capturing ");
            Serial.print(NUM_CAPTURES);
            Serial.println(" images to SD...");

            for (int i = 0; i < NUM_CAPTURES; i++) {
                char path[24];
                snprintf(path, sizeof(path), "/fs/CAM_%04d.bmp", i + 1);
                bool ok = camCaptureToSD(path);
                Serial.print("  ");
                Serial.print(path);
                Serial.println(ok ? " OK" : " FAILED");
                if (!ok) g_packet.m7_errors++;
            }
            Serial.println("Camera burst done.");
        }
    }

#ifndef ISOLATION_TEST
    // Forward M4 RPC.print() output to USB Serial
    // Limit to 64 bytes per loop to avoid blocking if M4 floods the channel.
    { int rpc_n = 0; while (RPC.available() && rpc_n < 64) { Serial.write(RPC.read()); rpc_n++; } }

    // APC220 time sync command from ground station.
    // Ground operator sends: TIME:<unix_epoch>   e.g. TIME:1742123456
    // python -c "import time,serial; s=serial.Serial('COM1',9600); s.write(b'TIME:'+str(int(time.time())).encode()+b'\n'); s.close()"
#ifdef APC_ENABLED
    if (APC_SERIAL.available()) {
        APC_SERIAL.setTimeout(200);  // cap at 200 ms (default 1000 ms blocks the loop)
        String cmd = APC_SERIAL.readStringUntil('\n');
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
#endif  // APC_ENABLED
#endif  // ISOLATION_TEST

    // -----------------------------------------------------------------------
    // Read M4 sensor data from shared SRAM — non-blocking.
    //
    // M4 writes here continuously, regardless of M7 state.
    // M7 polls on every loop iteration and uses the data when:
    //   (a) seq_write == seq_done  → snapshot is complete (no write in progress)
    //   (b) seq_write != last seen → data is newer than what we already have
    //
    // SRAM4 is non-cacheable (MPU region 7), so reads always hit physical SRAM.
    // -----------------------------------------------------------------------
    {
        static uint32_t s_last_m4_seq = 0;
        static uint8_t  s_m4_samples  = 0;   // discard first 2 BME readings (warm-up)

        __DMB();  // ordering barrier before reading shared SRAM

        uint32_t s1 = M4_SHARED->seq_write;
        uint32_t s2 = M4_SHARED->seq_done;



        if (s1 == s2 && s1 != s_last_m4_seq) {
            // New, consistent snapshot is ready.
            float t = M4_SHARED->temperature;
            float p = M4_SHARED->pressure;

            if (s_m4_samples < 2) {
                // Discard first 2 samples — BME688 first readings after bme.begin()
                // are unreliable until the heater and compensation circuits settle.
                s_m4_samples++;
            } else {
                // Outlier rejection — I2C collisions can produce impossible values.
                if (t >= -50.0f && t <= 85.0f && p >= 300.0f && p <= 1100.0f) {
                    g_packet.primary.temperature   = t;
                    g_packet.primary.pressure      = p;
                    g_packet.primary.humidity      = M4_SHARED->humidity;
                    g_packet.primary.altitude_baro = M4_SHARED->altitude;
                }
                g_packet.m4_errors = M4_SHARED->m4_errors;
            }

            rtCoreLoop    = (long)M4_SHARED->m4_loop;
            s_last_m4_seq = s1;
        }
    }

    if (nicla_active && now_ms - g_last_nicla_ms >= NICLA_INTERVAL_MS) {
        BHY2Host.update();
        niclaUpdate();
        g_last_nicla_ms = now_ms;
    }

#ifndef ISOLATION_TEST
    // GPS — rate-limited to 10 Hz to reduce Wire (I2C3) bus load on shared WK2132.
    // Skipped when IIC-UART failed — each attempt would block ~1 s on I2C timeout.
    if (g_iic_uart_ok && now_ms - g_last_gps_ms >= GPS_INTERVAL_MS) {
        gpsPoll(g_packet);
        g_last_gps_ms = now_ms;
    }

#ifdef SLDEBUG
    // Periodic acquisition status — stops once fix is acquired
    if (g_iic_uart_ok && !gpsHasFix() && now_ms - g_last_gps_status_ms >= GPS_STATUS_INTERVAL_MS) {
        gpsDebugStatus();
        g_last_gps_status_ms = now_ms;
    }
#endif
#endif

    // Push M7 data into Backup SRAM so M4 can include it in LoRa packets.
    // Backup SRAM is non-cacheable (MPU region 7), so writes are visible to M4 immediately.
    M4_SHARED->m7_flight_state    = g_packet.state;
    M4_SHARED->nicla_temperature2 = g_packet.primary.temperature2;
    M4_SHARED->nicla_pressure2    = g_packet.primary.pressure2;
    M4_SHARED->gps_latitude       = g_packet.secondary.latitude;
    M4_SHARED->gps_longitude      = g_packet.secondary.longitude;
    M4_SHARED->gps_altitude       = g_packet.secondary.altitude_gps;
    M4_SHARED->gps_speed          = g_packet.secondary.speed_ms;
    M4_SHARED->gps_heading        = g_packet.secondary.heading;
    M4_SHARED->gps_satellites     = g_packet.secondary.gps_satellites;
    M4_SHARED->gps_fix            = g_packet.secondary.gps_fix;

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
        if (g_iic_uart_ok) {
            telemetrySend(g_packet, g_packet);
        }
#endif
        g_last_tx_ms = now_ms;

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

// Simulate altitude: cycles 0 → 1000 → 0 continuously (30 s full cycle)
float getSimulatedAltitude()
{
    static unsigned long start_time = millis();
    unsigned long elapsed = millis() - start_time;
    unsigned long cycle_period = 30000;  // 30 s total (15 s up, 15 s down)
    unsigned long cycle_pos = elapsed % cycle_period;
    
    if (cycle_pos < 15000) {
        // Ascent phase: 0 → 1000 m over 15 s
        return (cycle_pos / 15000.0f) * 1000.0f;
    } else {
        // Descent phase: 1000 → 0 m over 15 s
        return (1.0f - (cycle_pos - 15000) / 15000.0f) * 1000.0f;
    }
}

void navigationSetup()
{
    // Nicla Sense ME via ESLOV (I2C, address 0x55 on Wire/I2C3).
    // Nicla must be flashed with NICLA_I2C firmware.
    Serial.print("Nicla Sense init (ESLOV I2C)... ");
    if (BHY2Host.begin(false, NICLA_VIA_ESLOV)) {
        Wire.setTimeout(200);  // BHY2Host.begin() calls Wire.begin(), resetting timeout
        nicla_temp.begin(1, 0);
        nicla_barometer.begin(1, 0);
        nicla_ori.begin(10, 0);
        nicla_active = true;
        Serial.println("OK");
    } else {
        nicla_active = false;
        Serial.println("FAILED");
    }
    // Restore Wire timeout — BHY2Host.begin() may have called Wire.begin()
    Wire.setTimeout(200);

    // Servos init at neutral position
    Serial.print("Servos init... ");
    break_left.attach(SERVO_PIN_LEFT);
    break_right.attach(SERVO_PIN_RIGHT);
    break_left.write(LEFT_NEUTRAL);
    break_right.write(RIGHT_NEUTRAL);
    Serial.println("OK");
}
// Called at 5 Hz from the main loop, after BHY2Host.update().
// Reads the latest cached sensor values from the BHY2 host library and
// writes them into g_packet. BHY2Host.update() (I2C) must be called first
// to refresh the cache.
void niclaUpdate()
{
    // Always read latest values — returns 0.0 until first I2C packet arrives.
    float t2 = nicla_temp.value();
    float p2 = nicla_barometer.value() / 100.0f;  // Pa → hPa

    // Outlier rejection — corrupt I2C data can produce impossible values that
    // would falsely trigger the flight state machine.
    bool t2_valid = (t2 >= -50.0f && t2 <= 85.0f);
    if (t2_valid)                        g_packet.primary.temperature2 = t2;
    if (p2 >= 300.0f && p2 <= 1100.0f)  g_packet.primary.pressure2    = p2;

#ifdef SLDEBUG
    static uint8_t nicla_dbg_count = 0;
    if (nicla_dbg_count < 20) {
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
        if (altitude_sim) {
            nicla_altitude = getSimulatedAltitude();
        } else {
            nicla_altitude = alt.get_alt(g_packet.primary.temperature2, g_packet.primary.pressure2);
        }
    }

}


void navigationUpdate()
{
    
    if (!alt_init) return;   // altitude reference not yet set — nothing to do

    // Ascent detection — threshold 30 m to absorb BMP390 warm-up pressure drift
    // (~10 m altitude error in the first ~90 s after power-on).  10 m was enough
    // to trigger a false PAD→ASCENT→DESCENT transition outdoors.
    if (g_packet.state == STATE_PAD && nicla_altitude > 30.0f) {
        g_packet.state = STATE_ASCENT;
        Serial.println("NAV: ascent detected");
    }

    // Descent phase transition
    if (g_packet.state == STATE_ASCENT && nicla_altitude < 100.0f) {
        g_packet.state = STATE_DESCENT;
        Serial.println("NAV: descent phase started");
    }

    // Checkpoint navigation — active during descent with valid GPS
    if (g_packet.state == STATE_DESCENT && !flight_end) {
        if (g_packet.secondary.gps_fix) {
            double longitude = g_packet.secondary.longitude;
            double latitude  = g_packet.secondary.latitude;

            if (checkpoints_counter < 99 &&
                arr[checkpoints_counter].x != 0 && arr[checkpoints_counter].y != 0) {

                float dist = Target.Calc_dist(longitude, latitude);
                float target_heading = Target.Calc_desiered_heading(yaw_deg, longitude, latitude);

                // Output navigation data when altitude sim is enabled
                if (altitude_sim) {
                    char buf[100];
                    snprintf(buf, sizeof(buf), "NAV_SIM: heading=%.1f dist=%.1f alt=%.0f\r\n",
                             target_heading, dist, nicla_altitude);
                    Serial.print(buf);
                }

                if (dist <= 5.0f) {
                    checkpoints_counter++;
                    Target = Go_to_checkpoint(arr[checkpoints_counter].y, arr[checkpoints_counter].x);
                }

                int leftServo  = LEFT_NEUTRAL;
                int rightServo = RIGHT_NEUTRAL;

                target_heading = target_heading * TURNING_RATE;
                target_heading = constrain(target_heading, -100.0f, 100.0f);

                // Convert heading magnitude to brake amount (0 → 90 degrees)
                // Neutral: 90°, Full extension at 0° (right) and 180° (left)
                int brake = map(abs(target_heading), 0, 100, 0, 90);

                if (target_heading > 0) {
                    // Right servo: 90° (no brake) → 0° (full brake)
                    rightServo = constrain(RIGHT_NEUTRAL - brake, 0, 90);
                }

                if (target_heading < 0) {
                    // Left servo: 90° (no brake) → 180° (full brake)
                    leftServo = constrain(LEFT_NEUTRAL + brake, 90, 180);
                }

                break_left.write(leftServo);
                break_right.write(rightServo);

            } 
            
            else {
                // Checkpoint list exhausted — start landing loop (3 waypoints 10m apart)
                if (!landing_phase) {
                    landing_phase = true;
                    landing_wp_counter = 0;
                    // Calculate 3 waypoints around current position at 120° intervals, 10m away
                    LatLng wp0 = calculateDestinationPoint(latitude, longitude, 0.0, 10.0);
                    LatLng wp1 = calculateDestinationPoint(latitude, longitude, 120.0, 10.0);
                    LatLng wp2 = calculateDestinationPoint(latitude, longitude, 240.0, 10.0);
                    landing_waypoints[0] = {(float)wp0.lng, (float)wp0.lat};
                    landing_waypoints[1] = {(float)wp1.lng, (float)wp1.lat};
                    landing_waypoints[2] = {(float)wp2.lng, (float)wp2.lat};
                    Target = Go_to_checkpoint(landing_waypoints[0].y, landing_waypoints[0].x);
                    Serial.println("NAV: entering landing loop (3 waypoints 10m apart)");
                } else {
                    // Flying landing loop — cycle through 3 waypoints and steer toward them
                    float dist = Target.Calc_dist(longitude, latitude);
                    
                    // Calculate heading to current landing waypoint
                    float target_heading = Target.Calc_desiered_heading(yaw_deg, longitude, latitude);
                    
                    int leftServo  = LEFT_NEUTRAL;
                    int rightServo = RIGHT_NEUTRAL;
                    
                    target_heading = target_heading * TURNING_RATE;
                    target_heading = constrain(target_heading, -100.0f, 100.0f);
                    
                    // Convert heading magnitude to brake amount (0 → 90 degrees)
                    int brake = map(abs(target_heading), 0, 100, 0, 90);
                    
                    if (target_heading > 0) {
                        // Right servo: 90° (no brake) → 0° (full brake)
                        rightServo = constrain(RIGHT_NEUTRAL - brake, 0, 90);
                    }
                    
                    if (target_heading < 0) {
                        // Left servo: 90° (no brake) → 180° (full brake)
                        leftServo = constrain(LEFT_NEUTRAL + brake, 90, 180);
                    }
                    
                    break_left.write(leftServo);
                    break_right.write(rightServo);
                    
                    // Advance to next waypoint when close enough
                    if (dist <= 5.0f) {
                        landing_wp_counter = (landing_wp_counter + 1) % 3;
                        Target = Go_to_checkpoint(landing_waypoints[landing_wp_counter].y, landing_waypoints[landing_wp_counter].x);
                    }
                }
            }
        } else if (landing_phase) {
            // GPS lost during landing loop — set servos to neutral and wait for GPS to return
            break_left.write(LEFT_NEUTRAL);
            break_right.write(RIGHT_NEUTRAL);
            Serial.println("NAV: GPS lost during landing loop — servos neutral, descending");
        }
    }

    // Landing — when below 3m in landing loop, shut down all nav systems
    if (landing_phase && nicla_altitude < 3.0f && g_packet.state != STATE_LANDED) {
        flight_end = true;
        g_packet.state = STATE_LANDED;
        // Hold servos at neutral to avoid flutter during final descent
        break_left.write(LEFT_NEUTRAL);
        break_right.write(RIGHT_NEUTRAL);
        Serial.println("NAV: landing threshold reached — nav systems shutdown");
        Serial.println("NAV: landed — triggering M4 log dump");
        RPC.call("DumpM4Log");
    }
}

// ---------------------------------------------------------------------------
// RPC callbacks — called from M4
// ---------------------------------------------------------------------------
// UpdateSensorPackage, setH4CoreLoop, M4Error, getGPSDataForM4 have been
// removed.  M4 sensor data now arrives via shared SRAM polling in loop()
// (see the M4_SHARED poll block above).  No RPC callbacks are needed for the
// M4→M7 data path any more.
//
// RPC.begin() is still called in setup() because:
//   • M7 triggers RPC.call("DumpM4Log") on landing (M7→M4 direction).
//   • M7 drains M4's RPC.print() debug output from setup via RPC.available().

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
    Serial.print(nicla_active ? "C" : "C(ble-lost)");
    Serial.print("  P:");
    Serial.print(g_packet.primary.pressure, 1);
    Serial.print("hPa  P2:");
    Serial.print(g_packet.primary.pressure2, 1);
    Serial.print("hPa  Alt_baro:");
    Serial.print(g_packet.primary.altitude_baro, 0);
    Serial.print("m  Alt_nav:");
    Serial.print(nicla_altitude, 0);
    Serial.print("m  yaw:");
    Serial.print(yaw_deg, 1);
    Serial.print("  ");
    if (landing_phase) {
        Serial.print("LANDING_PHASE  wp:");
        Serial.print(landing_wp_counter);
    } else {
        Serial.print("checkpoint:");
        Serial.print(checkpoints_counter);
    }
    if (g_packet.secondary.gps_fix) {
        Serial.print("  GPS:");
        Serial.print(g_packet.secondary.latitude, 5);
        Serial.print(",");
        Serial.print(g_packet.secondary.longitude, 5);
        Serial.print("  sats:");
        Serial.print(g_packet.secondary.gps_satellites);
    } else {
        Serial.print("  GPS:no-fix sats:");
        Serial.print(g_packet.secondary.gps_satellites);
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
