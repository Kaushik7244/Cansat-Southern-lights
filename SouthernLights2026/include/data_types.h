/**
 * data_types.h — SouthernLights CanSat 2026
 *
 * Single source of truth for all sensor and state data structures.
 * Lives in include/ — shared automatically across all PlatformIO environments.
 * Do NOT copy this file into individual sketch folders.
 *
 * ============================================================
 * ADDING A NEW SENSOR — read this first:
 * ============================================================
 *   Primary   (transmitted every cycle, mission-critical):
 *             → add field to PrimaryData
 *             → update UpdateSensorPackage() RPC call in cansat_m4.ino
 *             → update toBinary() in telemetry.cpp
 *             → update toCSV()   in storage.cpp
 *
 *   Secondary (GPS / system state, transmitted when available):
 *             → add field to SecondaryData
 *             → update toBinary() in telemetry.cpp
 *             → update toCSV()   in storage.cpp
 *
 *   Tertiary  (local storage and post-flight analysis only, never transmitted):
 *             → add field to TertiaryData
 *             → update toCSV() in storage.cpp
 *             → that's it — no radio changes needed
 * ============================================================
 */

#pragma once
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

// ---------------------------------------------------------------------------
// FLIGHT STATE
// Owned and driven by M7 state machine.
// M4 receives current state via RPC so it can adapt behaviour (e.g. sample rate).
// ---------------------------------------------------------------------------
enum FlightState : uint8_t {
    STATE_BOOT    = 0,   // System initialising — not ready
    STATE_PAD     = 1,   // On ground, pre-launch — waiting
    STATE_ASCENT  = 2,   // Climbing — primary mission fully active
    STATE_APOGEE  = 3,   // Peak altitude — brief transition, triggers image capture
    STATE_DESCENT = 4,   // Falling — paraglider deployed
    STATE_LANDED  = 5    // On ground, post-flight — data dump mode
};

// ---------------------------------------------------------------------------
// TIER 1 — PRIMARY DATA
// Always transmitted. Always stored to SD. Mission-critical.
// Source: M4 core — BME688 (main sensor) + BMP390 via Nicla Sense (independent)
//
// Having two independent pressure sensors cross-checking each other is the
// primary mission requirement. These fields are non-negotiable in every packet.
// ---------------------------------------------------------------------------
struct PrimaryData {
    float temperature;    // °C   — BME688
    float pressure;       // hPa  — BME688
    float humidity;       // %RH  — BME688
    float altitude_baro;  // m    — derived from BME688 pressure (ISA model)

    float temperature2;   // °C   — BMP390 via Nicla Sense (independent sensor)
    float pressure2;      // hPa  — BMP390 via Nicla Sense (independent sensor)
};

// ---------------------------------------------------------------------------
// TIER 2 — SECONDARY DATA
// Transmitted when available. Always stored to SD.
// Source: M7 core — Grove Air530 GPS
//
// GPS fix may not be available at all times. Check gps_fix before using
// position fields. heading and speed_ms are only valid when gps_fix is true
// and gps_satellites >= 4.
// ---------------------------------------------------------------------------
struct SecondaryData {
    float   latitude;       // degrees — positive = North
    float   longitude;      // degrees — positive = East
    float   altitude_gps;   // m above sea level
    float   speed_ms;       // m/s ground speed
    float   heading;        // degrees 0–360, 0 = North
    uint8_t gps_satellites; // satellites in fix (>= 4 recommended for navigation)
    bool    gps_fix;        // true = position is valid
};

// ---------------------------------------------------------------------------
// TIER 3 — TERTIARY DATA
// Stored locally to SD only. Never transmitted over the air.
// Available for post-flight analysis or on-request after landing.
// Source: M7 core — Nicla Sense (BLE or I2C)
//
// This is the "fun" data. Adding new exotic sensors? Put them here.
// No radio protocol changes required — just add the field and update toCSV().
// ---------------------------------------------------------------------------
struct TertiaryData {
    // BMM150 magnetometer — useful for heading reference and post-flight analysis
    float mag_x;          // µT
    float mag_y;          // µT
    float mag_z;          // µT

    // BHI260AP motion — flight dynamics, useful for apogee detection tuning
    float accel_x;        // m/s²
    float accel_y;        // m/s²
    float accel_z;        // m/s²
    float gyro_x;         // deg/s
    float gyro_y;         // deg/s
    float gyro_z;         // deg/s

    // BME688 gas sensor (Nicla Sense) — air quality, not mission-critical
    float gas_resistance; // kΩ
};

// ---------------------------------------------------------------------------
// CANONICAL SENSOR PACKET
// The single authoritative data record for one measurement cycle.
//
// One live instance (g_packet) lives on M7 and is the source for both
// SD writes and radio transmission.
// M4 populates the primary fields via RPC.
// M7 fills secondary (GPS) and tertiary (Nicla Sense) directly.
// ---------------------------------------------------------------------------
struct SensorPacket {
    // Header
    uint16_t    sequence;      // Packet counter — wraps at 65535, used to detect gaps
    uint32_t    timestamp_ms;  // millis() stamped on M7 at the moment of transmission
    uint32_t    boot_epoch;    // Unix time at boot (seconds). 0 = not yet synced.
                               // Set once by NTP, GPS, or APC220 ground command.
                               // utc = boot_epoch + timestamp_ms/1000
    FlightState state;         // Current flight state
    uint8_t     m4_errors;     // M4 cumulative error count (incremented on sensor failure)
    uint8_t     m7_errors;     // M7 cumulative error count
    uint8_t     flags;         // status bits — see FLAG_* constants

    PrimaryData   primary;     // Tier 1 — always transmitted
    SecondaryData secondary;   // Tier 2 — transmitted when available
    TertiaryData  tertiary;    // Tier 3 — SD only, never transmitted
};

// ---------------------------------------------------------------------------
// TRANSMISSION PACKET  (binary, over-the-air)
// Flattened, packed struct containing only Tier 1 + Tier 2 data.
// Built from g_packet by toBinary() in telemetry.cpp before sending.
// __attribute__((packed)) removes padding so sizeof() matches wire size exactly.
// ---------------------------------------------------------------------------
// Status flags for TxPacket.flags
static const uint8_t FLAG_NICLA_BLE_OK = 0x01;  // bit 0: Nicla BLE connected and delivering data

struct __attribute__((packed)) TxPacket {
    uint16_t    sequence;
    uint32_t    timestamp_ms;
    uint32_t    boot_epoch;   // 0 until time synced — ground station uses this for UTC display
    FlightState state;
    uint8_t     m4_errors;
    uint8_t     m7_errors;
    uint8_t     flags;        // status bits — see FLAG_* constants above
    PrimaryData   primary;    // 6 floats = 24 bytes
    SecondaryData secondary;  // 5 floats + uint8 + bool = 22 bytes
};
// Total TxPacket payload: ~60 bytes. Well within APC220 and LoRa limits.

// ---------------------------------------------------------------------------
// WIRE FRAME FORMAT
//
//   [SYNC_1][SYNC_2][LENGTH:1][...TxPacket...][CRC_HI:1][CRC_LO:1]
//
// SYNC bytes allow the receiver to find the start of a packet in a
// stream of bytes, even after noise or a partial packet.
// CRC16 lets the receiver discard corrupted packets silently.
// ---------------------------------------------------------------------------
static const uint8_t FRAME_SYNC_1      = 0xAA;
static const uint8_t FRAME_SYNC_2      = 0x55;
static const uint8_t FRAME_PAYLOAD_LEN = sizeof(TxPacket);

// ---------------------------------------------------------------------------
// GPS REPLY — M4 pulls this from M7 via RPC.call("getGPSData") each cycle.
// All floats to avoid alignment issues across cores. fix: 1.0 = valid, 0.0 = none.
// Only M4→M7 direction used, so no bidirectional RPC deadlock possible.
// ---------------------------------------------------------------------------
struct GPSReply {
    float lat;
    float lon;
    float alt;
    float sats;
    float fix;
#ifdef MSGPACK_DEFINE
    MSGPACK_DEFINE(lat, lon, alt, sats, fix);
#endif
};

// ---------------------------------------------------------------------------
// RING BUFFER  (M7 only)
// Decouples async RPC data arrival (M4 push, 1 Hz) from SD batch writes.
// RPC handler writes to g_ring[g_ring_head % RING_SIZE] and bumps g_ring_head.
// Main loop drains from g_ring_tail to SD, then bumps g_ring_tail.
// One writer (RPC thread), one reader (main loop) — no mutex needed.
// 30 slots = 30 seconds of headroom. Uses ~4.5 KB of M7 RAM.
// ---------------------------------------------------------------------------
#ifdef CORE_CM7

#define RING_SIZE 30

extern SensorPacket     g_ring[RING_SIZE];
extern volatile uint8_t g_ring_head;   // written by RPC callback
extern uint8_t          g_ring_tail;   // read by main loop only

// Convenience: how many unread slots are waiting?
inline uint8_t ring_available() {
    return (g_ring_head - g_ring_tail);  // unsigned wraps correctly
}

#endif  // CORE_CM7
