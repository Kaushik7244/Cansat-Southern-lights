/**
 * gps.h — SouthernLights CanSat 2026
 *
 * Grove Air530 GPS — NMEA parsing via TinyGPS++.
 * Requires: TinyGPS++ library (install via Arduino Library Manager)
 *
 * Kaushik owns this file. Interface:
 *   gpsInit()      — call once in setup()
 *   gpsPoll(pkt)   — call every loop iteration; fills pkt.secondary when fix valid
 *
 * Time sync: on the first valid fix that includes UTC time, gpsPoll() calls
 * storageSetBootEpoch() automatically. No manual action needed.
 *
 * GPS serial port: Serial2 — change GPS_SERIAL below if wired differently.
 */

#pragma once
#include "data_types.h"

// Serial port the Grove Air530 is connected to.
// Change this if GPS is wired to a different UART.
#define GPS_SERIAL Serial2
#define GPS_BAUD   9600

/**
 * Initialise GPS serial port. Call once in setup().
 */
void gpsInit();

/**
 * Poll the GPS — read available bytes, parse NMEA, update pkt.secondary.
 * Call every loop iteration. Returns true if a new complete fix was parsed.
 * Non-blocking: only reads bytes already in the serial buffer.
 */
bool gpsPoll(SensorPacket& pkt);

/**
 * Returns true if GPS has a valid fix.
 */
bool gpsHasFix();
