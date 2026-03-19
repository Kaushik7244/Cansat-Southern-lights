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
 * GPS serial port: defined by GPS_SERIAL in hw_config.h — change it there.
 */

#pragma once
#include "data_types.h"
#include "hw_config.h"   // GPS_SERIAL, GPS_BAUD — change port assignments there

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

/**
 * Print a one-line acquisition status to Serial:
 *   chars received, sentences with fix, bad checksums, satellites in view, HDOP, fix state.
 * Call periodically (e.g. every 5 s) while waiting for a fix.
 */
void gpsDebugStatus();
