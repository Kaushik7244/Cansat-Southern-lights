/**
 * gps.cpp — SouthernLights CanSat 2026
 *
 * Grove Air530 GPS via TinyGPS++.
 * See gps.h for interface documentation.
 *
 * KAUSHIK: this is your file.
 *
 * How TinyGPS++ works:
 *   - Call gps.encode(byte) for each byte read from the GPS serial port.
 *   - After encode() returns true, a new NMEA sentence was parsed.
 *   - Read fix data via gps.location, gps.altitude, gps.speed, etc.
 *   - Each field has .isValid() and .age() — only trust data with age < 2000ms.
 */

#include "gps.h"
#include "storage.h"
#include <TinyGPS++.h>

static TinyGPSPlus gps;
static bool g_has_fix       = false;
static bool g_time_synced   = false;   // true once we've called storageSetBootEpoch

void gpsInit() {
    GPS_SERIAL.begin(GPS_BAUD);
}

bool gpsPoll(SensorPacket& pkt) {
    bool new_sentence = false;

    // Drain the serial buffer — non-blocking
    while (GPS_SERIAL.available()) {
        if (gps.encode(GPS_SERIAL.read())) {
            new_sentence = true;
        }
    }

    if (!new_sentence) return false;

    // Update fix status
    g_has_fix = gps.location.isValid() && gps.location.age() < 2000;

    if (g_has_fix) {
        // Populate Tier 2 secondary data
        pkt.secondary.latitude       = (float)gps.location.lat();
        pkt.secondary.longitude      = (float)gps.location.lng();
        pkt.secondary.altitude_gps   = (float)gps.altitude.meters();
        pkt.secondary.speed_ms       = (float)gps.speed.mps();
        pkt.secondary.heading        = (float)gps.course.deg();
        pkt.secondary.gps_satellites = (uint8_t)gps.satellites.value();
        pkt.secondary.gps_fix        = true;

        // Time sync — runs once on first valid fix with UTC time
        // Converts GPS date+time to Unix epoch and stores it.
        if (!g_time_synced && gps.time.isValid() && gps.date.isValid()) {
            // TODO: convert TinyGPS date/time to Unix epoch
            // Hint: implement a simple makeEpoch(year, month, day, h, m, s) function,
            //       or use the 'time.h' mktime() approach shown below.
            //
            // struct tm t = {0};
            // t.tm_year = gps.date.year() - 1900;
            // t.tm_mon  = gps.date.month() - 1;
            // t.tm_mday = gps.date.day();
            // t.tm_hour = gps.time.hour();
            // t.tm_min  = gps.time.minute();
            // t.tm_sec  = gps.time.second();
            // time_t gps_epoch = mktime(&t);  // local time — adjust for UTC if needed
            // storageSetBootEpoch((uint32_t)gps_epoch - millis()/1000, "GPS");
            // pkt.boot_epoch = storageGetBootEpoch();
            // g_time_synced = true;

            // KAUSHIK: uncomment and complete the block above when ready.
            // The storageSetBootEpoch call only fires once (first caller wins).
        }

    } else {
        pkt.secondary.gps_fix = false;
    }

    return true;
}

bool gpsHasFix() {
    return g_has_fix;
}
