/**
 * storage.h — SouthernLights CanSat 2026
 *
 * SD card storage for sensor data and images.
 *
 * Sensor data  → sens_0001.csv, sens_0002.csv …  (new file each power cycle)
 * Image files  → IMG_0001.jpg … IMG_9999.jpg      (sequential, never overwritten)
 * Image index  → img_mfst.csv                     (filename, reason, flight, timestamp)
 *
 * A new sens_nnnn.csv is created on every storageInit(). This keeps pre-launch
 * power cycles, test runs, and actual flights in separate files. The flight
 * number is reported back so it can be logged to Serial for pre-launch checks.
 *
 * Usage pattern in main loop:
 *   storageInit()          once in setup() — check return value before arming
 *   storageFlightNumber()  to read the assigned flight number for status logging
 *   storageDrain()         every ~10 seconds to flush ring buffer to SD
 *   storageWriteImage()    at key flight events (apogee, descent, etc.)
 *
 * storageDrain() and storageWriteImage() must not be called concurrently.
 * Call storageWriteImage() only between drains, when SD is known free.
 *
 * storageInit() will NOT reformat the card on mount failure.
 * A failed init leaves g_sd_ok false — all subsequent calls are safe no-ops.
 * Detect this in STATE_PAD and report it before launch.
 */

#pragma once
#include "data_types.h"

/**
 * Initialise SD card. Mounts filesystem, creates sens_nnnn.csv for this run,
 * writes CSV header row. Scans existing files to avoid number collisions.
 * Returns true on success. On any failure, returns false — does NOT reformat.
 */
bool storageInit();

/**
 * Returns the flight number assigned during storageInit().
 * 0 means init has not been called or failed.
 * Log this to Serial during STATE_PAD so the team can confirm SD is working.
 */
uint16_t storageFlightNumber();

/**
 * Set the Unix epoch time at boot. Call this from any time source that syncs:
 *   NTP:    storageSetBootEpoch(ntp_now - millis()/1000, "NTP")
 *   GPS:    storageSetBootEpoch(gps_epoch - millis()/1000, "GPS")
 *   APC220: storageSetBootEpoch(received_epoch - millis()/1000, "APC")
 *
 * First caller wins — GPS will not overwrite a prior NTP sync, etc.
 * Writes a session info file (sess_nnnn.txt) recording the sync event.
 * After this call, writeRow() will populate the utc_ts column correctly.
 */
void storageSetBootEpoch(uint32_t epoch, const char* source);

/**
 * Returns the stored boot epoch. 0 = not yet synced.
 * Use to guard against lower-priority sources overwriting a good sync.
 */
uint32_t storageGetBootEpoch();

/**
 * Drain the ring buffer to SD, then flush.
 * Writes one CSV row per pending SensorPacket.
 * Calls flush() after all rows — protects against power loss.
 * Increments g_packet.m7_errors on write failure; does not halt.
 * Safe to call even if the ring buffer is empty (no-op).
 */
void storageDrain(SensorPacket& g_packet);

/**
 * Write a JPEG image buffer to a numbered file on SD.
 * Also appends one row to img_manifest.csv recording the filename,
 * the reason for capture, and the sequence + timestamp from the packet.
 *
 * @param buf      Pointer to JPEG data in memory
 * @param len      Length of JPEG data in bytes
 * @param reason   Short string describing why this image was taken
 *                 e.g. "apogee", "descent", "periodic", "landed"
 * @param pkt      Current sensor packet — used for sequence and timestamp in manifest
 * @param g_packet Live packet — m7_errors incremented on failure
 * Returns true on success.
 */
bool storageWriteImage(const uint8_t* buf, size_t len, const char* reason,
                       const SensorPacket& pkt, SensorPacket& g_packet);
