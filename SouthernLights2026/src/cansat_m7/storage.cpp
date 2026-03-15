/**
 * storage.cpp — SouthernLights CanSat 2026
 *
 * SD card access via mbed SDMMCBlockDevice + FATFileSystem.
 * File operations use standard POSIX fopen/fwrite/fclose.
 *
 * Filesystem mounted at "/fs". All file paths use this prefix.
 * SDMMCBlockDevice uses the Vision Shield SDMMC1 peripheral — no CS pin needed.
 *
 * See storage.h for public interface documentation.
 */

#include "storage.h"
#include <SDMMCBlockDevice.h>
#include <FATFileSystem.h>
#include <sys/stat.h>
#include <stdio.h>

// ---------------------------------------------------------------------------
// Filesystem objects — private to this module
// ---------------------------------------------------------------------------

static SDMMCBlockDevice  block_device;
static mbed::FATFileSystem fs("fs");

// ---------------------------------------------------------------------------
// Module state
// ---------------------------------------------------------------------------

static bool     g_sd_ok        = false;
static uint16_t g_flight_num   = 0;    // assigned in storageInit(), 1-based
static uint16_t g_image_count  = 1;    // next image number to write
static char     g_log_path[24] = {0};  // "/fs/sens_nnnn.csv"
static uint32_t g_boot_epoch   = 0;    // Unix time at boot, 0 until synced

// ---------------------------------------------------------------------------
// CSV column header — must match the format string in writeRow() exactly.
// To add a Tier 3 field: add it here and in writeRow(), nowhere else.
// ---------------------------------------------------------------------------
static const char* CSV_HEADER =
    "seq,ts_ms,utc_ts,state,"          // utc_ts = 0 until time synced
    "temp1,press1,hum,alt_baro,temp2,press2,"
    "lat,lon,alt_gps,speed_ms,heading,gps_sats,gps_fix,"
    "mag_x,mag_y,mag_z,"
    "accel_x,accel_y,accel_z,"
    "gyro_x,gyro_y,gyro_z,"
    "gas_res,"
    "m4_err,m7_err\n";

static const char* MANIFEST_PATH = "/fs/img_mfst.csv";

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------

static const char* stateToStr(FlightState s) {
    switch (s) {
        case STATE_BOOT:    return "BOOT";
        case STATE_PAD:     return "PAD";
        case STATE_ASCENT:  return "ASCENT";
        case STATE_APOGEE:  return "APOGEE";
        case STATE_DESCENT: return "DESCENT";
        case STATE_LANDED:  return "LANDED";
        default:            return "UNKNOWN";
    }
}

static bool fileExists(const char* path) {
    struct stat st;
    return stat(path, &st) == 0;
}

// Scan sens_nnnn.csv files to find the next unused flight number.
// Each power cycle gets its own file — no appending across runs.
static uint16_t findNextFlightNumber() {
    char path[24];
    for (uint16_t i = 1; i <= 9999; i++) {
        snprintf(path, sizeof(path), "/fs/sens_%04u.csv", i);
        if (!fileExists(path)) return i;
    }
    return 9999;
}

// Scan IMG_nnnn.jpg to find the next unused image number.
// Images are not grouped by flight — sequential across all runs.
static uint16_t findNextImageNumber() {
    char path[24];
    for (uint16_t i = 1; i <= 9999; i++) {
        snprintf(path, sizeof(path), "/fs/IMG_%04u.jpg", i);
        if (!fileExists(path)) return i;
    }
    return 9999;
}

// Write one SensorPacket as a single CSV row to an open file.
// Column order must match CSV_HEADER exactly.
static void writeRow(FILE* f, const SensorPacket& p) {
    uint32_t utc_ts = (g_boot_epoch > 0) ? g_boot_epoch + p.timestamp_ms / 1000 : 0;
    fprintf(f,
        "%u,%lu,%lu,%s,"                        // header (seq, ts_ms, utc_ts, state)
        "%.2f,%.2f,%.2f,%.1f,%.2f,%.2f,"       // tier 1 — primary
        "%.6f,%.6f,%.1f,%.2f,%.1f,%u,%d,"      // tier 2 — secondary
        "%.2f,%.2f,%.2f,"                       // tier 3 — magnetometer
        "%.3f,%.3f,%.3f,"                       // tier 3 — accelerometer
        "%.2f,%.2f,%.2f,"                       // tier 3 — gyroscope
        "%.1f,"                                 // tier 3 — gas resistance
        "%u,%u\n",                              // system health
        p.sequence, p.timestamp_ms, utc_ts, stateToStr(p.state),
        p.primary.temperature,  p.primary.pressure,
        p.primary.humidity,     p.primary.altitude_baro,
        p.primary.temperature2, p.primary.pressure2,
        p.secondary.latitude,   p.secondary.longitude,
        p.secondary.altitude_gps, p.secondary.speed_ms,
        p.secondary.heading,    p.secondary.gps_satellites,
        (int)p.secondary.gps_fix,
        p.tertiary.mag_x, p.tertiary.mag_y, p.tertiary.mag_z,
        p.tertiary.accel_x, p.tertiary.accel_y, p.tertiary.accel_z,
        p.tertiary.gyro_x,  p.tertiary.gyro_y,  p.tertiary.gyro_z,
        p.tertiary.gas_resistance,
        p.m4_errors, p.m7_errors
    );
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool storageInit() {
    int err = fs.mount(&block_device);
    if (err) {
        // Mount failed — SD card missing, unformatted, or hardware fault.
        // We do NOT reformat here: a reformat after recovery would destroy
        // flight data. Detect this failure in STATE_PAD before launch.
        g_sd_ok = false;
        return false;
    }

    g_flight_num  = findNextFlightNumber();
    g_image_count = findNextImageNumber();
    snprintf(g_log_path, sizeof(g_log_path), "/fs/sens_%04u.csv", g_flight_num);

    // Create sensor log for this run and write header
    FILE* f = fopen(g_log_path, "w");
    if (!f) { g_sd_ok = false; return false; }
    fputs(CSV_HEADER, f);
    fclose(f);

    // Create manifest if this is the first run on this card
    if (!fileExists(MANIFEST_PATH)) {
        FILE* m = fopen(MANIFEST_PATH, "w");
        if (m) {
            fputs("flight,img_num,filename,reason,seq,ts_ms\n", m);
            fclose(m);
        }
    }

    g_sd_ok = true;
    return true;
}

uint16_t storageFlightNumber() {
    return g_flight_num;
}

void storageSetBootEpoch(uint32_t epoch, const char* source) {
    if (g_boot_epoch > 0) return;   // first caller wins — don't overwrite a good sync
    if (epoch < 1700000000UL) return; // sanity check: must be after 2023
    g_boot_epoch = epoch;

    // Write a session info file so flight number, sync source, and epoch
    // are recorded in one place for post-flight reference.
    if (!g_sd_ok) return;
    char path[24];
    snprintf(path, sizeof(path), "/fs/sess_%04u.txt", g_flight_num);
    FILE* f = fopen(path, "w");
    if (f) {
        fprintf(f, "flight:   %u\n", g_flight_num);
        fprintf(f, "epoch:    %lu\n", epoch);
        fprintf(f, "source:   %s\n", source);
        fclose(f);
    }
}

uint32_t storageGetBootEpoch() {
    return g_boot_epoch;
}

void storageDrain(SensorPacket& g_packet) {
    if (!g_sd_ok)              return;
    if (ring_available() == 0) return;

    FILE* f = fopen(g_log_path, "a");
    if (!f) {
        g_packet.m7_errors++;
        return;
    }

    while (ring_available() > 0) {
        writeRow(f, g_ring[g_ring_tail % RING_SIZE]);
        g_ring_tail++;
    }

    // fclose() flushes stdio buffer and commits the FAT entry.
    // Primary protection against power-loss data loss.
    fclose(f);
}

bool storageWriteImage(const uint8_t* buf, size_t len, const char* reason,
                       const SensorPacket& pkt, SensorPacket& g_packet) {
    if (!g_sd_ok) return false;

    char path[24];
    snprintf(path, sizeof(path), "/fs/IMG_%04u.jpg", g_image_count);

    FILE* img = fopen(path, "wb");
    if (!img) {
        g_packet.m7_errors++;
        return false;
    }

    size_t written = fwrite(buf, 1, len, img);
    fclose(img);   // fclose commits the FAT entry

    if (written != len) {
        g_packet.m7_errors++;
        return false;
    }

    // Manifest row: flight number included so images can be correlated to sens_nnnn.csv
    FILE* m = fopen(MANIFEST_PATH, "a");
    if (m) {
        fprintf(m, "%u,%u,IMG_%04u.jpg,%s,%u,%lu\n",
                g_flight_num, g_image_count, g_image_count,
                reason, pkt.sequence, pkt.timestamp_ms);
        fclose(m);
    }

    g_image_count++;
    return true;
}
