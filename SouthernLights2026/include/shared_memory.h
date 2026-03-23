/**
 * shared_memory.h — SouthernLights CanSat 2026
 *
 * Non-blocking inter-core data exchange via a fixed region of shared SRAM.
 *
 * ============================================================
 * WHY THIS EXISTS — THE PROBLEM WITH RPC.call()
 * ============================================================
 * The original design used RPC.call() from M4 to deliver BME688 data to M7.
 * RPC.call() is SYNCHRONOUS AND BLOCKING — M4's loop stalled until M7
 * acknowledged each call.  If M7 crashed, hung, or was delayed (e.g. in a
 * BLE or SD callback), M4 froze with it.  The two cores shared a fate.
 *
 * ============================================================
 * THE FIX — SHARED SRAM
 * ============================================================
 * M4 writes sensor data directly to a known address in SRAM4 and continues
 * immediately.  M7 polls that address at its own pace.  Neither core waits
 * for the other.  If M7 crashes and restarts, it simply reads the latest
 * data again on its next poll.
 *
 * ============================================================
 * HARDWARE
 * ============================================================
 * STM32H745 SRAM4 at 0x38000000 — 64 KB, accessible from both M7 and M4.
 * We occupy the first sizeof(M4SharedData) bytes.  The rest is free.
 *
 * ============================================================
 * CACHE COHERENCY (important on Cortex-M7)
 * ============================================================
 * The Cortex-M7 has a hardware D-cache; the M4 does not.
 * Without explicit cache management, M7 may read stale cached values instead
 * of M4's freshly written data.
 *
 * Solution: M7 setup() configures MPU Region 6 to mark the shared SRAM3 region
 * as non-cacheable.  All M7 reads/writes bypass the D-cache entirely.
 * __DMB() barriers ensure write ordering on both cores.
 * M4 (Cortex-M4) has no D-cache — its accesses always hit physical SRAM.
 *
 * ============================================================
 * WRITE CONSISTENCY — SEQUENCE COUNTER PATTERN
 * ============================================================
 * M4 increments seq_write before touching any data field, and increments
 * seq_done after the last field.  M7 reads are consistent only when
 * seq_write == seq_done (no write was in progress during the read).
 * If they differ, M7 skips the read and retries next cycle.
 *
 *   M4 writer:              M7 reader:
 *   seq_write++             invalidate cache
 *   __DMB()                 s1 = seq_write
 *   write fields...         s2 = seq_done
 *   __DMB()                 if (s1 == s2 && s1 != last_seq) → use data
 *   seq_done++
 *
 * ============================================================
 * STARTUP
 * ============================================================
 * M7 zeroes the struct at the very start of its setup() so the seq counters
 * start at 0 and M7's poll block never mistakes power-on SRAM garbage for
 * a valid first snapshot.  M4 starts writing as soon as RPC.begin() returns
 * (the only natural sync point — OpenAMP channel negotiation requires both
 * cores to call RPC.begin()).  No explicit handshake is needed: both cores
 * run their SPI buses independently, so M4 does not have to wait for M7.
 */

#pragma once
#include <stdint.h>
#include "data_types.h"   // FlightState enum

// ---------------------------------------------------------------------------
// Physical address — Backup SRAM (0x38800000, 4 KB).
//
// SRAM4 (0x38000000) is entirely occupied by OpenAMP/RPC — our struct there
// got silently corrupted by OpenAMP resource table writes.
// SRAM3 (0x30040000) is used by the mbed linker for heap/data.
//
// Backup SRAM is a dedicated 4 KB region accessible from both M7 and M4.
// Neither linker nor OpenAMP uses it.  Requires RCC BKPRAM clock enable
// (done in M7 setup before first access).
// ---------------------------------------------------------------------------
#define SHARED_SRAM_BASE  0x38800000UL

// ---------------------------------------------------------------------------
// Shared data structure
// ---------------------------------------------------------------------------
struct M4SharedData {
    // --- Consistency guard (written by M4) ----------------------------------
    // seq_write is incremented before any field is modified.
    // seq_done  is incremented after all fields are written.
    // A snapshot is valid only when seq_write == seq_done.
    volatile uint32_t seq_write;

    // --- Sensor data (written by M4, read by M7) ----------------------------
    float    temperature;     // °C
    float    pressure;        // hPa
    float    humidity;        // %RH
    float    gasresistance;   // Ω
    float    altitude;        // m (ISA model from BME688 pressure)
    uint8_t  m4_errors;       // cumulative M4 error count (e.g. BME688 failures)
    uint8_t  _pad[3];         // padding so m4_loop is 4-byte aligned
    uint32_t m4_loop;         // M4 loop counter — M7 monitors this as a watchdog

    // --- Consistency guard (written by M4) ----------------------------------
    volatile uint32_t seq_done;

    // --- Flight state (written by M7, read by M4) ---------------------------
    // M4 has no flight state machine — it uses this to stamp LoRa packets with
    // the correct state (ASCENT, DESCENT, etc.) instead of always sending PAD.
    // Single-byte write is atomic on ARM; no sequence guard needed.
    FlightState m7_flight_state;
};

// Typed pointer to the shared region — same physical address on both cores.
#define M4_SHARED  ((volatile M4SharedData *)SHARED_SRAM_BASE)
