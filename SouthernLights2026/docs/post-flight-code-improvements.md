# Post-Flight Code Improvements

Issues and improvements identified from the 2026-03-25 drone drop flight at Andoya.

## 1. Auto-calibrate sea-level pressure (P0) from GPS

**Problem:** The barometric altimeter uses the standard P0 = 1013.25 hPa. On flight day the actual SLP was 953.16 hPa (60.1 hPa below standard), causing a +512m altitude over-read. The reported `alt_baro` of ~570-600m was actually ~50-80m MSL.

**Fix:** Once GPS has a valid fix and a known ground altitude, compute and set P0:
```
P0 = pressure / (1 - gps_alt / 44330)^5.255
```
Or simpler: record the pressure on the pad with a known GPS altitude and compute the offset.

**Where:** `src/cansat_m7/` sensor init or a new calibration step after GPS lock.

## 2. State machine did not trigger on drone drop

**Problem:** The flight state stayed `PAD` throughout the entire flight. The launch detection logic expects a rocket-like acceleration profile and did not recognise the drone drop (gentle release, no high-g event).

**Fix:** Add an alternative launch trigger for drone-drop mode, e.g.:
- Barometric rate-of-climb exceeds a threshold (e.g. > 0.5 m/s descent sustained for 2s)
- Or a manual arm/trigger command over radio

## 3. BME sensor warm-up drift

**Problem:** The BME680 pressure readings drifted ~0.59 hPa/min after power-on due to the gas heater thermally coupling into the pressure sensor. This made the baro altitude appear to keep descending even after landing (-9m MSL on ground).

**Fix options:**
- Discard or flag the first ~60-90s of BME pressure data after boot
- Disable the gas heater during altitude-critical phases
- Apply a warm-up compensation curve (characterise drift on the bench)

## 4. GPS altitude goes negative after landing

**Problem:** After t=445s GPS altitude dropped to -122m with speeds of 10 m/s. Likely multipath or loss of fix on the ground.

**Fix:** Sanity-check GPS altitude against baro altitude. If the two diverge by more than e.g. 50m, flag GPS altitude as unreliable and fall back to baro-only.

## 5. GPS time sync not implemented

**Problem:** `utc_ts` is always 0 in the telemetry. The `gps.cpp` time-sync code is commented out (marked as TODO for Kaushik).

**Fix:** Uncomment and complete the `storageSetBootEpoch` block in `gps.cpp:58-77`.

## 6. First ~48s of flight data missing

**Problem:** The BME sensor took ~48s to produce its first valid reading. By that time the cansat had already descended ~68m from the 100m drop altitude.

**Fix:** Reduce sensor init time, or power on the cansat earlier (while still on the drone/pad). Alternatively, log raw GPS altitude from first fix as the initial altitude reference.
