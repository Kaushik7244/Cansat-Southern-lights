"""
correct_flight_data.py — Post-flight barometric altitude correction

Reads the raw APC220 telemetry CSV from the 2026-03-25 drone drop flight
and adds corrected altitude columns using the actual sea-level pressure
derived from GPS ground truth.

Background
----------
The BME680 barometric altitude uses the International Standard Atmosphere
reference pressure P0 = 1013.25 hPa.  On flight day the actual sea-level
pressure (SLP) was only 953.16 hPa — 60 hPa below standard — which caused
the altimeter to over-read by ~512 m.

The corrected P0 was determined by averaging the implied SLP from early,
stable GPS+baro pairs on the pad/parachute (t = 155–215 s, sats >= 7,
speed < 0.5 m/s):

    P0 = pressure / (1 - gps_alt_msl / 44330) ^ 5.255

This gave P0 = 953.16 +/- 0.05 hPa  (n = 26 readings).

The GPS ground level (50.2 m MSL) is the average of those same readings
and is used as the above-ground-level (AGL) reference.

Usage
-----
    python correct_flight_data.py [input.csv] [output.csv]

Defaults to the 135651 flight log in the same folder.
"""

import csv
import sys

# ---------------------------------------------------------------------------
# Calibration constants from post-flight GPS-baro analysis
# ---------------------------------------------------------------------------
P0_DEFAULT   = 1013.25   # hPa — standard atmosphere (used by firmware)
P0_CORRECTED = 953.16    # hPa — actual SLP on 2026-03-25 at Andoya
GPS_GROUND   = 50.2      # m MSL — average GPS altitude on the ground


def baro_altitude(pressure_hpa, p0_hpa):
    """
    Barometric altitude using the hypsometric formula (simplified ISA).

        alt = 44330 * (1 - (P / P0) ^ 0.1903)

    Parameters
    ----------
    pressure_hpa : float   — measured atmospheric pressure in hPa
    p0_hpa       : float   — sea-level pressure in hPa

    Returns
    -------
    float — altitude in metres above mean sea level
    """
    if pressure_hpa <= 0:
        return 0.0
    return 44330.0 * (1.0 - (pressure_hpa / p0_hpa) ** 0.1903)


def solve_p0(pressure_hpa, known_alt_m):
    """
    Back-calculate the sea-level pressure P0 from a known altitude and
    measured pressure.

        P0 = P / (1 - alt / 44330) ^ 5.255

    This is the inverse of baro_altitude().
    """
    return pressure_hpa / (1.0 - known_alt_m / 44330.0) ** 5.255


def main():
    in_path  = sys.argv[1] if len(sys.argv) > 1 else "tools/apc220_log_20260325_135651.csv"
    out_path = sys.argv[2] if len(sys.argv) > 2 else in_path.replace(".csv", "_corrected.csv")

    with open(in_path, newline="") as fin:
        reader = csv.DictReader(fin)
        fields_in = reader.fieldnames
        rows = list(reader)

    # Columns we add
    extra_fields = [
        "alt_baro_corrected_m",     # baro alt with corrected P0
        "alt_above_ground_baro_m",  # corrected baro minus GPS ground level
        "alt_above_ground_gps_m",   # GPS alt minus GPS ground level
        "gps_baro_diff_m",          # GPS alt - corrected baro (sensor agreement)
        "elapsed_s",                # seconds since first data row
        "p0_used_hPa",             # corrected P0 value used
        "gps_ground_ref_m",        # ground reference altitude used
        "gps_valid",               # 1 if GPS looks trustworthy, 0 otherwise
    ]
    fields_out = fields_in + extra_fields

    t0 = float(rows[0]["ts_ms"])

    with open(out_path, "w", newline="") as fout:
        writer = csv.DictWriter(fout, fieldnames=fields_out)
        writer.writeheader()

        for r in rows:
            press = float(r.get("press1_hPa", "0") or "0")
            gps   = float(r.get("alt_gps_m",  "0") or "0")
            sats  = int(r.get("gps_sats",     "0") or "0")
            ts    = float(r["ts_ms"])

            # Corrected barometric altitude
            alt_cor  = baro_altitude(press, P0_CORRECTED) if press > 0 else 0.0

            # Above-ground-level from baro
            agl_baro = alt_cor - GPS_GROUND if press > 0 else 0.0

            # Above-ground-level from GPS (blank if no GPS fix)
            agl_gps = f"{gps - GPS_GROUND:.2f}" if gps != 0 else ""

            # GPS vs corrected baro difference (blank if either is missing)
            diff = f"{gps - alt_cor:.2f}" if (gps != 0 and press > 0) else ""

            # Elapsed time in seconds
            elapsed = (ts - t0) / 1000.0

            # GPS validity flag: positive altitude and enough satellites
            gps_ok = 1 if (gps > 0 and sats >= 5) else 0

            # Write augmented row
            r["alt_baro_corrected_m"]    = f"{alt_cor:.2f}"
            r["alt_above_ground_baro_m"] = f"{agl_baro:.2f}"
            r["alt_above_ground_gps_m"]  = agl_gps
            r["gps_baro_diff_m"]         = diff
            r["elapsed_s"]               = f"{elapsed:.2f}"
            r["p0_used_hPa"]            = f"{P0_CORRECTED}"
            r["gps_ground_ref_m"]       = f"{GPS_GROUND}"
            r["gps_valid"]              = str(gps_ok)

            writer.writerow(r)

    print(f"Wrote {len(rows)} rows to {out_path}")
    print(f"P0 corrected:  {P0_CORRECTED} hPa  (default was {P0_DEFAULT})")
    print(f"GPS ground ref: {GPS_GROUND} m MSL")
    print(f"Baro offset:   ~{baro_altitude(947, P0_DEFAULT) - baro_altitude(947, P0_CORRECTED):.0f} m over-read removed")


if __name__ == "__main__":
    main()
