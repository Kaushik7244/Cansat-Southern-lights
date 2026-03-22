"""
apc220_receiver.py — SouthernLights CanSat 2026
APC220 ground station receiver and decoder.

Reads binary telemetry frames from the APC220 receiver over serial,
validates CRC, unpacks TxPacket, prints human-readable output and logs CSV.

Frame format (matches cansat_m7/telemetry.cpp and data_types.h):
    [0xAA][0x55][LEN:1][...TxPacket: 59 bytes...][CRC_HI:1][CRC_LO:1]
    Total: 64 bytes per packet.

CRC16-CCITT: poly 0x1021, init 0xFFFF, covers LEN byte + payload.

Usage:
    python apc220_receiver.py [port] [baud]
    python apc220_receiver.py COM14 9600

Requires: pyserial  (pip install pyserial)
Output:   console + apc220_log_YYYYMMDD_HHMMSS.csv in current directory
"""

import serial
import struct
import datetime
import sys
import os

# ---------------------------------------------------------------------------
# Frame constants — must match data_types.h exactly
# ---------------------------------------------------------------------------
SYNC_1 = 0xAA
SYNC_2 = 0x55

# TxPacket struct layout (little-endian, packed — __attribute__((packed)) in C++)
#   H   = uint16  sequence
#   I   = uint32  timestamp_ms
#   I   = uint32  boot_epoch
#   B   = uint8   state (FlightState enum)
#   B   = uint8   m4_errors
#   B   = uint8   m7_errors
#   B   = uint8   flags (bit 0 = FLAG_NICLA_BLE_OK)
#   fff fff = 6x float  PrimaryData (temp, press, hum, alt_baro, temp2, press2)
#   fffff   = 5x float  SecondaryData floats (lat, lon, alt_gps, speed_ms, heading)
#   B   = uint8   gps_satellites
#   ?   = bool    gps_fix
PACKET_FMT  = '<HIIBBBBfffffffffffB?xx'  # xx = 2 trailing pad bytes in SecondaryData
FLAG_NICLA_BLE_OK = 0x01
PAYLOAD_SIZE = struct.calcsize(PACKET_FMT)   # 59 bytes
FRAME_SIZE   = 3 + PAYLOAD_SIZE + 2          # sync(2) + len(1) + payload + crc(2) = 64

FLIGHT_STATES = {
    0: 'BOOT',
    1: 'PAD',
    2: 'ASCENT',
    3: 'APOGEE',
    4: 'DESCENT',
    5: 'LANDED',
}

CSV_HEADER = (
    "seq,ts_ms,utc_ts,utc_time,state,"
    "temp1_C,press1_hPa,hum_pct,alt_baro_m,temp2_C,press2_hPa,"
    "lat,lon,alt_gps_m,speed_ms,heading_deg,gps_sats,gps_fix,"
    "m4_errors,m7_errors,nicla_ble_ok"
)


# ---------------------------------------------------------------------------
# CRC16-CCITT — identical to telemetry.cpp
# Poly 0x1021, init 0xFFFF. Covers LEN byte + TxPacket payload.
# ---------------------------------------------------------------------------
def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
        crc &= 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# Packet parser
# ---------------------------------------------------------------------------
def unpack_packet(payload: bytes) -> dict:
    f = struct.unpack(PACKET_FMT, payload)
    return {
        'sequence':       f[0],
        'timestamp_ms':   f[1],
        'boot_epoch':     f[2],
        'state':          FLIGHT_STATES.get(f[3], f'UNKNOWN({f[3]})'),
        'm4_errors':      f[4],
        'm7_errors':      f[5],
        'flags':          f[6],
        'temperature':    f[7],
        'pressure':       f[8],
        'humidity':       f[9],
        'altitude_baro':  f[10],
        'temperature2':   f[11],
        'pressure2':      f[12],
        'latitude':       f[13],
        'longitude':      f[14],
        'altitude_gps':   f[15],
        'speed_ms':       f[16],
        'heading':        f[17],
        'gps_satellites': f[18],
        'gps_fix':        f[19],
    }


# ---------------------------------------------------------------------------
# Output
# ---------------------------------------------------------------------------
def utc_str(pkt: dict) -> str:
    if pkt['boot_epoch'] > 0:
        ts = pkt['boot_epoch'] + pkt['timestamp_ms'] // 1000
        return datetime.datetime.utcfromtimestamp(ts).strftime('%H:%M:%S')
    return '--:--:--'


def print_packet(pkt: dict, stats: dict) -> None:
    utc = utc_str(pkt)
    print(f"\n--- Pkt #{pkt['sequence']}  {pkt['timestamp_ms']}ms  UTC {utc}  {pkt['state']}"
          f"  [RX:{stats['rx']} CRC_ERR:{stats['crc_err']} SKIPPED:{stats['skipped']}] ---")
    print(f"  T1:{pkt['temperature']:.1f}°C  P1:{pkt['pressure']:.1f}hPa"
          f"  Hum:{pkt['humidity']:.1f}%  Alt_baro:{pkt['altitude_baro']:.0f}m")
    nicla_ok = bool(pkt['flags'] & FLAG_NICLA_BLE_OK)
    print(f"  T2:{pkt['temperature2']:.1f}°C  P2:{pkt['pressure2']:.1f}hPa"
          f"{'  [Nicla BLE lost]' if not nicla_ok else ''}")
    if pkt['gps_fix']:
        print(f"  GPS: {pkt['latitude']:.6f}, {pkt['longitude']:.6f}"
              f"  {pkt['altitude_gps']:.0f}m  {pkt['speed_ms']:.1f}m/s"
              f"  {pkt['heading']:.0f}°  {pkt['gps_satellites']}sats")
    else:
        print(f"  GPS: no fix  ({pkt['gps_satellites']} sats visible)")
    if pkt['m4_errors'] or pkt['m7_errors']:
        print(f"  *** ERRORS  M4:{pkt['m4_errors']}  M7:{pkt['m7_errors']} ***")


def packet_to_csv(pkt: dict) -> str:
    utc_ts = pkt['boot_epoch'] + pkt['timestamp_ms'] // 1000 if pkt['boot_epoch'] > 0 else 0
    return (
        f"{pkt['sequence']},{pkt['timestamp_ms']},{utc_ts},{utc_str(pkt)},{pkt['state']},"
        f"{pkt['temperature']:.2f},{pkt['pressure']:.2f},{pkt['humidity']:.2f},{pkt['altitude_baro']:.1f},"
        f"{pkt['temperature2']:.2f},{pkt['pressure2']:.2f},"
        f"{pkt['latitude']:.6f},{pkt['longitude']:.6f},{pkt['altitude_gps']:.1f},"
        f"{pkt['speed_ms']:.2f},{pkt['heading']:.1f},{pkt['gps_satellites']},{int(pkt['gps_fix'])},"
        f"{pkt['m4_errors']},{pkt['m7_errors']},{int(bool(pkt['flags'] & FLAG_NICLA_BLE_OK))}"
    )


# ---------------------------------------------------------------------------
# Main receive loop
# ---------------------------------------------------------------------------
def run(port: str, baud: int) -> None:
    log_filename = datetime.datetime.now().strftime('apc220_log_%Y%m%d_%H%M%S.csv')
    log_path = os.path.join(os.path.dirname(__file__), log_filename)

    stats = {'rx': 0, 'crc_err': 0, 'skipped': 0, 'last_seq': None}

    print(f"SouthernLights APC220 receiver")
    print(f"Port: {port}  Baud: {baud}")
    print(f"Expected frame: {FRAME_SIZE} bytes  payload: {PAYLOAD_SIZE} bytes")
    print(f"Logging CSV to: {log_path}")
    print("Waiting for packets... (Ctrl+C to stop)\n")

    with serial.Serial(port, baud, timeout=1) as ser, \
         open(log_path, 'w', newline='') as log:

        log.write(CSV_HEADER + '\n')
        log.flush()

        buf = bytearray()

        while True:
            buf += ser.read(64)

            # Scan for sync pattern
            while len(buf) >= FRAME_SIZE:
                # Find 0xAA 0x55
                idx = -1
                for i in range(len(buf) - 1):
                    if buf[i] == SYNC_1 and buf[i + 1] == SYNC_2:
                        idx = i
                        break

                if idx == -1:
                    # No sync found — discard all but last byte
                    stats['skipped'] += len(buf) - 1
                    buf = buf[-1:]
                    break

                if idx > 0:
                    stats['skipped'] += idx
                    buf = buf[idx:]

                if len(buf) < FRAME_SIZE:
                    break

                # Validate LEN byte
                payload_len = buf[2]
                if payload_len != PAYLOAD_SIZE:
                    print(f"  [bad LEN {payload_len}, expected {PAYLOAD_SIZE} — skipping byte]")
                    stats['skipped'] += 1
                    buf = buf[1:]
                    continue

                frame_end = 3 + payload_len + 2
                if len(buf) < frame_end:
                    break  # wait for more bytes

                # Validate CRC
                crc_region = bytes(buf[2: 2 + 1 + payload_len])
                expected = crc16_ccitt(crc_region)
                received = (buf[frame_end - 2] << 8) | buf[frame_end - 1]

                if expected != received:
                    print(f"  [CRC error: expected 0x{expected:04X} got 0x{received:04X}]")
                    stats['crc_err'] += 1
                    buf = buf[1:]
                    continue

                # Good frame — unpack and output
                payload = bytes(buf[3: 3 + payload_len])
                buf = buf[frame_end:]

                pkt = unpack_packet(payload)
                stats['rx'] += 1

                # Sequence gap detection
                if stats['last_seq'] is not None:
                    gap = (pkt['sequence'] - stats['last_seq'] - 1) & 0xFFFF
                    if 0 < gap < 1000:
                        print(f"  *** SEQUENCE GAP: {gap} packet(s) missed ***")
                stats['last_seq'] = pkt['sequence']

                print_packet(pkt, stats)

                csv_line = packet_to_csv(pkt)
                log.write(csv_line + '\n')
                log.flush()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM7'
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    try:
        run(port, baud)
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
