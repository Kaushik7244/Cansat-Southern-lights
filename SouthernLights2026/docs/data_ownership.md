# Data Ownership & Global Variable Reference
## SouthernLights CanSat 2026

This document is the authoritative reference for who owns each sensor, what data it produces,
where that data ends up, and who reads and writes every shared global variable.

---

## Sensors

### M4 Core Sensors

| Sensor | Library | Bus | Owned by |
|--------|---------|-----|----------|
| BME688 | DFRobot_BME68X | I2C (0x77) | `cansat_m4/main.cpp` |

#### BME688 — fields produced

| Field | Unit | Destination in packet | Used for |
|-------|------|-----------------------|----------|
| temperature | °C | `g_packet.primary.temperature` | Primary mission — pressure/altitude data collection |
| pressure | hPa | `g_packet.primary.pressure` | Primary mission — pressure/altitude data collection |
| humidity | %RH | `g_packet.primary.humidity` | Secondary data — logged and transmitted |
| altitude_baro | m | `g_packet.primary.altitude_baro` | Transmitted altitude reference (ISA absolute model) |
| gas_resistance | Ω | *(discarded for now)* | Reserved — no field in PrimaryData yet. Will move to `TertiaryData.gas_resistance` when Nicla gas sensor is wired |

> **Note on altitude model:** `readAltitude()` uses the absolute ISA model (assumes 1013.25 hPa at sea level). This gives absolute altitude, not altitude above the launch pad. See Nicla Altitude below for the relative model used by navigation.

> **Note on units:** DFRobot library returns temperature in 0.01 °C units and humidity in 0.001 %RH units. Conversions (`/100` and `/1000`) are applied in `cansat_m4/main.cpp` before the RPC call. Pressure units (Pa vs hPa) should be verified from serial output on first hardware run.

---

### M7 Core Sensors

| Sensor | Library | Bus | Owned by |
|--------|---------|-----|----------|
| Nicla Sense ME — BMP390 (barometer) | Arduino_BHY2Host | SPI/I2C to Nicla | `niclaUpdate()` in `cansat_m7/main.cpp` |
| Nicla Sense ME — BHI260AP (orientation) | Arduino_BHY2Host | SPI/I2C to Nicla | `niclaUpdate()` in `cansat_m7/main.cpp` |
| Grove Air530 GPS | TinyGPSPlus | Serial2 (9600 bps) | `cansat_m7/gps.cpp` — owned by Kaushik |

#### Nicla BMP390 — fields produced

| Field | Unit | Destination | Used for |
|-------|------|-------------|----------|
| temperature2 | °C | `g_packet.primary.temperature2` | Independent temperature verification (primary mission cross-check) |
| pressure2 | hPa | `g_packet.primary.pressure2` | Independent pressure verification (primary mission cross-check) |
| *(derived)* nicla_altitude | m above launch pad | local `nicla_altitude` global | Navigation — parachute deploy trigger, flight state transitions |

> **Relative vs absolute altitude:** `Altitude::get_alt()` (Kaushik's class) computes altitude relative to the conditions recorded at the launch pad (`alt_init`). This is more accurate than the ISA model for flight-event detection. It is used exclusively for navigation decisions and is **not transmitted** — `g_packet.primary.altitude_baro` (from BME688) is what goes over the air.

#### Nicla BHI260AP orientation — fields produced

| Field | Unit | Destination | Used for |
|-------|------|-------------|----------|
| *(derived)* yaw_deg | degrees | local `yaw_deg` global | Navigation — servo heading calculation |

> Yaw is derived from the quaternion output (w, x, y, z) using `atan2(2(wz+xy), 1−2(y²+z²))`.

#### Nicla — fields reserved but not yet wired

These fields exist in `TertiaryData` (stored to SD, never transmitted) but no code populates them yet.

| Field | Sensor | Status |
|-------|--------|--------|
| `tertiary.mag_x/y/z` | BMM150 magnetometer | Not wired — no `SENSOR_ID_MAG` call yet |
| `tertiary.accel_x/y/z` | BHI260AP accelerometer | Not wired — orientation only, not raw accel |
| `tertiary.gyro_x/y/z` | BHI260AP gyroscope | Not wired |
| `tertiary.gas_resistance` | BME688 on Nicla | Not wired |

#### Grove Air530 GPS — fields produced

| Field | Unit | Destination | Used for |
|-------|------|-------------|----------|
| latitude | degrees (+ = North) | `g_packet.secondary.latitude` | Navigation, telemetry |
| longitude | degrees (+ = East) | `g_packet.secondary.longitude` | Navigation, telemetry |
| altitude_gps | m ASL | `g_packet.secondary.altitude_gps` | Telemetry, logged |
| speed_ms | m/s | `g_packet.secondary.speed_ms` | Telemetry, logged |
| heading | degrees 0–360 | `g_packet.secondary.heading` | Telemetry, logged |
| gps_satellites | count | `g_packet.secondary.gps_satellites` | Fix quality indicator |
| gps_fix | bool | `g_packet.secondary.gps_fix` | Guards all position reads in `navigationUpdate()` |
| *(derived)* UTC epoch | seconds | `g_packet.boot_epoch` via `storageSetBootEpoch()` | Timestamps all packets — fires once on first valid fix |

---

## Data Flow Summary

```
M4 core                          M7 core
───────                          ───────
BME688
  └─ readTemperature()           niclaUpdate()
  └─ readPressure()    ─RPC─►    g_packet.primary.temperature
  └─ readHumidity()              g_packet.primary.pressure
  └─ readAltitude()              g_packet.primary.humidity
                                 g_packet.primary.altitude_baro

                                 niclaUpdate()
                                 g_packet.primary.temperature2
                                 g_packet.primary.pressure2
                                 yaw_deg  (local)
                                 nicla_altitude  (local)

                                 gpsPoll()
                                 g_packet.secondary.*

                                 loop() TX interval
                                 g_packet.sequence / timestamp_ms
                                     └─► telemetrySend()   → APC220 + LoRa
                                     └─► g_ring[]          → storageDrain() → SD card
```

---

## Global Variables — M7 (`cansat_m7/main.cpp`)

### Packet & ring buffer

| Variable | Type | Writer(s) | Reader(s) | Notes |
|----------|------|-----------|-----------|-------|
| `g_packet` | `SensorPacket` | `UpdateSensorPackage` (RPC), `niclaUpdate`, `gpsPoll`, `navigationUpdate`, `loop` | `telemetrySend`, `storageDrain`, `printPacketToSerial`, navigation logic | Single live packet — source of truth for all transmission and storage |
| `g_ring[RING_SIZE]` | `SensorPacket[30]` | `loop` (TX interval) | `storageDrain` | Ring buffer decoupling SD writes from the transmit cycle. 30 slots = 30 s headroom |
| `g_ring_head` | `volatile uint8_t` | RPC callback (intended) | `ring_available()`, `storageDrain` | Bumped each time a packet is pushed onto the ring |
| `g_ring_tail` | `uint8_t` | `storageDrain` | `ring_available()`, `storageDrain` | Bumped after each SD write |

> Each TX interval, `loop()` snapshots `g_packet` into `g_ring[g_ring_head % RING_SIZE]` and bumps `g_ring_head` before calling `telemetrySend`. `storageDrain` then drains accumulated slots to SD every 10 s.

### Navigation state

| Variable | Type | Writer | Reader | Notes |
|----------|------|--------|--------|-------|
| `yaw_deg` | `float` | `niclaUpdate` | `navigationUpdate` | Heading in degrees derived from BHI260AP quaternion |
| `nicla_altitude` | `float` | `niclaUpdate` | `navigationUpdate` | Relative altitude above launch pad (metres). Drives all flight-event triggers |
| `alt` | `Altitude` | `niclaUpdate` (init once) | `niclaUpdate` | Kaushik's hypsometric model. Reference set on first Nicla reading |
| `alt_init` | `bool` | `niclaUpdate` | `niclaUpdate`, `navigationUpdate` | Guards altitude calculations until reference is set |
| `crossed_500` | `bool` | `navigationUpdate` | `navigationUpdate` | Set when `nicla_altitude > 480 m`. Latching — never clears |
| `parachute_deployed` | `bool` | `navigationUpdate` | `navigationUpdate`, servo writes | Set when `crossed_500 && nicla_altitude < 60 m`. Enables servo navigation |
| `flight_end` | `bool` | `navigationUpdate` | `navigationUpdate` | Set when checkpoint list exhausted. Combined with `nicla_altitude < 6 m` → `STATE_LANDED` |
| `Target` | `Go_to_checkpoint` | `navigationUpdate` | `navigationUpdate` | Current active checkpoint. Replaced when within 5 m of target |
| `checkpoints_counter` | `int` | `navigationUpdate` | `navigationUpdate`, `arr[]` lookup | Index into the `arr[]` checkpoint array |

### RPC bookkeeping

| Variable | Type | Writer | Reader | Notes |
|----------|------|--------|--------|-------|
| `rtCoreLoop` | `long` | `setH4CoreLoop` (RPC) | `printPacketToSerial` | M4 loop counter — used as a heartbeat indicator |
| `mainCoreLoop` | `long` | `loop` (TX interval) | *(debug only)* | M7 TX counter |
| `timeForLastH4tick` | `time_t` | `setH4CoreLoop` (RPC) | *(available for watchdog use)* | Wall-clock time of last M4 check-in. Could drive an M4 watchdog alert |

### Timing

| Variable | Type | Writer | Reader | Notes |
|----------|------|--------|--------|-------|
| `g_last_tx_ms` | `uint32_t` | `loop` | `loop` | Controls 1 s transmit interval |
| `g_last_drain_ms` | `uint32_t` | `loop` | `loop` | Controls 10 s SD drain interval |

### WiFi

| Variable | Type | Writer | Reader | Notes |
|----------|------|--------|--------|-------|
| `WiFirssi` | `long` | `printWifiStatus` | `printWifiStatus` | Last measured RSSI. Not currently included in transmitted packet |
| `WiFistatus` | `int` | `setup` WiFi connect loop | `setup` WiFi connect loop | Discarded after successful connection |

---

## Global Variables — M4 (`cansat_m4/main.cpp`)

| Variable | Type | Writer | Reader | Notes |
|----------|------|--------|--------|-------|
| `temperature` | `float` | `loop` (BME688 read) | `RPC.call` | Degrees °C — converted from raw before RPC |
| `pressure` | `float` | `loop` (BME688 read) | `RPC.call` | hPa (verify units on first run — may need /100) |
| `humidity` | `float` | `loop` (BME688 read) | `RPC.call` | %RH — converted from raw before RPC |
| `gasresistance` | `float` | `loop` (BME688 read) | `RPC.call` | Ω — M7 discards until TertiaryData wired |
| `altitude` | `float` | `loop` (BME688 read) | `RPC.call` | Metres — ISA absolute model |
| `localLoop` | `int` | `setup`, `loop` | `RPC.call("setH4CoreLoop")` | Sent to M7 every 10 cycles as heartbeat |
| `cnt` | `int` | `loop` | `loop` | Counts loops between heartbeats (resets at 10) |
