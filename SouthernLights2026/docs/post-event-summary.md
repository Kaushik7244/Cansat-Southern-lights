# SouthernLights CanSat 2026 — Post-Event Summary

**Competition date:** 2026-03-25, Andøya Space Center
**Team:** SouthernLights

---

## Overall result

The team was happy with the outcome overall. The mission was completed and the cansat recovered safely. Both the physical design and the downlink infrastructure worked well under flight conditions. Primary mission temperature ok, pressure failed. Secondary mission pictures success, paraglider guidance partial.

---

## Mission outcomes

### Primary mission — Partial success

Barometric pressure data was **unusable** during the flight. Root causes:

- The BME688 gas heater thermal coupling caused pressure drift (~0.59 hPa/min post-boot), making altitude readings unreliable in the critical early phase.
- P0 (sea-level pressure) was not calibrated to actual conditions at Andøya (actual SLP was ~953 hPa vs standard 1013.25 hPa), causing a +512 m altitude over-read.
- The first ~48 s of sensor data was absent due to BME warm-up time.

All other primary telemetry channels (temperature, humidity, GPS, APC220 downlink) functioned correctly.

### Secondary mission — 50% success

| Sub-task | Result |
|----------|--------|
| Photo collection | **Success** — camera burst write to SD worked well |
| Paraglider recovery | **Success** — worked as designed |
| Guided descent (paraglider) | **Success** — shortened brake line caused the cansat to circle down as expected |
| Yaw-guided navigation | **Failed** — Nicla sensor data unavailable during flight (see below) |

---

## Physical design — Success

The mechanical design held up well under flight conditions. The paraglider deployed and the modified brake line produced the expected circling descent. This confirms the physical integration and recovery system design were correct.

The acryllic transaprent protection for the camera worked well. 

The encapsulation system worked well and was easy to work with once assembled, but integration took a long time.

---

## Biggest failure: Nicla Sense ME sensor loss

The Nicla Sense ME module (BHI260AP IMU + barometer) was **unavailable during the flight**. This had cascading effects:

- **Yaw/orientation data lost** — the navigation system could not guide the cansat to checkpoints.
- **Secondary pressure/temperature channel lost** — no cross-validation of BME688 data.

### Investigation history — three failed approaches

**Phase 1 — ESLOV I2C (abandoned early)**
The initial integration used the ESLOV connector for I2C communication. This failed due to an address conflict: the Portenta H7 PMIC (PF1550) occupies address 0x08 on Wire1, the same bus and address as the Nicla ESLOV slave. Every read call was silently intercepted by the PMIC. ESLOV was abandoned in favour of BLE.

**Phase 2 — BLE via BHY2Host (unstable)**
Switched to BLE (NICLA_VIA_BLE). This worked on the bench and was confirmed during the pre-flight test (Flight 86, 2026-03-16). However, the BLE link was fragile under real conditions: `BHY2Host.update()` could block the M7 main loop indefinitely when the BLE connection dropped, killing all telemetry. A second Nicla module was sourced and flashed days before the flight but the H7↔Nicla BLE connection on the new unit was not fully confirmed before departure.

**Phase 3 — Return to ESLOV I2C (hardware failure)**
A decision was made to switch back to ESLOV/I2C as a more reliable physical link. Both Nicla modules were tested. Each would acknowledge on the bus (I2C address visible in a scan), but **neither ever produced sensor data**. The most likely cause is a physical hardware fault — either on the ESLOV cable itself or on both sensor modules. The root cause was not identified before flight.

### Summary of root causes

1. **Physical hardware fault** — ESLOV cable or both Nicla modules were defective; I2C acknowledge without data delivery.
2. **Late integration** — The Nicla subsystem was never stable enough in the days before the flight to be trusted. Each fix opened a new problem with no time remaining to resolve it.
3. **Library constraints** — Nicla firmware can only be flashed via Arduino IDE (not PlatformIO), library version 1.0.8 required. This added friction to last-minute debugging.

---

## Project planning — Biggest overall problem

The team identified **delayed integration** as the primary project management failure. Key issues:

- Software subsystems (Nicla BLE, LoRa, navigation) were developed in isolation and only integrated close to the competition deadline.
- Late integration left insufficient time to validate the full system under realistic conditions before flight.
- Hardware/firmware debugging (ESLOV → BLE migration, LoRa Vision Shield AT failures, RFM95W SPI wiring) consumed time that should have been available for end-to-end system tests.
- The LoRa RFM95W module was destroyed during final integration. It was confirmed working the day before the flight but was not operational on flight day — a direct consequence of handling sensitive hardware under time pressure without a spare.

**Lesson:** Integration testing should begin no later than 4 weeks before the flight date, with a full-stack "dress rehearsal" at least 1 week before.

---

## What worked well

| System | Status |
|--------|--------|
| Portenta H7 dual-core (M7+M4) RPC | Stable throughout |
| APC220 433 MHz downlink | Solid — decoded correctly by Python ground station |
| LoRa downlink (RFM95W via SPI on M4) | **Not used in competition** — module destroyed during integration. Confirmed working the day before launch; not operational on flight day. |
| SD card logging | Working |
| GPS fix and telemetry | Working |
| WiFi + NTP time sync | Working |
| Flight state machine | Working (but see state machine issue in post-flight-code-improvements.md) |
| BME688 temperature & humidity | Working |
| Physical recovery system (paraglider) | Working — circling descent confirmed |
| Camera / photo collection | Working — burst write to SD successful |
| Telemetry packet format (TxPacket) | Solid — CRC16-CCITT framing, decoded correctly throughout flight |

---

## Key technical lessons

1. **ESLOV I2C on Portenta H7 Wire1 is unusable** due to the PMIC PF1550 address conflict at 0x08. If ESLOV is used on a different host platform, verify no address collision exists before committing to it.
2. **I2C acknowledge does not mean data delivery.** Both Nicla modules responded to a bus scan but produced no sensor output. Always verify a sensor delivers valid data under load, not just that it ACKs.
3. **BHY2Host.update() must have a timeout guard** for any future BLE-based Nicla integration. A dropped BLE link without a guard kills the entire M7 main loop.
4. **Barometric P0 must be calibrated to local conditions** using a GPS-referenced ground measurement before launch.
5. **BME gas heater thermally couples into the pressure sensor** — disable the heater or discard the first 60–90 s of pressure data.
6. **The flight state machine did not trigger on drone drop** — altitude rate-of-change trigger is needed in addition to the acceleration-based trigger.
7. **Handle sensitive hardware modules carefully under time pressure, and always carry a spare.** The RFM95W LoRa module was working the day before the flight and was destroyed during integration on flight day.

---

## References

- [post-flight-code-improvements.md](post-flight-code-improvements.md) — Specific code fixes identified from flight data analysis
- [cansat_lora_p2p_notes.md](cansat_lora_p2p_notes.md) — LoRa investigation history
- [data_ownership.md](data_ownership.md) — Sensor/data responsibilities per team member
