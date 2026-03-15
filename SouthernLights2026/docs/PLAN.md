# CanSat SouthernLights 2026 — Development Plan
**Deadline:** March 22, 2026 | **Competition:** March 23–27

---

## What's Working

| Component | Status | Notes |
|-----------|--------|-------|
| M4 ↔ M7 RPC communication | ✅ Prototype working | |
| SD card write | ✅ Prototype working | |
| BME688 sensor (M4) | ✅ Working | |
| Image capture (Vision Shield) | ✅ Working | |
| Nicla Sense env data | ✅ Working | 1 Hz rate — acceptable for mission |
| APC220 radio link | ✅ Working | C+D pair, 433.920 MHz, 9600 bps |
| LoRa Vision Shield | ⏳ Replacement arriving Monday March 16 before 16:00 | Current unit has ESD-damaged RF frontend |
| GPS (Grove Air530) | 🔲 Not started | |
| Servo / paraglider | 🔲 Not started | |
| Unified M4 + M7 sketch | 🔲 To be built this weekend | |

---

## Code Structure

Build **two fresh unified sketches** — do not merge the prototype sketches. Move prototypes to `reference/` as read-only archive.

```
CanSat/
├── cansat_m4/
│   ├── cansat_m4.ino       ← main loop only, calls modules
│   ├── data_types.h        ← shared packet struct (define this first)
│   ├── sensors.h / .cpp    ← BME688, NTC, Nicla Sense
│   ├── storage.h / .cpp    ← SD card write
│   ├── telemetry.h / .cpp  ← APC220 + LoRa send
│   └── rpc.h / .cpp        ← M4 → M7 data push
├── cansat_m7/
│   ├── cansat_m7.ino       ← main loop only
│   ├── camera.h / .cpp     ← image capture
│   ├── gps.h / .cpp        ← Grove Air530 (stubbed for Kaushik)
│   ├── navigation.h / .cpp ← servo + paraglider (Kaushik's domain)
│   └── rpc.h / .cpp        ← receive from M4
└── reference/              ← prototype sketches, read-only
```

**Rule for the team:** each person owns one file. If Kaushik breaks `navigation.cpp`, he hasn't touched sensors or telemetry.

---

## Architecture Principles (M4 / M7)

- **M4 core** — reliability is the highest goal. Collects sensor data, writes to SD, transmits via radio. Loop must never block.
- **M7 core** — handles camera, GPS, paraglider/servo. Receives sensor data from M4 via RPC.
- Enforce a max runtime per function. No blocking waits in the main loop.
- Avoid repeated expensive `getData()` calls — store result in a local variable and reuse within the loop iteration.

---

## Weekend Plan (March 14–15) — Havar + Teo + Kaushik

| # | Task | Who | Output |
|---|------|-----|--------|
| 1 | Define `data_types.h` — shared packet struct | Havar | Single source of truth for all data fields |
| 2 | Build `cansat_m4.ino` non-blocking loop skeleton | Havar + Teo | Clean main loop |
| 3 | Port BME688 into `sensors.h` / `sensors.cpp` | Teo | Working sensor module |
| 4 | Port SD card write into `storage.h` / `storage.cpp` | Teo | Working storage module |
| 5 | Port M4→M7 RPC into `rpc.h` | Havar | Working RPC module |
| 6 | Build `cansat_m7.ino` skeleton + port camera | Havar + Teo | Working M7 skeleton |
| 7 | Stub `gps.h` with clear TODOs for Kaushik | Havar | Ready for Kaushik Monday |
| 8 | **Finalized pinout schema** | Havar | See todo list |

---

## Week Plan (March 16–21) — Teenagers working independently

### Monday March 16
- LoRa Vision Shield arrives — handle with ESD precautions, attach 868 MHz u.FL antenna before powering
- Kaushik: Get Grove Air530 reading lat/lon/alt/speed into `gps.h`, populate packet struct
- Teo: Add Nicla Sense into `sensors.h`
- APC220 transmitting live sensor packets to ground station

### Tuesday March 17
- LoRa: verify RF link (follow steps in `cansat_lora_p2p_notes.md`), integrate into `telemetry.h` as secondary link
- Kaushik: Begin `navigation.cpp` — servo control + simple GPS heading-to-home

### Wednesday March 18
- Full M4 + M7 integration test — all sensors, both radios, camera running together
- Fix any blocking/crash issues found

### Thursday March 19
- **Feature freeze** — no new features after this point
- Drop/throw test for paraglider deployment
- Ground station receives continuous telemetry for 10+ minutes without interruption

### Friday March 20 – Saturday March 21
- Bug fixes only
- Final hardware assembly in enclosure
- Repeat integration test in enclosure

### Sunday March 22 — Freeze & Pack
- No changes to software
- Pack for travel

---

## Tasks for Emily & Aurora (no code required)

These are real blockers if not done. Emily and Aurora own these.

| Task | Why it matters |
|------|---------------|
| Finalize all wiring and solder connections to match the pinout schema | Nothing works if pins are wrong |
| Label every wire and connector (colored tape or heat shrink labels) | Can't debug in the field without this |
| Battery endurance test — time how long the 18650 pack runs the full system | Need to know if packs last the mission |
| Assemble and test APC220 ground station setup on a laptop | Must work independently before launch day |
| Check both u.FL connectors on Vision Shield — confirm which is LoRa (868 MHz) vs WiFi (2.4 GHz) | Wrong antenna = no radio link |
| Inventory all hardware components and confirm nothing is missing | Prevents surprises during competition week |

---

## Kaushik — GPS + Paraglider Track

Kaushik owns `gps.h` and `navigation.cpp` on M7. His path:

1. **Monday:** Get Grove Air530 reading lat/lon/alt/speed over UART. Fill the GPS fields in the shared packet struct.
2. **Tuesday:** Basic servo control working from M7. Verify physically.
3. **Wednesday:** Simple heading-to-home algorithm using GPS coordinates.
4. **Thursday:** Test in drop/throw scenario.

The GPS data feeds into the shared packet struct, so even if the paraglider doesn't fully work, the GPS telemetry is still a mission win.

---

## Key Risks

| Risk | Mitigation |
|------|-----------|
| LoRa replacement still doesn't link | APC220 is primary — don't gate anything on LoRa |
| Paraglider algorithm too complex in time | Fallback: fixed servo position for glide, skip closed-loop |
| Nicla Sense data rate | 1 Hz is fine for temperature/pressure — don't chase higher rate |
| M4 loop blocking | Enforce max-runtime rule from day one in the skeleton |
| Not enough test time | Hard feature freeze Thursday March 19 |

---

## Telemetry Notes

- **APC220 (primary):** Modules C + D, 433.920 MHz, 9600 bps UART. Wired to Portenta UART1 (D14/D13).
- **LoRa (secondary):** Vision Shield, 868 MHz, MKRWAN library with SERIAL_8E1 patch. See `cansat_lora_p2p_notes.md` for full setup.
- Ground station: PC via CP2102 adapter on COM1 for APC220. MKR WAN 1310 for LoRa RX.
