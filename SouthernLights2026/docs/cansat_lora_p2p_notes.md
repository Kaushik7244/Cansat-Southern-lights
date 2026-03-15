# CanSat LoRa P2P — Debug Session Notes
## Portenta H7 + Vision Shield + MKR WAN 1310

---

## Hardware Architecture

- Vision Shield LoRa uses **Murata CMWX1ZZABZ-078** (STM32L0 + SX1276 inside)
- Module connects to Portenta H7 via **Serial3 (UART)** — NOT SPI
- RadioLib cannot be used directly (requires SPI register access)
- BSP pin definitions (from `ArduinoCore-mbed`):
  - `LORA_BOOT0 = PG_7`
  - `LORA_RESET = PC_7`
  - `LORA_IRQ_DUMB = PJ_11`
  - `SerialLoRa = Serial3`

---

## Firmware

- The `MKRWANFWUpdate_standalone` sketch flashes the **AT_Slave** binary (`mlm32l07x01`)
- Confirmed version: **ARD-078 1.2.3**
- DevEUI: `a8610a3434397910`
- This firmware has the full AT command set including RF test commands

---

## Critical: MKRWAN.h Patch

The library opens SerialLoRa with wrong framing by default. **Patch required:**

File: `C:\Users\havar\OneDrive\Dokumenter\Arduino\libraries\MKRWAN\src\MKRWAN.h`

Find in `bool begin()`:
```cpp
SerialLoRa.begin(baud, config);
```

Replace with:
```cpp
#ifdef ARDUINO_PORTENTA_H7_M7
  SerialLoRa.begin(baud, SERIAL_8E1);   // Portenta requires 8E1 parity
#else
  SerialLoRa.begin(baud, config);        // MKR WAN 1310 uses original config
#endif
```

**Why:** The Murata module on the Portenta Vision Shield requires `SERIAL_8E1` (8 data bits, even parity, 1 stop bit). The MKR WAN 1310 uses `SERIAL_8N1`. Without this fix, `AT` ping works but all other commands fail on the Portenta.

---

## AT Command Reference (ARD-078 1.2.3)

### Key syntax rules
- Terminator: `\r` only (not `\r\n`)
- Query form: `AT+CMD?` (not `AT+CMD=?`)
- Set form: `AT+CMD=value`
- Run form: `AT+CMD` (no suffix)
- Band is integer: EU868 = `5`

### Working commands confirmed
| Command | Result |
|---------|--------|
| `AT` | `+OK` |
| `AT+VER?` | version string (via library) |
| `AT+DEVEUI?` | DevEUI (via library) |
| `AT+TOFF` | `Test Stop` — stops any active RF test |
| `AT+TCONF?` | dumps current RF config |
| `AT+TCONF=<params>` | sets RF config (see format below) |
| `AT+TRSSI` | starts continuous RSSI scan |
| `AT+TTLRA` | starts continuous LoRa TX burst |
| `AT+TRLRA` | starts LoRa RX listen |

### TCONF format (confirmed working)
```
AT+TCONF=<freq_MHz>:<power_dBm>:<bandwidth_kHz>:<SF>:<CR_ratio>:<LNA>:<PABoost>
```

**Example:**
```
AT+TCONF=868:14:125:7:4/5:0:0
```

- `freq_MHz` — integer MHz, e.g. `868` (NOT Hz)
- `power_dBm` — e.g. `14`
- `bandwidth_kHz` — e.g. `125` (not an index)
- `SF` — spreading factor 7–12
- `CR_ratio` — full ratio string: `4/5`, `4/6`, `4/7`, `4/8`
- `LNA` — 0=off, 1=on
- `PABoost` — 0=off, 1=on

**Formats that DO NOT work:**
- Frequency in Hz: `868100000` → `+ERR_PARAM`
- CR as integer: `1` or `5` → `+ERR_PARAM`
- Bandwidth as index: `0` → `+ERR_PARAM`

### TRSSI output (when working)
```
Rx Test
>>> LNA is OFF       ← status note, not error
>>> RSSI= -45 dBm   ← printed repeatedly when signal detected
```

### TTLRA / TRLRA behaviour
- Both are **continuous** modes — they run until `AT+TOFF`
- `AT+TTLRA` returns nothing while running; `Test Stop` comes after `AT+TOFF`
- `AT+TRLRA` prints `>>> LNA is OFF` immediately on start, then receive events

---

## modem.begin() Init Sequence

The MKRWAN library's `modem.begin(EU868)` must be called before any AT commands work. It:
1. Opens SerialLoRa
2. Drives LORA_BOOT0 LOW, toggles LORA_RESET
3. Runs `autoBaud()` — hammers `AT` until `+OK`
4. Calls `version()` — sends `AT+DEV?` then `AT+VER?`
5. Calls `configureBand(EU868)` — sends `AT+BAND=5`
6. Calls `dutyCycle(true)` for EU868

After `modem.begin()` succeeds, raw AT commands can be sent directly via `SerialLoRa`.

---

## Working TX Sketch (Portenta H7)

```cpp
/**
 * CanSat LoRa P2P TX - Portenta H7 + Vision Shield
 * Requires MKRWAN.h patched for SERIAL_8E1 (see above)
 * Requires 868 MHz u.FL antenna attached before powering
 */
#include <MKRWAN.h>

LoRaModem modem(SerialLoRa);

void sendCmd(const String& cmd, int timeout = 3000) {
  while (SerialLoRa.available()) SerialLoRa.read();
  Serial.print("\n>> "); Serial.println(cmd);
  SerialLoRa.print(cmd + "\r");
  unsigned long t = millis();
  while (millis() - t < timeout) {
    while (SerialLoRa.available()) Serial.write(SerialLoRa.read());
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== CanSat LoRa TX ===");

  if (!modem.begin(EU868)) {
    Serial.println("FAILED - modem.begin()");
    while(1);
  }
  Serial.println("Modem: " + modem.version());
  Serial.println("DevEUI: " + modem.deviceEUI());

  sendCmd("AT+TOFF");   // stop any leftover test mode
  delay(2000);

  // 868 MHz | 14 dBm | BW125 kHz | SF7 | CR4/5 | LNA off | PABoost off
  sendCmd("AT+TCONF=868:14:125:7:4/5:0:0");
  delay(200);
  sendCmd("AT+TCONF?"); // verify

  Serial.println("\nWaiting 8s before TX...");
  delay(8000);          // allow RX side to get ready

  Serial.println("=== TX START ===");
  SerialLoRa.print("AT+TTLRA\r");  // start continuous TX
  delay(3000);                      // transmit for 3 seconds
  sendCmd("AT+TOFF");               // stop TX — returns "Test Stop"
  Serial.println("=== TX DONE ===");
}

void loop() {}
```

---

## Working RX Sketch (MKR WAN 1310)

```cpp
/**
 * CanSat LoRa P2P RX - MKR WAN 1310
 * No MKRWAN.h patch needed for 1310
 * Start this BEFORE the H7 TX sketch
 */
#include <MKRWAN.h>

LoRaModem modem;  // uses Serial1 automatically on MKR WAN 1310

void sendCmd(const String& cmd, int timeout = 3000) {
  while (modem.stream.available()) modem.stream.read();
  Serial.print("\n>> "); Serial.println(cmd);
  modem.stream.print(cmd + "\r");
  unsigned long t = millis();
  while (millis() - t < timeout) {
    while (modem.stream.available()) Serial.write(modem.stream.read());
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("=== CanSat LoRa RX ===");

  if (!modem.begin(EU868)) {
    Serial.println("FAILED - modem.begin()");
    while(1);
  }
  Serial.println("Modem: " + modem.version());

  sendCmd("AT+TOFF");
  delay(2000);

  // Must match TX side exactly
  sendCmd("AT+TCONF=868:14:125:7:4/5:0:0");
  delay(200);
  sendCmd("AT+TCONF?");
  delay(500);

  Serial.println("\n=== RX LISTENING ===");
  sendCmd("AT+TRLRA", 60000);  // listen for 60 seconds
  Serial.println("=== RX DONE ===");
}

void loop() {}
```

---

## Current Status & Next Steps

### Status
- ✅ MKRWAN.h patch confirmed working on Portenta H7
- ✅ `modem.begin(EU868)` succeeds, version and DevEUI confirmed
- ✅ AT command set fully working after init
- ✅ TCONF set/get working with correct format
- ✅ MKR WAN 1310 RX side initialises and enters RX mode
- ❌ No RF link established — Vision Shield not transmitting/receiving

### Hardware fault on current Vision Shield
The Vision Shield LoRa module RF frontend appears dead:
- UART/digital side fully functional (AT commands work)
- RF side silent — TRSSI on 1310 at 1cm range with no antenna hears nothing from H7
- Most likely cause: **ESD damage** to SX1276 RF frontend
- Secondary possibility: PABoost antenna routing (try `PABoost=1` with replacement)

### When replacement Vision Shield arrives
1. Handle with ESD precautions
2. Attach **868 MHz u.FL antenna** before powering (not WiFi antenna — wrong frequency)
3. Flash TX sketch, confirm `AT+TCONF?` returns SF=7
4. Start 1310 RX first, then H7 TX
5. Expect `Test Stop` after `AT+TOFF` on H7
6. Expect RSSI readings on 1310 TRSSI, and receive event on TRLRA
7. If still no link, try `AT+TCONF=868:14:125:7:4/5:0:1` (PABoost=1)

### Next development after RF link confirmed
- Replace continuous TTLRA/TRLRA test mode with `AT+UTX` / `AT+RECV` for framed packets
- Build sensor data serialisation (temp, pressure, humidity, GPS, timestamp)
- Implement flight event detection (launch, apogee, landing) using IMU data
- Integrate APC220 UHF as redundant downlink

---

## Antenna Note
- Vision Shield has two u.FL connectors: one WiFi (2.4 GHz), one LoRa (868 MHz)
- **Must use 868 MHz LoRa antenna** — WiFi antenna is wrong frequency and will not radiate
- Never connect/disconnect antenna while powered
- u.FL connectors rated ~30 insertion cycles — handle carefully
