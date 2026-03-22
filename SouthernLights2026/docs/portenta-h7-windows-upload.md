# Portenta H7 — Windows Upload via DFU (PlatformIO)

## Problem

On some Windows machines, the default PlatformIO upload for Portenta H7 fails because:
- The 1200bps touch resets the board into DFU mode, which does **not** reappear as a COM port
- PlatformIO's `BeforeUpload` hook times out waiting for a serial port, consuming the DFU mode window (~10 seconds)
- libusb cannot claim the DFU interface without the correct driver

## One-time setup

### 1. Install Arduino Mbed OS Portenta board support

Open Arduino IDE → Board Manager → search **"Portenta"** → install **"Arduino Mbed OS Portenta Boards"**.
This installs the correct USB/DFU drivers that dfu-util depends on.

### 2. Install WinUSB driver via Zadig

1. Download [Zadig](https://zadig.akeo.ie/) (use version 2.9 or newer)
2. Put the Portenta in DFU mode: **double-tap the reset button** (green LED starts pulsing, USB disconnect/reconnect sound)
3. Open Zadig → **Options → List All Devices**
4. Select the **"@Internal Flash"** entry
5. Set driver to **WinUSB** → click **Replace Driver**

> **Do NOT use libusbK** — it hides the device from Device Manager and breaks detection.
> **STM32CubeProgrammer will not work** — it only supports ST's VID (0x0483), not Arduino's (0x2341).

### 3. Disable USB power management (critical on laptops)

Windows aggressive USB power saving causes the board to drop out of DFU mode mid-upload.

**Disable USB selective suspend:**
Control Panel → Power Options → Change plan settings → Change advanced power settings
→ USB settings → USB selective suspend → **Disabled**

**Disable power management on USB root hubs:**
Device Manager → Universal Serial Bus controllers → right-click each **USB Root Hub**
→ Properties → Power Management → uncheck **"Allow the computer to turn off this device to save power"**

## platformio.ini settings

`upload_protocol = custom` bypasses PlatformIO's `BeforeUpload` serial port detection entirely,
avoiding the timeout that would otherwise exhaust the DFU mode window.

```ini
[env:cansat_m7]
platform = ststm32
board = portenta_h7_m7
framework = arduino
upload_protocol = custom
upload_command = "C:/Users/hfa/.platformio/packages/tool-dfuutil-arduino/dfu-util" -d 0x2341:0x035b -a 0 -s 0x08040000:leave -D "$SOURCE"

[env:cansat_m4]
platform = ststm32
board = portenta_h7_m4
framework = arduino
upload_protocol = custom
upload_command = "C:/Users/hfa/.platformio/packages/tool-dfuutil-arduino/dfu-util" -d 0x2341:0x035b -a 0 -s 0x08100000:leave -D "$SOURCE"
```

Flash addresses:
- M7 core: `0x08040000` (after the 256KB bootloader at `0x08000000`)
- M4 core: `0x08100000` (start of flash bank 2)

dfu-util binary location: `C:/Users/hfa/.platformio/packages/tool-dfuutil-arduino/dfu-util`

## Upload procedure

1. Double-tap reset → green LED pulses (DFU mode, ~10 second window)
2. Run upload immediately:
   ```powershell
   pio run -e cansat_m7 -t upload
   pio run -e cansat_m4 -t upload
   ```
3. Each core requires a separate DFU mode entry — double-tap reset between M7 and M4 uploads

## Skip rebuild — upload last binary directly

```powershell
& "C:/Users/hfa/.platformio/packages/tool-dfuutil-arduino/dfu-util" -d 0x2341:0x035b -a 0 -s 0x08040000:leave -D .pio\build\cansat_m7\firmware.bin
& "C:/Users/hfa/.platformio/packages/tool-dfuutil-arduino/dfu-util" -d 0x2341:0x035b -a 0 -s 0x08100000:leave -D .pio\build\cansat_m4\firmware.bin
```
