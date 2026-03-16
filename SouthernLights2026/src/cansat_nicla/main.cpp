/**
 * cansat_nicla/main.cpp — SouthernLights CanSat 2026
 *
 * Firmware for the Nicla Sense ME.
 * Runs the BHI260AP sensor hub in standalone mode with BLE enabled.
 *
 * The Portenta H7 (host) connects via BHY2Host.begin(false, NICLA_VIA_BLE).
 * All sensor configuration (sample rate, latency) is sent by the host over BLE —
 * no sensor setup is needed here.
 *
 * BLE advertisement:
 *   Local name : "NICLA"
 *   Service    : 34c2e3bb-34aa-11eb-adc1-0242ac120002
 *   Config char: 34c2e3bd-...  (host writes sensor config)
 *   Data char  : 34c2e3bc-...  (Nicla notifies SensorDataPacket)
 *   LongData   : 34c2e3be-...  (Nicla notifies SensorLongDataPacket)
 *
 * LED:
 *   Red   — booting / BHY2 init
 *   Green — advertising, waiting for host connection
 *   Blue  — host connected, streaming data
 */

#include "Arduino.h"
#include "Arduino_BHY2.h"
#include "Nicla_System.h"

void setup()
{
    nicla::begin();
    nicla::leds.begin();
    nicla::leds.setColor(red);   // red = booting

    // NICLA_AS_SHIELD starts the BHI260 sensor hub, enables BLE advertising as "NICLA",
    // and exposes the sensor service UUIDs that BHY2Host expects.
    // NICLA_STANDALONE runs sensors locally only — no BLE advertising.
    // The host drives all sensor configuration over BLE — nothing to configure here.
    NiclaSettings settings = NICLA_AS_SHIELD;
    BHY2.begin(settings);

    nicla::leds.setColor(green); // green = advertising, waiting for host
}

void loop()
{
    // BHY2.update() drives the BHI260 data pipeline and BLE stack.
    // Must run as fast as possible — do not add delays.
    BHY2.update();
}
