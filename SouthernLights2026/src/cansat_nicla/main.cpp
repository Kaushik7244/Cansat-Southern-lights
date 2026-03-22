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
 */

#include "Arduino_BHY2.h"

void setup()
{
    BHY2.begin(NICLA_STANDALONE);
}

void loop()
{
    BHY2.update();
}
