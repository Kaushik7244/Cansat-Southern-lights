/**
 * cansat_nicla/main.cpp — SouthernLights CanSat 2026
 *
 * Firmware for the Nicla Sense ME.
 * Runs the BHI260AP sensor hub in I2C slave mode (ESLOV connector).
 *
 * The Portenta H7 (host) connects via BHY2Host.begin(false, NICLA_VIA_ESLOV).
 * All sensor configuration (sample rate, latency) is sent by the host over I2C —
 * no sensor setup is needed here.
 */

#include "Arduino_BHY2.h"

void setup()
{
    NiclaConfig cfg = NICLA_I2C;
    BHY2.begin(cfg);
}

void loop()
{
    BHY2.update();
}
