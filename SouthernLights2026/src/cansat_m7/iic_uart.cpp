/**
 * iic_uart.cpp — SouthernLights CanSat 2026
 *
 * DFRobot IIC to Dual UART (DFR0627, WK2132) on Wire (I2C3, ESLOV/Qwiic).
 * DIP switches A1=OFF, A0=OFF → I2C base address 0x10.
 * Constructor: (wire, subUartChannel, IA1, IA0)
 *
 * NOTE: Wire (I2C3) is also used by M4 for BME688 (0x77). Both cores share
 * the same physical bus. Transactions are short (<1ms) so collisions are rare,
 * but occasional BME688 read errors on M4 are expected and counted.
 */

#include "iic_uart.h"
#include <Arduino.h>

DFRobot_IICSerial iic_apc(Wire, SUBUART_CHANNEL_1, /*IA1=*/0, /*IA0=*/0);  // APC220
DFRobot_IICSerial iic_gps(Wire, SUBUART_CHANNEL_2, /*IA1=*/0, /*IA0=*/0);  // GPS Air530

bool iicUartInit()
{
    bool apc_ok = (iic_apc.begin(9600) == 0);
    bool gps_ok = (iic_gps.begin(9600) == 0);

    // Set Wire (I2C3) timeout AFTER DFRobot begin() calls.
    // DFRobot_IICSerial::begin() calls Wire.begin() internally,
    // which resets any previously set timeout.  Setting it here
    // ensures the 200 ms timeout is active for all subsequent I2C ops.
    Wire.setTimeout(200);  // 200 ms — prevents permanent I2C hangs

    Serial.print("IIC-UART: APC220 ch1=");
    Serial.print(apc_ok ? "OK" : "FAIL");
    Serial.print("  GPS ch2=");
    Serial.println(gps_ok ? "OK" : "FAIL");

    return apc_ok && gps_ok;
}
