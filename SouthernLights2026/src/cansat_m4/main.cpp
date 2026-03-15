#include <RPC.h>
#include "DFRobot_BME68x.h"

// ----- Southern lights ---------
// THIS IS THE SECONDARY CORE (M4) LOGIC
// Sensors and main telemetry (APC and LoRa)
#ifdef CORE_CM4

int localLoop;
int cnt = 0;
float temperature, pressure, humidity, gasresistance, altitude;

DFRobot_BME68x_I2C bme(0x77); // 0x77 I2C address

void setup()
{

    // Serial.begin(9600);
    // Serial.println("Southern Lights M4 core booting");

    RPC.begin();
    RPC.call("setH4CoreLoop", localLoop++);

    uint8_t bme_rslt = 1;
    while (bme_rslt != 0)
    {
        bme_rslt = bme.begin();
        if (bme_rslt != 0)
        {
            RPC.call("M4Error");
            RPC.print("bme begin failure!");
            delay(10000);
        }
    }

    localLoop = 0;
    // Serial.println("Boot complete");
}

void loop()
{
    delay(1000);
    bme.setGasHeater(320, 100);
    bme.startConvert();
    bme.update();

    // DFRobot_BME68x unit conversions — library returns fixed-point integers as float:
    //   readTemperature() → 0.01 °C   → divide by 100 → °C
    //   readHumidity()    → 0.001 %RH → divide by 1000 → %RH
    //   readPressure()    → Pa         → divide by 100  → hPa   ← TODO: verify, not yet divided
    //   readAltitude()    → m          → no conversion needed
    temperature   = bme.readTemperature() / 100.0f;  // °C
    pressure      = bme.readPressure();               // Pa or hPa — verify from serial on first run; if ~101325 add / 100.0f
    humidity      = bme.readHumidity()    / 1000.0f;  // %RH
    gasresistance = bme.readGasResistance();           // Ω — M7 ignores until TertiaryData is wired
    altitude      = bme.readAltitude();                // m

    // RPC contract — M7 UpdateSensorPackage expects:
    //   temperature   °C   float
    //   pressure      hPa  float
    //   humidity      %RH  float
    //   gasresistance Ω    float  (M7 currently ignores — reserved for TertiaryData)
    //   altitude      m    float
    RPC.call("UpdateSensorPackage", temperature, pressure, humidity, gasresistance, altitude);

    cnt++;
    if (cnt > 10)
    {
        RPC.call("setH4CoreLoop", localLoop++);
        cnt = 0;
    }
}

#endif