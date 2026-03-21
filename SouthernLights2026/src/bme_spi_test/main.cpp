/**
 * bme_spi_test/main.cpp
 *
 * Standalone BME688 SPI validation — runs on M7, prints readings to USB Serial.
 *
 * Wiring (Portenta H7 Arduino header):
 *   BME688 MOSI → D8  (PC_3)
 *   BME688 SCK  → D9  (PI_1)
 *   BME688 MISO → D10 (PC_2)
 *   BME688 CS   → D7  (PI_0)
 *   BME688 VCC  → 3.3V
 *   BME688 GND  → GND
 *
 * LED: red=booting, green=running, red blink=init failed
 */

#include <Arduino.h>
#include <SPI.h>
#include <DFRobot_BME68x.h>

#define BME_CS  D7

static DFRobot_BME68x_SPI bme(BME_CS);

static const int LED_ON  = LOW;
static const int LED_OFF = HIGH;

void setup()
{
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, LED_ON);
    digitalWrite(LEDG, LED_OFF);
    digitalWrite(LEDB, LED_OFF);

    Serial.begin(115200);
    { unsigned long t = millis(); while (!Serial && millis() - t < 2000) ; }
    Serial.println("BME688 SPI test starting...");

    uint8_t rslt = bme.begin();
    if (rslt != 0) {
        Serial.print("bme.begin() FAILED — result: ");
        Serial.println(rslt);
        while (true) {
            digitalWrite(LEDR, LED_ON);  delay(200);
            digitalWrite(LEDR, LED_OFF); delay(200);
        }
    }

    Serial.println("bme.begin() OK — reading every 2 s");
    digitalWrite(LEDR, LED_OFF);
    digitalWrite(LEDG, LED_ON);
}

void loop()
{
    bme.setGasHeater(320, 100);
    bme.startConvert();
    bme.update();

    float temp  = bme.readTemperature()   / 100.0f;   // °C
    float press = bme.readPressure()      / 100.0f;   // hPa
    float hum   = bme.readHumidity()      / 1000.0f;  // %RH
    float gas   = bme.readGasResistance();             // Ω
    float alt   = bme.readAltitude();                  // m

    char buf[100];
    snprintf(buf, sizeof(buf),
             "T=%.2fC  P=%.2fhPa  H=%.2f%%  Gas=%.0fohm  Alt=%.1fm",
             temp, press, hum, gas, alt);
    Serial.println(buf);

    delay(2000);
}
