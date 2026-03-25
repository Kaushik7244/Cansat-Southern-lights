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

#define BME_CS  7      // D7 = PI_0 — must be Arduino pin number, not PinName

static DFRobot_BME68x_SPI bme(BME_CS);

static const int LED_ON  = LOW;
static const int LED_OFF = HIGH;

void setup()
{
    // Assert CS LOW immediately — BME688 latches SPI mode at power-on based on CS.
    // This must happen before any delay or Serial init.
    pinMode(BME_CS, OUTPUT);
    digitalWrite(BME_CS, LOW);
    delayMicroseconds(100);
    SPI.begin();
    digitalWrite(BME_CS, HIGH);   // deselect after SPI bus is up

    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, LED_ON);
    digitalWrite(LEDG, LED_OFF);
    digitalWrite(LEDB, LED_OFF);

    Serial.begin(115200);
    { unsigned long t = millis(); while (!Serial && millis() - t < 4000) ; }
    Serial.println("BME688 SPI test starting...");
    delay(10);

    // Mode probe — try all 4 SPI modes at 3 speeds.
    // Expected chip ID: 0x61 (BME688). 0xFF = MISO floating. 0x00 = no clock reaching chip.
    const uint8_t  modes[4]  = { SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3 };
    const uint32_t speeds[3] = { 100000, 1000000, 4000000 };
    for (uint8_t s = 0; s < 3; s++) {
        for (uint8_t m = 0; m < 4; m++) {
            digitalWrite(BME_CS, LOW);
            SPI.beginTransaction(SPISettings(speeds[s], MSBFIRST, modes[m]));
            SPI.transfer(0xD0 | 0x80);
            uint8_t chip_id = SPI.transfer(0x00);
            SPI.endTransaction();
            digitalWrite(BME_CS, HIGH);
            Serial.print(speeds[s]); Serial.print("Hz Mode"); Serial.print(m);
            Serial.print(" ID: 0x"); Serial.print(chip_id, HEX);
            Serial.println(chip_id == 0x61 ? "  <-- OK" : "");
            delay(10);
        }
    }
    SPI.end();

    // Re-latch CS after probe — BME688 needs CS LOW→HIGH with SPI active
    digitalWrite(BME_CS, LOW);
    delayMicroseconds(100);
    SPI.begin();
    digitalWrite(BME_CS, HIGH);
    delay(10);

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
