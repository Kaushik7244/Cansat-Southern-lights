/**
 * iic_uart_test/main.cpp
 *
 * DFRobot WK2132 — forwards $GNRMC and $GNGGA from GPS to APC220.
 * LED: red=booting, green=running, blue flash=sentence forwarded, red blink=init fail
 */

#include <Arduino.h>
#include <Wire.h>
#include <DFRobot_IICSerial.h>

static DFRobot_IICSerial ch1(Wire, SUBUART_CHANNEL_1, /*IA1=*/0, /*IA0=*/0);  // APC220
static DFRobot_IICSerial ch2(Wire, SUBUART_CHANNEL_2, /*IA1=*/0, /*IA0=*/0);  // GPS Air530

static const int LED_ON  = LOW;
static const int LED_OFF = HIGH;

static bool g_ok = false;

void setup()
{
    pinMode(LEDR, OUTPUT);
    pinMode(LEDG, OUTPUT);
    pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, LED_ON);
    digitalWrite(LEDG, LED_OFF);
    digitalWrite(LEDB, LED_OFF);

    delay(1000);   // APC220 power-on settle

    bool apc_ok = (ch1.begin(9600) == 0);
    bool gps_ok = (ch2.begin(9600) == 0);
    g_ok = apc_ok && gps_ok;

    if (g_ok) {
        ch1.println("BOOT OK");
        digitalWrite(LEDR, LED_OFF);
        digitalWrite(LEDG, LED_ON);
    } else {
        while (true) {           // halt with red blink = init failed
            digitalWrite(LEDR, LED_ON);  delay(200);
            digitalWrite(LEDR, LED_OFF); delay(200);
        }
    }
}

void loop()
{
    static char    buf[100];
    static uint8_t idx = 0;

    while (ch2.available()) {
        char c = ch2.read();
        if (c == '\n') {
            buf[idx] = '\0';
            if (idx > 0 && (strncmp(buf, "$GNRMC", 6) == 0 ||
                            strncmp(buf, "$GNGGA", 6) == 0)) {
                ch1.println(buf);
                digitalWrite(LEDB, LED_ON);
                delay(20);
                digitalWrite(LEDB, LED_OFF);
            }
            idx = 0;
        } else if (c != '\r' && idx < sizeof(buf) - 1) {
            buf[idx++] = c;
        }
    }
}
