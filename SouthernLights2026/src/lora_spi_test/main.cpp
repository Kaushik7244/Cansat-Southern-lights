/**
 * lora_spi_test/main.cpp — RFM95W SPI feasibility test
 *
 * Direct SX1276 control via Arduino LoRa library. No AT commands, no Vision Shield.
 * Sends a properly framed TxPacket every 2 s so the MKR WAN 1310 ground station
 * can decode it with full telemetry output.
 *
 * Wiring (Portenta H7 Arduino header):
 *   RFM95W MOSI  → D8  (PC_3)
 *   RFM95W MISO  → D10 (PC_2)
 *   RFM95W SCK   → D9  (PI_1)
 *   RFM95W NSS   → D6  (PA_8)
 *   RFM95W DIO0  → D5  (PC_6)
 *   RFM95W RESET → D4  (PC_7)
 *   RFM95W VCC   → 3.3V
 *   RFM95W GND   → GND
 *
 * LED: red=booting, green=running, blue flash=packet sent, red blink=init fail
 */

#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include "data_types.h"

#define LORA_NSS    6     // D6 = PA_8
#define LORA_DIO0   5     // D5 = PC_6
#define LORA_RESET  4     // D4 = PC_7

#define LORA_FREQ       868000000L   // must match ground station
#define LORA_SF         9
#define LORA_BW         125000L
#define LORA_CR         5
#define LORA_TX_POWER   17

static const int LED_ON  = LOW;
static const int LED_OFF = HIGH;
static uint16_t  g_seq   = 0;

// CRC16-CCITT — identical to telemetry.cpp / groundstation_1310
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// Build and send one framed TxPacket: [0xAA][0x55][LEN][payload][CRC_HI][CRC_LO]
static void sendPacket()
{
    TxPacket tx;
    memset(&tx, 0, sizeof(tx));

    tx.sequence      = g_seq++;
    tx.timestamp_ms  = millis();
    tx.boot_epoch    = 0;           // no time sync in test
    tx.state         = STATE_PAD;
    tx.m4_errors     = 0;
    tx.m7_errors     = 0;

    // Dummy sensor values — realistic enough to see on ground station
    tx.primary.temperature   = 22.5f;
    tx.primary.pressure      = 1013.25f;
    tx.primary.humidity      = 45.0f;
    tx.primary.altitude_baro = 10.0f;
    tx.primary.temperature2  = 22.3f;
    tx.primary.pressure2     = 1013.10f;

    tx.secondary.latitude       = 59.123456f;
    tx.secondary.longitude      = 10.654321f;
    tx.secondary.altitude_gps   = 12.0f;
    tx.secondary.speed_ms       = 0.0f;
    tx.secondary.heading        = 0.0f;
    tx.secondary.gps_satellites = 0;
    tx.secondary.gps_fix        = false;

    // Frame: [SYNC1][SYNC2][LEN][...TxPacket...][CRC_HI][CRC_LO]
    const uint8_t payload_len = sizeof(TxPacket);
    uint8_t frame[2 + 1 + payload_len + 2];
    frame[0] = FRAME_SYNC_1;
    frame[1] = FRAME_SYNC_2;
    frame[2] = payload_len;
    memcpy(&frame[3], &tx, payload_len);

    // CRC covers LEN byte + payload (same as parseFrame in ground station)
    uint16_t crc = crc16(&frame[2], 1 + payload_len);
    frame[3 + payload_len]     = (uint8_t)(crc >> 8);
    frame[3 + payload_len + 1] = (uint8_t)(crc & 0xFF);

    LoRa.beginPacket();
    LoRa.write(frame, sizeof(frame));
    LoRa.endPacket();
}

void setup()
{
    pinMode(LEDR, OUTPUT); pinMode(LEDG, OUTPUT); pinMode(LEDB, OUTPUT);
    digitalWrite(LEDR, LED_ON);
    digitalWrite(LEDG, LED_OFF);
    digitalWrite(LEDB, LED_OFF);

    Serial.begin(115200);
    { unsigned long t = millis(); while (!Serial && millis() - t < 4000) ; }
    Serial.println("LoRa SPI test starting...");

    LoRa.setPins(LORA_NSS, LORA_RESET, LORA_DIO0);

    if (!LoRa.begin(LORA_FREQ)) {
        Serial.println("LoRa.begin() FAILED — check wiring");
        while (true) {
            digitalWrite(LEDR, LED_ON);  delay(200);
            digitalWrite(LEDR, LED_OFF); delay(200);
        }
    }

    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSyncWord(0x12);

    Serial.print("LoRa OK — 868MHz SF9 BW125 CR4/5 sync 0x12  frame=");
    Serial.print(2 + 1 + (int)sizeof(TxPacket) + 2);
    Serial.println(" bytes");
    Serial.println("Sending framed TxPackets every 2 s...");

    digitalWrite(LEDR, LED_OFF);
    digitalWrite(LEDG, LED_ON);
}

void loop()
{
    sendPacket();

    Serial.print("TX #"); Serial.println(g_seq - 1);

    digitalWrite(LEDB, LED_ON);  delay(50);
    digitalWrite(LEDB, LED_OFF);

    delay(2000);
}
