/**
 * lora_test_1310 — SouthernLights LoRa P2P test bench
 *
 * MKR WAN 1310 TX beacon.
 * Transmits a short packet every 2 s and reports channel RSSI every 5 s.
 * Used alongside lora_test_h7 to probe the Vision Shield AT command interface.
 *
 * RF: 868 MHz | BW 125 kHz | SF 9 | CR 4/5 | sync 0x12 | preamble 8
 */

#include <SPI.h>
#include <LoRa.h>

static const long  FREQ      = 868000000L;
static const int   SF        = 9;
static const long  BW        = 125000L;
static const int   CR        = 5;
static const int   SYNC_WORD = 0x34;  // LoRaWAN public — matches Murata AT+UTX / TTLRA
static const int   PREAMBLE  = 8;
static const int   TX_POWER  = 14;

static uint32_t g_seq        = 0;
static uint32_t g_rx         = 0;
static uint32_t g_last_tx_ms = 0;
static uint32_t g_last_rs_ms = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("=== LoRa P2P Test — 1310 TX beacon ===");

    if (!LoRa.begin(FREQ)) {
        Serial.println("LoRa init FAILED");
        while (true);
    }
    LoRa.setSpreadingFactor(SF);
    LoRa.setSignalBandwidth(BW);
    LoRa.setCodingRate4(CR);
    LoRa.setSyncWord(SYNC_WORD);
    LoRa.setPreambleLength(PREAMBLE);
    LoRa.setTxPower(TX_POWER);

    Serial.println("868MHz BW125 SF9 CR4/5 sync=0x34 — listening + beaconing");
}

void loop()
{
    uint32_t now = millis();

    // Transmit beacon every 2 s
    if (now - g_last_tx_ms >= 2000) {
        g_seq++;
        LoRa.beginPacket();
        LoRa.print("PING ");
        LoRa.print(g_seq);
        LoRa.endPacket();
        Serial.print("TX PING "); Serial.println(g_seq);
        g_last_tx_ms = now;
    }

    // Check for incoming packet
    int pktSize = LoRa.parsePacket();
    if (pktSize > 0) {
        String msg;
        while (LoRa.available()) msg += (char)LoRa.read();
        g_rx++;
        Serial.print("RX [");
        Serial.print(pktSize);
        Serial.print("B] RSSI=");
        Serial.print(LoRa.packetRssi());
        Serial.print("dBm SNR=");
        Serial.print(LoRa.packetSnr(), 1);
        Serial.print("dB: ");
        Serial.println(msg);
    }

    // Channel RSSI every 5 s (shows background noise floor)
    if (now - g_last_rs_ms >= 5000) {
        Serial.print("--- idle | TX:");
        Serial.print(g_seq);
        Serial.print(" RX:");
        Serial.println(g_rx);
        g_last_rs_ms = now;
    }
}
