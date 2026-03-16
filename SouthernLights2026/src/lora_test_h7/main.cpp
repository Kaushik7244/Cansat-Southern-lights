/**
 * lora_test_h7 — SouthernLights LoRa AT+UTX test
 *
 * Portenta H7 + Vision Shield LoRa — raw AT+UTX packet TX via MKRWAN interface.
 *
 * AT+UTX sends an unconfirmed LoRaWAN-mode uplink using the Murata firmware.
 * Sync word is 0x34 (public LoRaWAN) — matches TTLRA confirmed TX.
 *
 * Sequence:
 *   1. lora.begin(EU868) — init Murata
 *   2. lora.dataRate(3)  — DR3 = SF9/BW125 in EU868, matches lora_test_1310
 *   3. TX loop: lora.beginPacket() / write / endPacket every 5 s
 *   4. Report send result (positive = bytes sent, -6 = no network join required)
 *
 * 1310 must be at sync=0x34 to receive these packets.
 */

#include <Arduino.h>
#include <MKRWAN.h>

#ifdef CORE_CM7

LoRaModem lora(SerialLoRa);

static uint32_t g_seq      = 0;
static uint32_t g_last_tx  = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.println("=== LoRa AT+UTX Test — H7 Vision Shield ===\n");

    Serial.print("Murata init... ");
    if (!lora.begin(EU868)) {
        Serial.println("FAILED");
        while (1);
    }
    Serial.println("OK");
    Serial.print("Firmware: "); Serial.println(lora.version());
    Serial.print("DevEUI:   "); Serial.println(lora.deviceEUI());

    // DR3 in EU868 = SF9 / BW125 / CR4/5 — matches lora_test_1310
    lora.dataRate(3);

    // Configure ABP credentials directly via AT commands — avoids AT+JOIN which hangs.
    // Dummy keys are fine; the 1310 receives raw LoRa bytes and ignores LoRaWAN MAC.
    auto atSet = [](const char* cmd) {
        while (SerialLoRa.available()) SerialLoRa.read();
        SerialLoRa.print(cmd); SerialLoRa.print("\r");
        delay(300);
        String r; while (SerialLoRa.available()) r += (char)SerialLoRa.read();
        Serial.print("  "); Serial.print(cmd); Serial.print(" → "); Serial.println(r.length() ? r : "(no response)");
    };
    Serial.println("ABP credential setup:");
    atSet("AT+NJM=0");
    atSet("AT+DEVADDR=12345678");
    atSet("AT+NWKSKEY=00000000000000000000000000000001");
    atSet("AT+APPSKEY=00000000000000000000000000000001");

    Serial.println("\nConfig: DR3 (SF9/BW125) EU868 sync=0x34");
    Serial.println("1310 must be at sync=0x34 to receive.");
    Serial.println("Sending H7 PING every 5 s...\n");
}

void loop()
{
    uint32_t now = millis();

    if (now - g_last_tx >= 5000) {
        g_seq++;

        // Build payload
        char payload[32];
        int plen = snprintf(payload, sizeof(payload), "H7 PING %lu", g_seq);

        lora.beginPacket();
        lora.write((uint8_t*)payload, plen);
        int rc = lora.endPacket();

        Serial.print("TX ["); Serial.print(plen); Serial.print("B] \"");
        Serial.print(payload); Serial.print("\" → rc="); Serial.println(rc);

        if (rc == plen) {
            Serial.println("  +OK (sent)");
        } else if (rc == -6) {
            Serial.println("  -6 LORA_ERROR_NO_NETWORK — need ABP/OTAA join");
        } else {
            Serial.print("  rc="); Serial.println(rc);
        }

        g_last_tx = now;
    }
}

#endif  // CORE_CM7
