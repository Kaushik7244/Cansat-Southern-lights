/**
 * lora_test_h7 — H7 Vision Shield AT mode probe v4
 *
 * Previous findings:
 *  - TCONF= (all formats) → +ERR_PARAM   (read-only on ARD-078 1.2.3)
 *  - Individual setters (TPABOOST, TLNA, TSF…) → +ERR
 *  - TTX=1 → +ERR  (state rejection, not syntax error)
 *  - Without modem.begin(): module stuck in TRLRA → +ERR_RX to everything
 *
 * New hypothesis: modem.begin(EU868) leaves the LoRaWAN stack active, which
 * blocks test-mode TX.  AT+NWM=0 (network-working-mode = LoRa P2P) may
 * disable the LoRaWAN stack and allow AT+TTX.
 *
 * This build probes:
 *  1. AT+NWM? / AT+NWM=0 / AT+NWM=1   — mode switch
 *  2. AT+P2P? / AT+P2P=<params>        — P2P mode (RAK-style)
 *  3. AT+TTEST=1                        — test mode entry guard
 *  4. AT+TTXCW / AT+TTXCWMOD           — CW TX (simpler)
 *  5. AT+TTX variants                   — packet TX
 *  6. AT+TRSSI                          — RSSI (confirms radio alive)
 *  7. 60-second wait then retry TTX     — duty-cycle ruling-out
 */

#include <Arduino.h>
#include <MKRWAN.h>

#ifdef CORE_CM7

#ifndef NODE_ID
#define NODE_ID 1
#endif

static const uint32_t TX_INTERVAL = 5000;
static const uint32_t RX_WINDOW   = 4000;

LoRaModem modem(SerialLoRa);

static uint32_t g_tx = 0;
static uint32_t g_rx = 0;

static String atCmd(const char* cmd, uint32_t timeout_ms = 2000)
{
    delay(50);
    while (SerialLoRa.available()) SerialLoRa.read();
    SerialLoRa.print(cmd); SerialLoRa.print("\r");
    uint32_t deadline = millis() + timeout_ms;
    String r;
    while (millis() < deadline) {
        while (SerialLoRa.available()) r += (char)SerialLoRa.read();
        if (r.indexOf("+OK") >= 0 || r.indexOf("+ERR") >= 0) {
            delay(50);
            while (SerialLoRa.available()) r += (char)SerialLoRa.read();
            break;
        }
    }
    r.trim();
    return r;
}

static void rxWindow(uint32_t duration_ms)
{
    while (SerialLoRa.available()) SerialLoRa.read();
    SerialLoRa.print("AT+TRLRA\r");
    uint32_t deadline = millis() + duration_ms;
    String line;
    while (millis() < deadline) {
        while (SerialLoRa.available()) {
            char c = SerialLoRa.read();
            if (c == '\n') {
                line.trim();
                if (line.length() && !line.equals("+OK")) {
                    g_rx++;
                    Serial.print("RX #"); Serial.print(g_rx);
                    Serial.print("  "); Serial.println(line);
                }
                line = "";
            } else { line += c; }
        }
    }
    while (SerialLoRa.available()) SerialLoRa.read();
    SerialLoRa.print("AT+TOFF\r");
    delay(200);
    while (SerialLoRa.available()) SerialLoRa.read();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);
    Serial.print("\n=== H7 Vision Shield AT probe v4 — Node ");
    Serial.print(NODE_ID); Serial.println(" ===\n");

    Serial.print("Murata init... ");
    if (!modem.begin(EU868)) { Serial.println("FAILED"); while (1); }
    Serial.print("OK  FW="); Serial.println(modem.version());

    Serial.print("TOFF:          "); Serial.println(atCmd("AT+TOFF"));

    // ── 1. Network working mode ────────────────────────────────────────────
    // AT+NWM=0 → LoRa P2P (disables LoRaWAN stack)
    // AT+NWM=1 → LoRaWAN (default)
    Serial.println("--- NWM (network working mode) ---");
    Serial.print("NWM?:          "); Serial.println(atCmd("AT+NWM?", 1000));
    Serial.print("NWM=0 (P2P):   "); Serial.println(atCmd("AT+NWM=0"));
    Serial.print("NWM? (after):  "); Serial.println(atCmd("AT+NWM?", 1000));

    // ── 2. TTX in P2P / plain mode ────────────────────────────────────────
    Serial.println("--- TX after NWM=0 ---");
    Serial.print("TCONF?:        "); Serial.println(atCmd("AT+TCONF?", 1000));
    Serial.print("TPABOOST=1:    "); Serial.println(atCmd("AT+TPABOOST=1"));
    Serial.print("TLNA=1:        "); Serial.println(atCmd("AT+TLNA=1"));
    Serial.print("TTXCW:         "); Serial.println(atCmd("AT+TTXCW", 3000));
    delay(200); SerialLoRa.print("AT+TOFF\r"); delay(200);
    while (SerialLoRa.available()) SerialLoRa.read();
    Serial.print("TTX=1:         "); Serial.println(atCmd("AT+TTX=1", 3000));
    Serial.print("TRSSI:         "); Serial.println(atCmd("AT+TRSSI", 2000));

    // ── 3. Restore LoRaWAN mode, probe TTEST and other guards ─────────────
    Serial.println("--- restore LoRaWAN, probe mode guards ---");
    Serial.print("NWM=1:         "); Serial.println(atCmd("AT+NWM=1"));
    Serial.print("TTEST=1:       "); Serial.println(atCmd("AT+TTEST=1"));
    Serial.print("TTEST?:        "); Serial.println(atCmd("AT+TTEST?", 1000));
    Serial.print("TTX=1 (post):  "); Serial.println(atCmd("AT+TTX=1", 3000));

    // ── 4. Duty-cycle ruling-out: wait 60 s then retry TTX ────────────────
    // At SF12/BW125 a test packet is ~2.8 s → 1% duty = 280 s needed.
    // 60 s won't clear SF12 duty but will clear SF7 (0.05 s air time).
    // If TTX succeeds after 60 s but not immediately: duty cycle is culprit.
    Serial.println("--- duty-cycle wait 60 s ---");
    Serial.print("TOFF:          "); Serial.println(atCmd("AT+TOFF"));
    for (int i = 60; i > 0; i -= 5) {
        Serial.print("  waiting "); Serial.print(i); Serial.println(" s...");
        delay(5000);
    }
    Serial.print("TTX=1 (60s):   "); Serial.println(atCmd("AT+TTX=1", 3000));

    Serial.println("\n--- entering loop ---");

#if NODE_ID == 2
    delay(TX_INTERVAL / 2);
#endif
}

void loop()
{
    g_tx++;
    String resp = atCmd("AT+TTX=1", 2000);
    Serial.print("TX #"); Serial.print(g_tx);
    Serial.print("  "); Serial.println(resp);

    rxWindow(RX_WINDOW);
}

#endif  // CORE_CM7
