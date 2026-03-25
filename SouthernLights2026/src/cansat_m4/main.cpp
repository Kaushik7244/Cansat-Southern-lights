#include <RPC.h>
#include <DFRobot_BME68x.h>
#include <SPI.h>
#include <LoRa.h>
#include "data_types.h"
#include "shared_memory.h"

// BME688 on SPI — CS on D7 (PI_0)
#define BME_CS  7

// RFM95W — shares SPI bus with BME688, separate CS
#define LORA_NSS    6   // D6 = PA_8
#define LORA_DIO0   5   // D5 = PC_6
#define LORA_RESET  4   // D4 = PC_7
#define LORA_FREQ   868000000L
#define LORA_SF     9
#define LORA_BW     125000L
#define LORA_CR     5
#define LORA_POWER  17

#define SLDEBUG

// Uncomment to match M7 isolation test mode — M4 idles after RPC.begin().
// BME688 is disconnected; attempting bme.begin() would hang forever.
//#define ISOLATION_TEST

// ----- Southern lights ---------
// THIS IS THE SECONDARY CORE (M4) LOGIC
// Sensors, shared-SRAM push to M7, and local in-memory flight log.
#ifdef CORE_CM4

// ---------------------------------------------------------------------------
// Sensor globals
// ---------------------------------------------------------------------------
int   localLoop   = 0;
float temperature, pressure, humidity, gasresistance, altitude;

DFRobot_BME68x_SPI bme(BME_CS);

// ---------------------------------------------------------------------------
// In-memory flight log
//
// Linear buffer — stops when full (does not overwrite).
// Preserves the most valuable data: launch-pad baseline and ascent.
// Sized for ~50 minutes at 1 Hz with comfortable margin inside M4 SRAM.
//
// Memory cost: M4_LOG_SIZE × sizeof(M4Record) = 3000 × 24 = 72 KB
// M4 has ~256 KB dedicated SRAM — 72 KB is well within budget.
//
// Dump is triggered by:
//   1. RPC call "DumpM4Log" from M7 (sent on STATE_LANDED or APC220 DUMP command)
//   2. Direct Serial command (if USB connected during bench testing)
// Dump output is CSV via RPC.print() → appears on M7's USB Serial.
// ---------------------------------------------------------------------------

struct M4Record {
    uint32_t timestamp_ms;
    float    temperature;    // °C
    float    pressure;       // hPa
    float    humidity;       // %RH
    float    gasresistance;  // Ω
    float    altitude;       // m
};

static const uint16_t M4_LOG_SIZE = 3000;
static M4Record  m4_log[M4_LOG_SIZE];
static uint16_t  m4_log_count = 0;
static bool      m4_log_full  = false;

// ---------------------------------------------------------------------------
// LoRa TX — RFM95W on D6 (CS), D5 (DIO0), D4 (RESET)
// Shares SPI bus with BME688 (D7 CS). Both CS lines managed independently.
// ---------------------------------------------------------------------------
static uint16_t g_lora_seq    = 0;
static bool     g_lora_ok     = false;
static uint8_t  g_lora_errors = 0;

// GPS in LoRa: not yet implemented — GPS lives on M7 and appears in APC220 packets.
// RPC struct returns (GPSReply) caused OpenAMP shared-memory issues; parked until
// a safe integer-encoded pull mechanism is implemented.

static uint16_t loraCrc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; b++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static void loraSendBME()
{
    if (!g_lora_ok) return;

    TxPacket tx;
    memset(&tx, 0, sizeof(tx));
    tx.sequence      = g_lora_seq++;
    tx.timestamp_ms  = millis();
    tx.boot_epoch    = 0;
    tx.state         = M4_SHARED->m7_flight_state;  // kept current by M7 via shared SRAM
    tx.m4_errors     = g_lora_errors;
    tx.m7_errors     = 0;
    tx.flags         = (M4_SHARED->nicla_temperature2 != 0.0f) ? FLAG_NICLA_BLE_OK : 0;

    tx.primary.temperature   = temperature;
    tx.primary.pressure      = pressure;
    tx.primary.humidity      = humidity;
    tx.primary.altitude_baro = altitude;
    tx.primary.temperature2  = M4_SHARED->nicla_temperature2;
    tx.primary.pressure2     = M4_SHARED->nicla_pressure2;

    tx.secondary.latitude       = M4_SHARED->gps_latitude;
    tx.secondary.longitude      = M4_SHARED->gps_longitude;
    tx.secondary.altitude_gps   = M4_SHARED->gps_altitude;
    tx.secondary.speed_ms       = M4_SHARED->gps_speed;
    tx.secondary.heading        = M4_SHARED->gps_heading;
    tx.secondary.gps_satellites = M4_SHARED->gps_satellites;
    tx.secondary.gps_fix        = M4_SHARED->gps_fix;

    const uint8_t plen = sizeof(TxPacket);
    uint8_t frame[2 + 1 + plen + 2];
    frame[0] = FRAME_SYNC_1;
    frame[1] = FRAME_SYNC_2;
    frame[2] = plen;
    memcpy(&frame[3], &tx, plen);
    uint16_t crc = loraCrc16(&frame[2], 1 + plen);
    frame[3 + plen]     = (uint8_t)(crc >> 8);
    frame[3 + plen + 1] = (uint8_t)(crc & 0xFF);

    // BME688 SPI operations corrupt the LoRa radio's registers on the shared
    // bus. Soft-resets (idle, re-config) are unreliable because the SPI reads
    // they depend on also return corrupt data.
    //
    // Fix: hardware-reset the radio via LoRa.begin() before every TX.
    // This toggles the RESET pin, forcing the SX1276 into a known state,
    // then reconfigures all registers from scratch over clean SPI.
    // Cost: ~25 ms — negligible in a 1 Hz TX cycle.
    LoRa.begin(LORA_FREQ);
    LoRa.setSpreadingFactor(LORA_SF);
    LoRa.setSignalBandwidth(LORA_BW);
    LoRa.setCodingRate4(LORA_CR);
    LoRa.setTxPower(LORA_POWER);
    LoRa.setSyncWord(0x12);

    int bp = LoRa.beginPacket();
    if (bp == 0) {
        g_lora_errors++;
#ifdef SLDEBUG
        RPC.print("LoRa: beginPacket FAILED\r\n");
#endif
        return;
    }

    size_t written = LoRa.write(frame, sizeof(frame));
    LoRa.endPacket(true);   // async — starts TX, returns immediately

    // Wait for TX to complete before returning to loop (shared SPI bus).
    // Air time at SF9/BW125/67 bytes ≈ 411 ms. 500 ms gives safe margin.
    delay(500);

#ifdef SLDEBUG
    {
        char buf[80];
        snprintf(buf, sizeof(buf),
                 "LoRa TX: seq=%u bp=%d wr=%u/%u\r\n",
                 (unsigned)(g_lora_seq - 1), bp, (unsigned)written,
                 (unsigned)sizeof(frame));
        RPC.print(buf);
    }
#endif
}

static void logSample()
{
    if (m4_log_full) return;
    m4_log[m4_log_count] = {
        millis(),
        temperature, pressure, humidity, gasresistance, altitude
    };
    m4_log_count++;
    if (m4_log_count >= M4_LOG_SIZE) {
        m4_log_full = true;
        RPC.print("M4 log full — ");
        RPC.print(m4_log_count);
        RPC.print(" records retained\r\n");
    }
}

void dumpLog()
{
    RPC.print("M4_LOG_START\r\n");
    RPC.print("ts_ms,temp_C,press,hum_pct,gas_ohm,alt_m\r\n");
    char line[80];
    for (uint16_t i = 0; i < m4_log_count; i++) {
        snprintf(line, sizeof(line), "%lu,%.2f,%.2f,%.2f,%.0f,%.1f\r\n",
                 (unsigned long)m4_log[i].timestamp_ms,
                 m4_log[i].temperature,
                 m4_log[i].pressure,
                 m4_log[i].humidity,
                 m4_log[i].gasresistance,
                 m4_log[i].altitude);
        RPC.print(line);
    }
    RPC.print("M4_LOG_END\r\n");
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------

void setup()
{
    // Both SPI CS lines must be HIGH before SPI.begin() to avoid false selects.
    pinMode(LORA_NSS, OUTPUT);
    digitalWrite(LORA_NSS, HIGH);

    // Assert BME688 CS LOW briefly so it latches SPI mode at power-on,
    // then release before SPI.begin() hands control to the library.
    pinMode(BME_CS, OUTPUT);
    digitalWrite(BME_CS, LOW);
    delayMicroseconds(100);
    SPI.begin();
    digitalWrite(BME_CS, HIGH);

    RPC.begin();
#ifdef ISOLATION_TEST
    // In isolation mode M4 idles here so it never writes to shared SRAM or touches sensors.
    // RPC.begin() must run first so the inter-core print channel is ready.
    while (true) delay(1000);
#endif

    // DumpM4Log is still triggered by M7 via RPC on landing — keep this binding.
    // M7→M4 RPC calls do NOT block M4's loop (they arrive as callbacks).
    RPC.bind("DumpM4Log", dumpLog);

    uint8_t bme_rslt = 1;
    while (bme_rslt != 0) {
        bme_rslt = bme.begin();
        if (bme_rslt != 0) {
            M4_SHARED->m4_errors++;
            delay(2000);
        }
    }

    // LoRa init — after BME is confirmed working so SPI is known good
    LoRa.setPins(LORA_NSS, LORA_RESET, LORA_DIO0);
    if (LoRa.begin(LORA_FREQ)) {
        LoRa.setSpreadingFactor(LORA_SF);
        LoRa.setSignalBandwidth(LORA_BW);
        LoRa.setCodingRate4(LORA_CR);
        LoRa.setTxPower(LORA_POWER);
        LoRa.setSyncWord(0x12);
        g_lora_ok = true;
    } else {
        g_lora_errors++;
        RPC.print("M4: LoRa FAILED\r\n");
    }
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop()
{
    delay(600);
    bme.setGasHeater(320, 100);
    bme.startConvert();
    bme.update();

    // DFRobot_BME68x unit conversions — library returns fixed-point integers as float:
    //   readTemperature() → 0.01 °C   → divide by 100 → °C
    //   readHumidity()    → 0.001 %RH → divide by 1000 → %RH
    //   readPressure()    → Pa         → divide by 100  → hPa   ← verify on first run
    //   readAltitude()    → m          → no conversion needed
    temperature   = bme.readTemperature() / 100.0f;  // °C
    pressure      = bme.readPressure() / 100.0f;      // Pa → hPa (confirmed: raw ~98682)
    humidity      = bme.readHumidity()    / 1000.0f;  // %RH
    gasresistance = bme.readGasResistance();           // Ω
    altitude      = bme.readAltitude();                // m

#ifdef SLDEBUG
    {
        char dbg[120];
        snprintf(dbg, sizeof(dbg),
                 "BME688 #%d: T=%.2f P=%.2f H=%.2f G=%.0f A=%.1f\r\n",
                 localLoop, temperature, pressure, humidity,
                 gasresistance, altitude);
        RPC.print(dbg);
    }
#endif

    logSample();

    // -----------------------------------------------------------------------
    // Write sensor data to shared SRAM BEFORE LoRa TX.
    // LoRa.endPacket() blocks ~330 ms and may hang if the RFM95W is
    // unresponsive — writing SRAM first guarantees M7 gets BME data
    // regardless of LoRa health.
    //
    // Sequence counter pattern ensures M7 never reads a partial snapshot:
    //   seq_write incremented first → signals "write starting"
    //   seq_done  incremented last  → signals "write complete"
    //   M7 only uses the data when seq_write == seq_done.
    // -----------------------------------------------------------------------
    M4_SHARED->seq_write++;
    __DMB();
    M4_SHARED->temperature   = temperature;
    M4_SHARED->pressure      = pressure;
    M4_SHARED->humidity      = humidity;
    M4_SHARED->gasresistance = gasresistance;
    M4_SHARED->altitude      = altitude;
    M4_SHARED->m4_loop       = (uint32_t)(localLoop++);
    __DMB();
    M4_SHARED->seq_done++;


    loraSendBME();   // transmit BME data over RFM95W LoRa (~330 ms blocking at SF9)
}

#endif
