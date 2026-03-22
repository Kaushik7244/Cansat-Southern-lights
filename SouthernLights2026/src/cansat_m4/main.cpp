#include <RPC.h>
#include <DFRobot_BME68x.h>
#include <SPI.h>
#include <LoRa.h>
#include "data_types.h"

// Select BME688 bus: define BME_SPI to use SPI (D7-D10), otherwise I2C (Wire, 0x77).
// Set via platformio.ini build_flags: -DBME_SPI
//#define BME_SPI

#ifdef BME_SPI
  #define BME_CS  7    // D7 = PI_0
#else
  #include <Wire.h>
#endif

// RFM95W — shares SPI bus with BME688, separate CS
#define LORA_NSS    6   // D6 = PA_8
#define LORA_DIO0   5   // D5 = PC_6
#define LORA_RESET  4   // D4 = PC_7
#define LORA_FREQ   868000000L
#define LORA_SF     9
#define LORA_BW     125000L
#define LORA_CR     5
#define LORA_POWER  17

//#define SLDEBUG

// Uncomment to match M7 isolation test mode — M4 idles after RPC.begin().
// BME688 is disconnected; attempting bme.begin() would hang forever.
//#define ISOLATION_TEST

// ----- Southern lights ---------
// THIS IS THE SECONDARY CORE (M4) LOGIC
// Sensors, RPC push to M7, and local in-memory flight log.
#ifdef CORE_CM4

// ---------------------------------------------------------------------------
// Sensor globals
// ---------------------------------------------------------------------------
int   localLoop;
int   cnt         = 0;
float temperature, pressure, humidity, gasresistance, altitude;

#ifdef BME_SPI
  DFRobot_BME68x_SPI bme(BME_CS);
#else
  DFRobot_BME68x_I2C bme(0x77, &Wire);
#endif

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
    float    pressure;       // hPa (verify units on first run)
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
    tx.state         = STATE_PAD;       // M4 does not track flight state
    tx.m4_errors     = g_lora_errors;
    tx.m7_errors     = 0;

    tx.primary.temperature   = temperature;
    tx.primary.pressure      = pressure;
    tx.primary.humidity      = humidity;
    tx.primary.altitude_baro = altitude;
    tx.primary.temperature2  = 0.0f;   // BMP390 not available on M4
    tx.primary.pressure2     = 0.0f;

    // secondary — GPS lives on M7; LoRa carries zeros until a safe pull mechanism is added
    tx.secondary.latitude       = 0.0f;
    tx.secondary.longitude      = 0.0f;
    tx.secondary.altitude_gps   = 0.0f;
    tx.secondary.gps_satellites = 0;
    tx.secondary.gps_fix        = false;

    const uint8_t plen = sizeof(TxPacket);
    uint8_t frame[2 + 1 + plen + 2];
    frame[0] = FRAME_SYNC_1;
    frame[1] = FRAME_SYNC_2;
    frame[2] = plen;
    memcpy(&frame[3], &tx, plen);
    uint16_t crc = loraCrc16(&frame[2], 1 + plen);
    frame[3 + plen]     = (uint8_t)(crc >> 8);
    frame[3 + plen + 1] = (uint8_t)(crc & 0xFF);

    LoRa.beginPacket();
    LoRa.write(frame, sizeof(frame));
    LoRa.endPacket();   // blocking — ~330 ms at SF9/BW125
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

static volatile bool m7_ready = false;
static void onM7Ready() { m7_ready = true; }

void setup()
{
#ifdef BME_SPI
    // Both SPI CS lines must be HIGH before SPI.begin() to avoid false selects.
    pinMode(LORA_NSS, OUTPUT);
    digitalWrite(LORA_NSS, HIGH);

    // Assert CS LOW briefly so BME688 latches SPI mode at power-on.
    // Must happen before RPC.begin() and any delay.
    pinMode(BME_CS, OUTPUT);
    digitalWrite(BME_CS, LOW);
    delayMicroseconds(100);
    SPI.begin();
    digitalWrite(BME_CS, HIGH);
#endif
    RPC.begin();
#ifdef ISOLATION_TEST
    // M7 never calls "M7Ready" in isolation mode — idle here to keep M4 silent.
    // RPC.begin() must run first so the inter-core link is established.
    while (true) delay(1000);
#endif

    RPC.bind("DumpM4Log", dumpLog);
    RPC.bind("M7Ready",   onM7Ready);
    RPC.call("setH4CoreLoop", localLoop++);

    // Wait until M7 has completed its own setup (WiFi, Nicla, etc.)
    // before touching the shared I2C bus.
    while (!m7_ready) delay(50);

#ifndef BME_SPI
    // I2C bus exercise — primes the shared Wire bus before bme.begin().
    auto i2cExercise = []() {
        Wire.begin();
        Wire.setClock(400000);
        delay(50);
        uint8_t acks = 0;
        for (uint8_t i = 0; i < 20; i++) {
            Wire.beginTransmission(0x77);
            Wire.write(0xD0);
            Wire.endTransmission();
            uint8_t cnt = Wire.requestFrom((uint8_t)0x77, (uint8_t)1);
            if (cnt > 0) { Wire.read(); acks++; }
            delay(5);
        }
        char buf[50];
        snprintf(buf, sizeof(buf), "M4 I2C exercise: %u/20 reads ACKed\r\n", acks);
        RPC.print(buf);
    };
    i2cExercise();
    RPC.print("M4 setup: I2C bus exercise done\r\n");
#endif

    RPC.print("M4 setup: calling bme.begin()\r\n");
    uint8_t bme_rslt = 1;
    while (bme_rslt != 0) {
        bme_rslt = bme.begin();
        if (bme_rslt != 0) {
            RPC.call("M4Error");
            RPC.print("bme begin failure!\r\n");
            delay(2000);
#ifndef BME_SPI
            i2cExercise();
#endif
        }
    }
    RPC.print("M4 setup: bme.begin() OK\r\n");

    // LoRa init — after BME is confirmed working so SPI is known good
    LoRa.setPins(LORA_NSS, LORA_RESET, LORA_DIO0);
    if (LoRa.begin(LORA_FREQ)) {
        LoRa.setSpreadingFactor(LORA_SF);
        LoRa.setSignalBandwidth(LORA_BW);
        LoRa.setCodingRate4(LORA_CR);
        LoRa.setTxPower(LORA_POWER);
        LoRa.setSyncWord(0x12);
        g_lora_ok = true;
        RPC.print("M4 setup: LoRa OK — 868MHz SF9 BW125\r\n");
    } else {
        g_lora_errors++;
        RPC.print("M4 setup: LoRa FAILED — check RFM95W wiring\r\n");
    }

    localLoop = 0;
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------

void loop()
{
#ifdef SLDEBUG
    RPC.print("M4 loop: delay\r\n");
#endif
    delay(1000);

#ifdef SLDEBUG
    RPC.print("M4 loop: setGasHeater\r\n");
#endif
    bme.setGasHeater(320, 100);

#ifdef SLDEBUG
    RPC.print("M4 loop: startConvert\r\n");
#endif
    bme.startConvert();

#ifdef SLDEBUG
    RPC.print("M4 loop: update\r\n");
#endif
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
        char buf[80];
        snprintf(buf, sizeof(buf), "M4 data: T=%.2f P=%.2f H=%.2f alt=%.1f\r\n",
                 temperature, pressure, humidity, altitude);
        RPC.print(buf);
    }
#endif

    logSample();
    loraSendBME();   // transmit BME data over RFM95W LoRa (~330 ms blocking at SF9)
    RPC.call("UpdateSensorPackage", temperature, pressure, humidity, gasresistance, altitude);

    cnt++;
    if (cnt > 10)
    {
        RPC.call("setH4CoreLoop", localLoop++);
        cnt = 0;
    }
}

#endif
