/**
 * cam_capture.cpp -- SouthernLights CanSat 2026
 *
 * Standalone Vision Shield HM0360 camera driver.
 * Does NOT use the Arduino Camera library (which creates a global CameraWire
 * MbedI2C instance on the same pins as Wire, corrupting the I2C bus).
 *
 * Uses Wire for HM0360 register I2C, TIM HAL for external clock,
 * and DIRECT REGISTER ACCESS for DCMI + DMA (no HAL DCMI/DMA calls).
 * This avoids linking HAL DCMI/DMA .o files whose boot-time side effects
 * corrupt the I2C bus.
 *
 * Camera external clock uses TIM1_CH1 on PK_1.
 * SERVO_PIN_RIGHT was moved to PG_7 to avoid conflict.
 */

#include "cam_capture.h"
#include <Arduino.h>
#include <Wire.h>
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// HM0360 constants
// ---------------------------------------------------------------------------
static const int CAM_W = 320;
static const int CAM_H = 240;
static const uint32_t FRAME_SZ = CAM_W * CAM_H;

#define HM0360_ADDR         0x24
#define SW_RESET            0x0103
#define MODE_SELECT         0x0100
#define COMMAND_UPDATE      0x0104
#define HIMAX_RESET         0x01
#define HIMAX_Standby       0x00
#define HIMAX_Streaming     0x01

// DCMI CR bits (from CMSIS, but defined here to avoid HAL header)
#ifndef DCMI_CR_CAPTURE
#define DCMI_CR_CAPTURE     (1U << 0)
#define DCMI_CR_CM          (1U << 1)  // Capture mode: 1=snapshot
#define DCMI_CR_CROP        (1U << 2)
#define DCMI_CR_JPEG        (1U << 3)
#define DCMI_CR_ESS         (1U << 4)
#define DCMI_CR_PCKPOL      (1U << 5)  // PCLK polarity: 1=falling
#define DCMI_CR_HSPOL       (1U << 6)
#define DCMI_CR_VSPOL       (1U << 7)
#define DCMI_CR_ENABLE      (1U << 14)
#endif

// DCMI interrupt flags
#ifndef DCMI_RIS_FRAME_RIS
#define DCMI_RIS_FRAME_RIS  (1U << 0)
#define DCMI_ICR_FRAME_ISC  (1U << 0)
#endif

// DMA stream CR bits
#ifndef DMA_SxCR_EN
#define DMA_SxCR_EN         (1U << 0)
#define DMA_SxCR_TCIE       (1U << 4)
#define DMA_SxCR_MINC       (1U << 10)
#define DMA_SxCR_PSIZE_1    (1U << 12)   // 32-bit periph
#define DMA_SxCR_MSIZE_1    (1U << 14)   // 32-bit memory
#define DMA_SxCR_PL_1       (1U << 17)   // Priority high
#endif

// DMAMUX request ID for DCMI on STM32H747
#define DMAMUX_DCMI_REQ     75U

// ---------------------------------------------------------------------------
// HM0360 default register table (QVGA, grayscale, from framework)
// ---------------------------------------------------------------------------
static const uint16_t hm0360_regs[][2] = {
    {0x0103, 0x00}, // SW_RESET
    {0x0370, 0x00}, {0x0371, 0x01}, {0x0372, 0x01},
    {0x1000, 0x01}, {0x1003, 0x04}, {0x1004, 0x04},
    {0x1007, 0x01}, {0x1008, 0x04}, {0x1009, 0x04}, {0x100A, 0x01},
    {0x1014, 0x0C},
    {0x101D, 0x00}, {0x101E, 0x01}, {0x101F, 0x00}, {0x1020, 0x01}, {0x1021, 0x00},
    {0x102F, 0x00},
    {0x1030, 0x09}, {0x1031, 0x12}, {0x1032, 0x23}, {0x1033, 0x31},
    {0x1034, 0x3E}, {0x1035, 0x4B}, {0x1036, 0x56}, {0x1037, 0x5E},
    {0x1038, 0x65}, {0x1039, 0x72}, {0x103A, 0x7F}, {0x103B, 0x8C},
    {0x103C, 0x98}, {0x103D, 0xB2}, {0x103E, 0xCC}, {0x103F, 0xE6},
    {0x3112, 0x00},
    {0x0300, 0x09}, {0x0301, 0x0A}, {0x0302, 0x77},
    {0x3024, 0x08}, {0x3112, 0x00},
    {0x2000, 0x5F}, {0x2001, 0x00},
    {0x2031, 0x20}, {0x2032, 0x00},
    {0x2034, 0x64}, {0x2035, 0x0A}, {0x2036, 0x23},
    {0x2037, 0x03}, {0x2038, 0x05},
    {0x2029, ((0x109-4)>>8)}, {0x202A, ((0x109-4)&0xFF)},
    {0x202B, 0x04}, {0x202C, 0x03}, {0x202D, 0x3F},
    {0x0202, 0x01}, {0x0203, 0x08},
    {0x2080, 0x6A}, {0x2083, 0x01}, {0x209B, 0x01}, {0x209E, 0x06},
    {0x2061, 0x00}, {0x2081, 0xF0}, {0x2082, 0xF0},
    {0x0340, (0x109>>8)}, {0x0341, (0x109&0xFF)},
    {0x0342, (0x178>>8)}, {0x0343, (0x178&0xFF)},
    {0x0380, 0x01}, {0x0381, 0x01}, {0x0382, 0x00},
    {0x3030, 0x00}, {0x0101, 0x00}, {0x0104, 0x01},
    {0x3010, 0x00}, {0x3013, 0x01}, {0x3019, 0x00}, {0x301A, 0x00},
    {0x301B, 0x20}, {0x301C, 0xFF},
    {0x3026, 0x03}, {0x3027, 0x81}, {0x3028, 0x01}, {0x3029, 0x00},
    {0x302A, 0x30}, {0x302E, 0x00}, {0x302F, 0x00},
    {0x302B, 0x2A}, {0x302C, 0x00}, {0x302D, 0x03},
    {0x3031, 0x01}, {0x3051, 0x00}, {0x305C, 0x03},
    {0x3060, 0x00}, {0x3061, 0xFA}, {0x3062, 0xFF}, {0x3063, 0xFF},
    {0x3064, 0xFF}, {0x3065, 0xFF}, {0x3066, 0xFF}, {0x3067, 0xFF},
    {0x3068, 0xFF}, {0x3069, 0xFF}, {0x306A, 0xFF}, {0x306B, 0xFF},
    {0x306C, 0xFF}, {0x306D, 0xFF}, {0x306E, 0xFF}, {0x306F, 0xFF},
    {0x3070, 0xFF}, {0x3071, 0xFF}, {0x3072, 0xFF}, {0x3073, 0xFF},
    {0x3074, 0xFF}, {0x3075, 0xFF}, {0x3076, 0xFF}, {0x3077, 0xFF},
    {0x3078, 0xFF}, {0x3079, 0xFF}, {0x307A, 0xFF}, {0x307B, 0xFF},
    {0x307C, 0xFF}, {0x307D, 0xFF}, {0x307E, 0xFF}, {0x307F, 0xFF},
    {0x3080, 0x01}, {0x3081, 0x01}, {0x3082, 0x03}, {0x3083, 0x20},
    {0x3084, 0x00}, {0x3085, 0x20}, {0x3086, 0x00}, {0x3087, 0x20},
    {0x3088, 0x00}, {0x3089, 0x04},
    {0x3094, 0x02}, {0x3095, 0x02}, {0x3096, 0x00}, {0x3097, 0x02},
    {0x3098, 0x00}, {0x3099, 0x02},
    {0x309E, 0x05}, {0x309F, 0x02}, {0x30A0, 0x02}, {0x30A1, 0x00},
    {0x30A2, 0x08}, {0x30A3, 0x00}, {0x30A4, 0x20}, {0x30A5, 0x04},
    {0x30A6, 0x02}, {0x30A7, 0x02}, {0x30A8, 0x01}, {0x30A9, 0x00},
    {0x30AA, 0x02}, {0x30AB, 0x34},
    {0x30B0, 0x03}, {0x30C4, 0x10}, {0x30C5, 0x01}, {0x30C6, 0xBF},
    {0x30C7, 0x00}, {0x30C8, 0x00},
    {0x30CB, 0xFF}, {0x30CC, 0xFF}, {0x30CD, 0x7F}, {0x30CE, 0x7F},
    {0x30D3, 0x01}, {0x30D4, 0xFF}, {0x30D5, 0x00}, {0x30D6, 0x40},
    {0x30D7, 0x00}, {0x30D8, 0xA7}, {0x30D9, 0x05}, {0x30DA, 0x01},
    {0x30DB, 0x40}, {0x30DC, 0x00}, {0x30DD, 0x27}, {0x30DE, 0x05},
    {0x30DF, 0x07}, {0x30E0, 0x40}, {0x30E1, 0x00}, {0x30E2, 0x27},
    {0x30E3, 0x05}, {0x30E4, 0x47}, {0x30E5, 0x30}, {0x30E6, 0x00},
    {0x30E7, 0x27}, {0x30E8, 0x05}, {0x30E9, 0x87}, {0x30EA, 0x30},
    {0x30EB, 0x00}, {0x30EC, 0x27}, {0x30ED, 0x05}, {0x30EE, 0x00},
    {0x30EF, 0x40}, {0x30F0, 0x00}, {0x30F1, 0xA7}, {0x30F2, 0x05},
    {0x30F3, 0x01}, {0x30F4, 0x40}, {0x30F5, 0x00}, {0x30F6, 0x27},
    {0x30F7, 0x05}, {0x30F8, 0x07}, {0x30F9, 0x40}, {0x30FA, 0x00},
    {0x30FB, 0x27}, {0x30FC, 0x05}, {0x30FD, 0x47}, {0x30FE, 0x30},
    {0x30FF, 0x00}, {0x3100, 0x27}, {0x3101, 0x05}, {0x3102, 0x87},
    {0x3103, 0x30}, {0x3104, 0x00}, {0x3105, 0x27}, {0x3106, 0x05},
    {0x310B, 0x10}, {0x3113, 0xA0}, {0x3114, 0x67}, {0x3115, 0x42},
    {0x3116, 0x10}, {0x3117, 0x0A}, {0x3118, 0x3F},
    {0x311C, 0x10}, {0x311D, 0x06}, {0x311E, 0x0F}, {0x311F, 0x0E},
    {0x3120, 0x0D}, {0x3121, 0x0F}, {0x3122, 0x00}, {0x3123, 0x1D},
    {0x3126, 0x03}, {0x3128, 0x57}, {0x312A, 0x11}, {0x312B, 0x41},
    {0x312E, 0x00}, {0x312F, 0x00}, {0x3130, 0x0C},
    {0x3141, 0x2A}, {0x3142, 0x9F}, {0x3147, 0x18}, {0x3149, 0x18},
    {0x314B, 0x01}, {0x3150, 0x50}, {0x3152, 0x00}, {0x3156, 0x2C},
    {0x315A, 0x0A}, {0x315B, 0x2F}, {0x315C, 0xE0}, {0x315F, 0x02},
    {0x3160, 0x1F}, {0x3163, 0x1F}, {0x3164, 0x7F}, {0x3165, 0x7F},
    {0x317B, 0x94}, {0x317C, 0x00}, {0x317D, 0x02}, {0x318C, 0x00},
    {0x0104, 0x01},  // COMMAND_UPDATE
    {0x0000, 0x00},  // sentinel
};

// ---------------------------------------------------------------------------
// Low-level I2C — 16-bit register address, 8-bit data
// ---------------------------------------------------------------------------
static bool regWrite(uint16_t reg, uint8_t val) {
    Wire.beginTransmission(HM0360_ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

static uint8_t regRead(uint16_t reg) {
    Wire.beginTransmission(HM0360_ADDR);
    Wire.write((uint8_t)(reg >> 8));
    Wire.write((uint8_t)(reg & 0xFF));
    Wire.endTransmission(false);
    Wire.requestFrom(HM0360_ADDR, 1);
    return Wire.available() ? Wire.read() : 0xFF;
}

// ---------------------------------------------------------------------------
// TIM handle — only HAL dependency kept (TIM HAL is already linked by framework)
// ---------------------------------------------------------------------------
static TIM_HandleTypeDef s_htim = {0};

// ---------------------------------------------------------------------------
// Configure TIM1 as camera external clock (6 MHz on PK_1)
// ---------------------------------------------------------------------------
static int configExtClk() {
    __TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOK_CLK_ENABLE();

    GPIO_InitTypeDef hgpio = {};
    hgpio.Pin       = GPIO_PIN_1;
    hgpio.Pull      = GPIO_NOPULL;
    hgpio.Speed     = GPIO_SPEED_HIGH;
    hgpio.Mode      = GPIO_MODE_AF_PP;
    hgpio.Alternate = GPIO_AF1_TIM1;
    HAL_GPIO_Init(GPIOK, &hgpio);

    uint32_t tclk   = HAL_RCC_GetPCLK2Freq() * 2;
    uint32_t period = (tclk / 6000000) - 1;

    s_htim.Instance               = TIM1;
    s_htim.Init.Period            = period;
    s_htim.Init.Prescaler         = 0;
    s_htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
    s_htim.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    s_htim.Init.RepetitionCounter = 0;
    s_htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    TIM_OC_InitTypeDef oc = {};
    oc.Pulse       = period / 2;
    oc.OCMode      = TIM_OCMODE_PWM1;
    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
    oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    oc.OCFastMode  = TIM_OCFAST_DISABLE;

    if (HAL_TIM_Base_Init(&s_htim) != HAL_OK) return -1;
    if (HAL_TIM_PWM_ConfigChannel(&s_htim, &oc, TIM_CHANNEL_1) != HAL_OK) return -1;
    if (HAL_TIM_PWM_Start(&s_htim, TIM_CHANNEL_1) != HAL_OK) return -1;
    return 0;
}

static int configExtClk24() {
    uint32_t tclk   = HAL_RCC_GetPCLK2Freq() * 2;
    uint32_t period = (tclk / 24000000) - 1;
    __HAL_TIM_SET_AUTORELOAD(&s_htim, period);
    __HAL_TIM_SET_COMPARE(&s_htim, TIM_CHANNEL_1, period / 2);
    return 0;
}

// ---------------------------------------------------------------------------
// DCMI pin table
// ---------------------------------------------------------------------------
static const struct { GPIO_TypeDef *port; uint16_t pin; } dcmi_pins[] = {
    {GPIOA, GPIO_PIN_4 }, {GPIOA, GPIO_PIN_6 },
    {GPIOI, GPIO_PIN_4 }, {GPIOI, GPIO_PIN_5 },
    {GPIOI, GPIO_PIN_6 }, {GPIOI, GPIO_PIN_7 },
    {GPIOH, GPIO_PIN_9 }, {GPIOH, GPIO_PIN_10},
    {GPIOH, GPIO_PIN_11}, {GPIOH, GPIO_PIN_12},
    {GPIOH, GPIO_PIN_14},
};

// ---------------------------------------------------------------------------
// Configure DCMI + DMA — PURE REGISTER ACCESS, no HAL_DCMI/HAL_DMA calls
// ---------------------------------------------------------------------------
static int configDCMI() {
    // Enable peripheral clocks
    __HAL_RCC_DCMI_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOI_CLK_ENABLE();
    __HAL_RCC_DMA2_CLK_ENABLE();

    // DCMI GPIO pins — all AF13
    GPIO_InitTypeDef hgpio = {};
    hgpio.Mode      = GPIO_MODE_AF_PP;
    hgpio.Pull      = GPIO_PULLUP;
    hgpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    hgpio.Alternate = GPIO_AF13_DCMI;
    for (auto &p : dcmi_pins) {
        hgpio.Pin = p.pin;
        HAL_GPIO_Init(p.port, &hgpio);
    }

    // --- DMA2 Stream 3 setup (direct register access) ---

    // Disable stream and wait
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream3->CR & DMA_SxCR_EN) {}

    // Clear all interrupt flags for stream 3 (bits 27,26,25,24,22 in LIFCR)
    DMA2->LIFCR = (0x3DU << 22);

    // Configure DMA: P2M, word/word, memory-increment, priority high, no FIFO
    DMA2_Stream3->CR = 0
        | DMA_SxCR_PL_1       // Priority high
        | DMA_SxCR_MSIZE_1    // Memory data size: 32-bit
        | DMA_SxCR_PSIZE_1    // Peripheral data size: 32-bit
        | DMA_SxCR_MINC;      // Memory increment enable
    // Direction = 00 (P2M), no circular, no double-buffer

    DMA2_Stream3->FCR = 0;    // Direct mode (FIFO disabled)

    // DMAMUX: DMA2_Stream3 = DMAMUX1 channel 11 (DMA2 streams 0-7 = channels 8-15)
    DMAMUX1_Channel11->CCR = DMAMUX_DCMI_REQ;

    // --- DCMI setup (direct register access) ---

    // Reset DCMI CR then configure:
    // PCKPOL=falling, everything else default (HS/VS active-low, HW sync, 8-bit, all frames)
    DCMI->CR = 0;
    DCMI->CR = DCMI_CR_PCKPOL;

    // CROP: capture QVGA window starting at (0,0)
    DCMI->CWSTRTR = 0;                                     // Start at pixel 0, line 0
    DCMI->CWSIZER = ((uint32_t)(CAM_H - 1) << 16) | (uint32_t)(CAM_W - 1);
    DCMI->CR |= DCMI_CR_CROP;

    return 0;
}

// ---------------------------------------------------------------------------
// HM0360 sensor init
// ---------------------------------------------------------------------------
static bool sensorReset() {
    for (int tries = 10; tries > 0; tries--) {
        regWrite(SW_RESET, HIMAX_RESET);
        delay(5);
        if (regRead(MODE_SELECT) == HIMAX_Standby) return true;
    }
    return false;
}

static bool sensorInit() {
    if (!sensorReset()) return false;
    delay(100);

    int count = 0, errs = 0;
    for (int i = 0; hm0360_regs[i][0] != 0; i++) {
        uint16_t reg = hm0360_regs[i][0];
        uint8_t  val = (uint8_t)hm0360_regs[i][1];

        // Skip 0x3060-0x307F — writing this block kills HM0360 I2C on shared Wire
        if (reg >= 0x3060 && reg <= 0x307F) { count++; continue; }

        delay(5);
        if (!regWrite(reg, val)) {
            errs++;
            delay(10);
            regWrite(reg, val);  // retry once
        }

        if (reg == SW_RESET)       delay(100);
        if (reg == COMMAND_UPDATE)  delay(200);

        count++;
    }
    Serial.print(count - errs); Serial.print("/"); Serial.print(count);
    Serial.print(" regs"); Serial.flush();

    // Start streaming
    delay(10);
    regWrite(MODE_SELECT, HIMAX_Streaming);
    delay(50);

    uint8_t mode = regRead(MODE_SELECT);
    if (mode != HIMAX_Streaming) {
        Serial.print(" mode=0x"); Serial.print(mode, HEX);
        return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Frame buffer (allocated once)
// ---------------------------------------------------------------------------
static uint8_t *s_fb = nullptr;
static bool     s_ok = false;

// ---------------------------------------------------------------------------
// BMP writer
// ---------------------------------------------------------------------------
static bool writeBMP(FILE* f, const uint8_t* pixels, int w, int h) {
    const uint32_t palette_sz = 256 * 4;
    const uint32_t row_bytes  = (w + 3) & ~3;
    const uint32_t img_sz     = row_bytes * h;
    const uint32_t offset     = 14 + 40 + palette_sz;
    const uint32_t file_sz    = offset + img_sz;
    const int32_t  neg_h      = -h;

    uint8_t hdr[14] = {'B','M'};
    memcpy(&hdr[2],  &file_sz, 4);
    memcpy(&hdr[10], &offset,  4);

    uint8_t dib[40] = {};
    uint32_t dib_sz = 40;
    uint16_t planes = 1, bpp = 8;
    memcpy(&dib[0],  &dib_sz, 4);
    memcpy(&dib[4],  &w,      4);
    memcpy(&dib[8],  &neg_h,  4);
    memcpy(&dib[12], &planes, 2);
    memcpy(&dib[14], &bpp,    2);
    memcpy(&dib[20], &img_sz, 4);

    if (fwrite(hdr, 1, 14, f) != 14) return false;
    if (fwrite(dib, 1, 40, f) != 40) return false;

    for (int i = 0; i < 256; i++) {
        uint8_t e[4] = {(uint8_t)i, (uint8_t)i, (uint8_t)i, 0};
        if (fwrite(e, 1, 4, f) != 4) return false;
    }

    uint8_t pad[3] = {};
    uint32_t pad_bytes = row_bytes - w;
    for (int y = 0; y < h; y++) {
        if (fwrite(&pixels[y * w], 1, w, f) != (size_t)w) return false;
        if (pad_bytes && fwrite(pad, 1, pad_bytes, f) != pad_bytes) return false;
    }
    return true;
}

// ---------------------------------------------------------------------------
// Public interface
// ---------------------------------------------------------------------------

bool camInit() {
    // Allocate frame buffer (32-byte aligned for DMA)
    uint8_t *raw = (uint8_t*)malloc(FRAME_SZ + 32);
    if (!raw) return false;
    s_fb = (uint8_t*)(((uintptr_t)raw + 31) & ~(uintptr_t)31);

    // Start 6 MHz external clock on PK_1 (TIM1_CH1)
    if (configExtClk() != 0) return false;
    HAL_Delay(10);

    // Hardware reset via PC_13
    pinMode(PC_13, OUTPUT);
    digitalWrite(PC_13, LOW);
    HAL_Delay(10);
    digitalWrite(PC_13, HIGH);
    HAL_Delay(20);

    // Switch to 24 MHz (HM0360 native clock)
    configExtClk24();
    HAL_Delay(10);

    // Probe sensor on I2C
    Wire.beginTransmission(HM0360_ADDR);
    if (Wire.endTransmission() != 0) {
        Serial.println("cam: HM0360 not found");
        return false;
    }

    // Configure sensor registers at 100 kHz I2C
    Wire.setClock(100000);
    Wire.setTimeout(500);
    delay(10);
    if (!sensorInit()) {
        Serial.println("cam: sensor init failed");
        return false;
    }

    // Configure DCMI + DMA
    if (configDCMI() != 0) {
        Serial.println("cam: DCMI failed");
        return false;
    }

    // Restore Wire for WK2132 / Nicla — 100 kHz is safe for WK2132
    Wire.setClock(100000);
    Wire.setTimeout(200);

    s_ok = true;
    return true;
}

bool camCaptureToSD(const char* path) {
    if (!s_ok || !s_fb) return false;

    // --- Start snapshot capture via direct DMA + DCMI registers ---

    // Disable DMA stream, wait for it to stop
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;
    while (DMA2_Stream3->CR & DMA_SxCR_EN) {}

    // Clear DMA stream 3 interrupt flags
    DMA2->LIFCR = (0x3DU << 22);

    // Set DMA addresses and transfer count
    DMA2_Stream3->PAR  = (uint32_t)&DCMI->DR;
    DMA2_Stream3->M0AR = (uint32_t)s_fb;
    DMA2_Stream3->NDTR = FRAME_SZ / 4;

    // Enable DMA stream
    DMA2_Stream3->CR |= DMA_SxCR_EN;

    // Clear DCMI frame flag, then start snapshot capture
    DCMI->ICR = 0x1F;  // Clear all DCMI interrupt flags
    DCMI->CR |= DCMI_CR_CM;      // Snapshot mode
    DCMI->CR |= DCMI_CR_ENABLE;  // Enable DCMI
    DCMI->CR |= DCMI_CR_CAPTURE; // Start capture

    // Poll for frame complete (DCMI clears CAPTURE bit when frame done)
    bool ok = false;
    for (uint32_t t0 = millis(); ;) {
        if (DCMI->RISR & DCMI_RIS_FRAME_RIS) { ok = true; break; }
        if ((millis() - t0) > 3000) break;
        delay(1);
    }

    // Stop DCMI + DMA
    DCMI->CR &= ~(DCMI_CR_CAPTURE | DCMI_CR_ENABLE);
    DMA2_Stream3->CR &= ~DMA_SxCR_EN;

    if (!ok) {
        Serial.println("  cam: capture timeout");
        return false;
    }

    // Invalidate D-cache for the frame buffer
    SCB_InvalidateDCache_by_Addr((uint32_t*)s_fb, FRAME_SZ);

    // Write BMP to SD
    FILE* f = fopen(path, "wb");
    if (!f) {
        Serial.print("  cam: fopen failed: ");
        Serial.println(path);
        return false;
    }

    bool wrote = writeBMP(f, s_fb, CAM_W, CAM_H);
    fclose(f);
    return wrote;
}
