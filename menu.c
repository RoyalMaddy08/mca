/**
 * @file    pulser_main_f429.c
 * @brief   Sliding-Precision Pulse Generator — Complete Firmware
 *          Ported from STM32F407VGT6  →  STM32F429ZIT6
 *
 *          MCU  : STM32F429ZIT6  (LQFP144, 2MB Flash, 256KB SRAM, 64KB CCM)
 *          LCD  : LCD-OlinuXino-7TS (800×480, RGB565, LTDC parallel RGB)
 *          Touch: XPT2046 (SPI touch controller on OlinuXino-7TS)
 *          DAC  : AD5791 18-bit SPI DAC (amplitude)
 *          Timer: TIM2 (pulse rep-rate), TIM3 (pulse width), TIM4 (ramp tick)
 *          UART : USART2 (RS-232, PA2/PA3)
 *          SDRAM: IS42S16400J / compatible, FMC Bank 5 → 0xC0000000
 *          NVRAM: Backup SRAM (4KB, VBAT-retained, 10 config slots)
 *
 * ── What changed vs the F407 version ───────────────────────────────────────
 *  1. FMC SDRAM initialisation  (sdram_init) replaces bare FSMC / assumed-
 *     external init.  The F429 FMC Bank 5 (0xC000_0000) hosts a 16-bit wide
 *     SDRAM; timing values match IS42S16400J at 90 MHz SDRAM clock.
 *     Adjust IS42S_* constants below for your specific SDRAM chip.
 *
 *  2. PLLSAI pixel-clock recalculated for F429 at 180 MHz system clock.
 *     PLLSAIN=192, PLLSAIR=4, PLLSAIDIVR_4 → pixel clock ≈ 12 MHz.
 *     For the OlinuXino-7TS you may need ~33 MHz; tune PLLSAIN/R/DIV to
 *     match your panel's HSYNC/VSYNC timing (see ltdc_init comments).
 *
 *  3. Pin map is unchanged (same PA/PB/PC/PD/PE/PF/PG usage) because the
 *     LQFP144 is a superset; all F407 LQFP100 signals exist on F429ZIT6.
 *     FMC SDRAM pins added on PB5, PB6, PC0–PC3, PD0–PD1, PD8–PD10,
 *     PD14–PD15, PE0–PE1, PE7–PE15, PF0–PF5, PF11–PF15, PG0–PG2,
 *     PG4–PG5, PG8, PG15.
 *
 *  4. stdarg.h added explicitly (needed for va_list / va_start / va_end in
 *     fb_printf; F407 HAL pulled it in transitively but F429 HAL may not).
 *
 *  5. C_GREEN colour macro added (referenced in screen_pulse but missing in
 *     the original — defined as bright green).
 *
 *  6. Minor: `va_list ap` was used without including <stdarg.h>; fixed.
 *
 * ── Call from main.c ────────────────────────────────────────────────────────
 *    extern void pulser_run(void);
 *    // After HAL_Init(), SystemClock_Config() [set to 180 MHz], MX_GPIO_Init():
 *    pulser_run();   // never returns
 *
 * ── Pin map (F429ZIT6, unchanged from F407 original) ───────────────────────
 *  LTDC RGB    : see ltdc_gpio_init()
 *  Touch CS    : PA4    Touch CLK: PA5  Touch MISO: PA6  Touch MOSI: PA7
 *  Touch IRQ   : PB0
 *  AD5791 CS   : PB12   AD5791 CLK: PB13  MISO: PB14  MOSI: PB15 (SPI2)
 *  S/H HOLD    : PC0    (high = sample, low = hold)
 *  PULSE OUT   : TIM2 CH1 (PA0)
 *  TRIG OUT    : TIM3 CH1 (PA6, remap if clash with touch MISO)
 *  USART2 TX   : PA2    USART2 RX : PA3
 *  Keypad rows : PD0–PD3  cols : PD4–PD7
 *  Spinner A   : PE0   Spinner B : PE1   Spinner SW : PE2
 *
 * ── FMC SDRAM pin map (F429ZIT6 specific) ──────────────────────────────────
 *  A0–A11 : PF0–PF5, PF12–PF15, PG0–PG2, PG4–PG5
 *  BA0–BA1: PG4–PG5
 *  D0–D15 : PD14–PD15, PD0–PD1, PE7–PE15, PD8–PD10
 *  NBL0–1 : PE0–PE1
 *  SDCLK  : PG8     SDNCAS: PG15    SDNRAS: PF11
 *  SDNE0  : PH3(alt) or PG13 — use PG13 for LQFP144
 *  SDNWE  : PC0 — NOTE: conflicts with S/H pin!
 *            → Move S/H to PC1 in this port (see SH_PIN below).
 * ──────────────────────────────────────────────────────────────────────────*/

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>
#include <stdarg.h>   /* ← added: required for va_list in fb_printf */

/* ============================================================
 *  SECTION 1 — COMPILE-TIME CONFIGURATION
 * ============================================================ */
#define LCD_W               800u
#define LCD_H               480u
#define FRAMEBUF_BASE       0xC0000000u   /* FMC Bank 5 — external SDRAM    */
#define BKPSRAM_BASE        0x40024000u   /* Backup SRAM base (same on F429) */

#define UART_BAUD           9600u
#define UART_RX_BUF         256u
#define CMD_MAX_LEN         80u
#define NUM_CONFIG_SLOTS    10u

/* Keypad: 4 rows × 4 cols */
#define KP_ROWS             4
#define KP_COLS             4

/* Spinner debounce */
#define SPINNER_DEBOUNCE_MS 5u

/* Remote lock escape character (Ctrl-L) */
#define REMOTE_ESCAPE_CHAR  0x0C

/* ============================================================
 *  SECTION 1b — F429 FMC / SDRAM PARAMETERS
 *  Adjust to your SDRAM chip.  Defaults match IS42S16400J @ fSDRAM = 90 MHz.
 *  (System clock 180 MHz, AHB/2 → 90 MHz SDCLK)
 * ============================================================ */
#define SDRAM_MODEREG_BURST_LENGTH_1        0x0000u
#define SDRAM_MODEREG_BURST_LENGTH_2        0x0001u
#define SDRAM_MODEREG_BURST_LENGTH_4        0x0002u
#define SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL 0x0000u
#define SDRAM_MODEREG_CAS_LATENCY_2         0x0020u
#define SDRAM_MODEREG_CAS_LATENCY_3         0x0030u
#define SDRAM_MODEREG_OPERATING_MODE_STANDARD 0x0000u
#define SDRAM_MODEREG_WRITEBURST_MODE_SINGLE  0x0200u

/* IS42S16400J: 64 Mbit, 4M × 16, 4 banks, CAS 3 @ 100 MHz */
#define IS42S_REFRESH_COUNT   1539u   /* 64 ms / 4096 rows @ 90 MHz     */
#define IS42S_CAS_LATENCY     FMC_SDRAM_CAS_LATENCY_3

/* ============================================================
 *  SECTION 2 — COLOUR PALETTE  (RGB565)
 * ============================================================ */
#define PIXEL(r,g,b) ((uint16_t)(((r)&0xF8u)<<8u|((g)&0xFCu)<<3u|(b)>>3u))

#define C_BG        PIXEL(  0,   0,   0)
#define C_FG        PIXEL(  0, 255,   0)   /* normal green text      */
#define C_HL        PIXEL(255, 255,   0)   /* selected / highlight   */
#define C_DIM       PIXEL( 80, 160,  80)   /* dim / inactive         */
#define C_BORDER    PIXEL(  0, 140,   0)
#define C_TITBG     PIXEL(  0,  50,   0)   /* panel title background */
#define C_TITFG     PIXEL(  0, 255,   0)
#define C_WARN      PIXEL(255, 140,   0)   /* orange warning         */
#define C_CYAN      PIXEL(  0, 220, 220)   /* action / execute       */
#define C_BLUE      PIXEL( 80, 160, 255)   /* sliding mode           */
#define C_PINK      PIXEL(255, 100, 180)   /* precision mode         */
#define C_RED       PIXEL(255,  60,  60)   /* error / remote lock    */
#define C_GREEN     PIXEL(  0, 220,  80)   /* start / active         */ /* ← added */
#define C_STATBG    PIXEL(  0,  25,   0)

static volatile uint16_t * const FB = (uint16_t *)FRAMEBUF_BASE;

/* ============================================================
 *  SECTION 3 — DATA STRUCTURES  (unchanged)
 * ============================================================ */

static const uint32_t RISE_NS[] = {50,100,200,500,1000,2000,5000,10000};
static const char    *RISE_STR[]= {"50ns","100ns","200ns","500ns",
                                    "1us","2us","5us","10us"};
#define N_RISE 8u

static const uint32_t FALL_NS[] = {500,1000,2000,5000,10000,20000,
                                    50000,100000,200000,500000,1000000};
static const char    *FALL_STR[]= {"500ns","1us","2us","5us","10us","20us",
                                    "50us","100us","200us","500us","1ms"};
#define N_FALL 11u

static const uint16_t ATTEN_VAL[]= {1,2,5,10,20,50,100,200,500,1000};
static const char    *ATTEN_STR[]= {"1x","2x","5x","10x","20x","50x",
                                     "100x","200x","500x","1000x"};
#define N_ATTEN 10u

typedef enum {
    TRIG_INTERNAL = 0,
    TRIG_EXTERNAL,
    TRIG_GATED,
    TRIG_ONE_PULSE
} TrigSrc_t;

typedef enum {
    OP_PRECISION = 0,
    OP_SLIDING
} OpMode_t;

typedef struct __attribute__((packed)) {
    uint8_t   valid;
    char      label[16];
    uint8_t   op_mode;
    uint8_t   trig_src;
    uint16_t  trig_threshold_mv;
    uint32_t  ampl_uv;
    uint32_t  width_ns;
    uint32_t  rate_hz;
    uint8_t   rise_idx;
    uint8_t   fall_idx;
    uint8_t   atten_idx;
    uint8_t   polarity;
    uint8_t   pulse_top;
    uint8_t   clamp;
    uint32_t  ramp_start_uv;
    uint32_t  ramp_stop_uv;
    uint32_t  ramp_time_s;
    uint16_t  ramp_cycles;
    uint8_t   display_kev;
    uint32_t  kev_full_scale;
    uint8_t   _pad[3];
} PulseConfig_t;

typedef struct __attribute__((packed)) {
    uint32_t      magic;
    PulseConfig_t slots[NUM_CONFIG_SLOTS];
} NVData_t;

typedef struct {
    PulseConfig_t cfg;
    uint8_t       active_slot;
    bool          pulse_on;
    bool          ramp_running;
    bool          remote_mode;
    uint32_t      ramp_ampl_uv;
    uint16_t      ramp_cycle;
    bool          temp_comp_due;
} LiveState_t;

static LiveState_t  G;
static NVData_t    *NV = (NVData_t *)BKPSRAM_BASE;

/* ============================================================
 *  SECTION 4 — 8×16 BITMAP FONT  (ASCII 0x20–0x7E, unchanged)
 * ============================================================ */
#define FONT_FIRST  0x20u
#define FONT_W      8u
#define FONT_H      16u

static const uint8_t FONT[96][16] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x30,0x78,0x78,0x78,0x30,0x30,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x6C,0x6C,0x6C,0x28,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x6C,0x6C,0xFE,0x6C,0x6C,0xFE,0x6C,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x18,0x18,0x7C,0xC6,0xC2,0xC0,0x7C,0x06,0x86,0xC6,0x7C,0x18,0x18,0x00,0x00,0x00},
    {0x00,0x00,0xC2,0xC6,0x0C,0x18,0x30,0x60,0xC6,0x86,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x38,0x6C,0x6C,0x38,0x76,0xDC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x30,0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x0C,0x18,0x30,0x60,0x60,0x60,0x60,0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x60,0x30,0x18,0x0C,0x0C,0x0C,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x18,0x18,0xFF,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x03,0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0xCE,0xD6,0xD6,0xE6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x18,0x38,0x78,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0x06,0x0C,0x18,0x30,0x60,0xC6,0xFE,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0x06,0x3C,0x06,0x06,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x0E,0x1E,0x36,0x66,0xC6,0xFF,0x06,0x06,0x0F,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFE,0xC0,0xC0,0xFC,0x06,0x06,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x38,0x60,0xC0,0xFC,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFE,0xC6,0x06,0x0C,0x18,0x30,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0xC6,0x7C,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0xC6,0x7E,0x06,0x06,0x0C,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x06,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFF,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x60,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0x0C,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0xDE,0xDE,0xDE,0xDC,0xC0,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x10,0x38,0x6C,0xC6,0xFE,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFC,0x66,0x66,0x7C,0x66,0x66,0x66,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x3C,0x66,0xC2,0xC0,0xC0,0xC2,0x66,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xF8,0x6C,0x66,0x66,0x66,0x66,0x6C,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFE,0x62,0x60,0x7C,0x60,0x62,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFE,0x66,0x62,0x78,0x60,0x60,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x3C,0x66,0xC2,0xC0,0xDE,0xC6,0x66,0x3A,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x1E,0x0C,0x0C,0x0C,0xCC,0xCC,0xCC,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xE6,0x66,0x6C,0x78,0x78,0x6C,0x66,0xE6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xF0,0x60,0x60,0x60,0x62,0x66,0x66,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xC6,0xEE,0xFE,0xD6,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xC6,0xE6,0xF6,0xDE,0xCE,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFC,0x66,0x66,0x7C,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0xC6,0xC6,0xD6,0xDE,0x7C,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFC,0x66,0x66,0x7C,0x6C,0x66,0x66,0xE6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x7C,0xC6,0x60,0x38,0x0C,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFF,0xDB,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xC6,0xC6,0xC6,0xD6,0xD6,0xFE,0x6C,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xC6,0x6C,0x38,0x38,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xCC,0xCC,0xCC,0x78,0x30,0x30,0x30,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xFE,0xC6,0x8C,0x18,0x30,0x60,0xC6,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x30,0x30,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x78,0x0C,0x7C,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xE0,0x60,0x7C,0x66,0x66,0x66,0x66,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xC0,0xC0,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x1C,0x0C,0x7C,0xCC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xFE,0xC0,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x1C,0x36,0x30,0xFC,0x30,0x30,0x30,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x76,0xCC,0xCC,0xCC,0x7C,0x0C,0xCC,0x78,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xE0,0x60,0x6C,0x76,0x66,0x66,0x66,0xE6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x18,0x00,0x78,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x06,0x00,0x1E,0x06,0x06,0x06,0x06,0x66,0x3C,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0xE0,0x60,0x66,0x6C,0x78,0x6C,0x66,0xE6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xEC,0xFE,0xD6,0xD6,0xD6,0xD6,0xD6,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xDC,0x66,0x66,0x66,0x7C,0x60,0x60,0xF0,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x76,0xCC,0xCC,0xCC,0x7C,0x0C,0x0C,0x1E,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xDC,0x76,0x62,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0x7C,0xC6,0x60,0x38,0x0C,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x10,0x30,0xFC,0x30,0x30,0x30,0x36,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0xD6,0xD6,0xFE,0xEE,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0x6C,0x38,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0x7E,0x06,0x0C,0xF8,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x00,0xFE,0xCC,0x18,0x30,0x60,0xC6,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x00,0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
    {0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00},
};

/* ============================================================
 *  SECTION 5 — FRAME BUFFER PRIMITIVES  (unchanged)
 * ============================================================ */
static inline void fb_pixel(int x, int y, uint16_t c) {
    if ((unsigned)x < LCD_W && (unsigned)y < LCD_H)
        FB[(uint32_t)y * LCD_W + (uint32_t)x] = c;
}
static void fb_fill(uint16_t c) {
    uint32_t n = LCD_W * LCD_H;
    for (uint32_t i = 0; i < n; i++) FB[i] = c;
}
static void fb_rect(int x, int y, int w, int h, uint16_t c) {
    for (int r = y; r < y+h; r++)
        for (int col = x; col < x+w; col++)
            fb_pixel(col, r, c);
}
static void fb_border(int x, int y, int w, int h, uint16_t c) {
    for (int i = x; i < x+w; i++) { fb_pixel(i,y,c); fb_pixel(i,y+h-1,c); }
    for (int i = y; i < y+h; i++) { fb_pixel(x,i,c); fb_pixel(x+w-1,i,c); }
}
static void fb_char(int px, int py, char ch, uint16_t fg, uint16_t bg) {
    if ((uint8_t)ch < FONT_FIRST || (uint8_t)ch > 0x7Eu) ch = ' ';
    const uint8_t *g = FONT[(uint8_t)ch - FONT_FIRST];
    for (int row = 0; row < (int)FONT_H; row++) {
        uint8_t b = g[row];
        for (int col = 0; col < (int)FONT_W; col++) {
            fb_pixel(px+col, py+row, (b & 0x80u) ? fg : bg);
            b <<= 1u;
        }
    }
}
static int fb_str(int px, int py, const char *s, uint16_t fg, uint16_t bg) {
    while (*s) { fb_char(px, py, *s++, fg, bg); px += FONT_W; }
    return px;
}
static void fb_str_c(int bx, int by, int bw, const char *s,
                     uint16_t fg, uint16_t bg) {
    int len = (int)strlen(s);
    int tw  = len * (int)FONT_W;
    int x   = bx + (bw - tw) / 2;
    if (x < bx) x = bx;
    fb_rect(bx, by, bw, FONT_H, bg);
    fb_str(x, by, s, fg, bg);
}
static int fb_printf(int px, int py, uint16_t fg, uint16_t bg,
                     const char *fmt, ...) {
    char buf[64];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    return fb_str(px, py, buf, fg, bg) - px;
}

/* ============================================================
 *  SECTION 6 — MENU SYSTEM  (unchanged)
 * ============================================================ */
typedef enum {
    MENU_MAIN = 0,
    MENU_TRIGGER,
    MENU_PULSE,
    MENU_RAMP,
    MENU_SCALE,
    MENU_SAVRECALL,
    MENU_OPMODE,
    MENU_EDIT_VALUE,
    MENU_CONFIRM,
    MENU_REMOTE_LOCK,
} MenuID_t;

typedef struct {
    MenuID_t   current;
    int        cursor;
    int        edit_field;
    char       edit_buf[24];
    int        edit_pos;
    MenuID_t   return_to;
    char       confirm_msg[48];
    bool       confirm_yes;
    int        savrecall_slot;
} MenuState_t;

static MenuState_t M;

#define MRG   8
#define TITBH 36
#define LHGT  (FONT_H + 4)
#define IPAD  6

static void draw_title_bar(const char *title) {
    fb_rect(0, 0, LCD_W, TITBH, PIXEL(0,50,0));
    fb_border(0, 0, LCD_W, TITBH, C_BORDER);
    fb_str_c(0, (TITBH - FONT_H)/2, LCD_W, title, C_HL, PIXEL(0,50,0));
}

static void draw_status_bar(void) {
    int y = LCD_H - FONT_H - 4;
    fb_rect(0, y, LCD_W, FONT_H+4, C_STATBG);
    fb_border(0, y, LCD_W, FONT_H+4, C_BORDER);
    char buf[128];
    snprintf(buf, sizeof(buf),
        "Slot:%u | %s | Pulse:%s | Ramp:%s | %s | %.6fV | %luHz",
        G.active_slot,
        G.cfg.op_mode == OP_PRECISION ? "PREC" : "SLID",
        G.pulse_on    ? "ON" : "OFF",
        G.ramp_running? "RUN" : "IDLE",
        G.remote_mode ? "REMOTE" : "LOCAL",
        (double)G.cfg.ampl_uv / 1e6,
        G.cfg.rate_hz);
    fb_str(MRG, y+2, buf, G.remote_mode ? C_RED : C_DIM, C_STATBG);
}

static void draw_item(int x, int y, int w, int idx, const char *text,
                      uint16_t normal_fg) {
    bool sel = (idx == M.cursor);
    uint16_t bg = sel ? PIXEL(0,80,0) : C_BG;
    uint16_t fg = sel ? C_HL          : normal_fg;
    fb_rect(x, y, w, LHGT, bg);
    if (sel) fb_str(x+2, y+2, ">", C_HL, bg);
    fb_str(x+12, y+2, text, fg, bg);
}

static void draw_remote_lock_overlay(void) {
    int bw = 500, bh = 70;
    int bx = (LCD_W - bw)/2, by = (LCD_H - bh)/2;
    fb_rect(bx, by, bw, bh, PIXEL(60,0,0));
    fb_border(bx, by, bw, bh, C_RED);
    fb_str_c(bx, by+6,  bw, "** REMOTE MODE ACTIVE **",       C_RED,  PIXEL(60,0,0));
    fb_str_c(bx, by+26, bw, "LOCAL INPUT IS DISABLED",        C_WARN, PIXEL(60,0,0));
    fb_str_c(bx, by+46, bw, "Send: set operating mode local", C_DIM,  PIXEL(60,0,0));
}

/* ── Screen renderers (unchanged from F407 version) ── */

static void screen_main(void) {
    fb_fill(C_BG);
    draw_title_bar("[ MAIN MENU ]  Sliding-Precision Pulse Generator");
    static const struct { const char *label; uint16_t col; } items[] = {
        {"1 - Trigger Mode",   C_FG},
        {"2 - Pulse Settings", C_PINK},
        {"3 - Ramp Settings",  C_BLUE},
        {"4 - Scale V/keV",    C_FG},
        {"5 - Save / Recall",  C_CYAN},
        {"6 - Operating Mode", C_WARN},
    };
    int y = TITBH + 30;
    for (int i = 0; i < 6; i++) {
        draw_item(LCD_W/2 - 160, y, 320, i, items[i].label, items[i].col);
        y += LHGT + 6;
    }
    draw_status_bar();
    if (G.remote_mode) draw_remote_lock_overlay();
}

static void screen_trigger(void) {
    fb_fill(C_BG);
    draw_title_bar("[ TRIGGER MODE ]");
    static const char *src_names[] = {
        "Internal","External Source","External Gated","Single Pulse"};
    char buf[64];
    int y = TITBH + 10;
    int x = MRG, w = LCD_W - 2*MRG;

    snprintf(buf, sizeof(buf), "Source : %s", src_names[G.cfg.trig_src]);
    draw_item(x, y, w, 0, buf, C_FG); y += LHGT + 4;

    snprintf(buf, sizeof(buf), "Threshold : %u mV  (100-3500, step 100)",
             G.cfg.trig_threshold_mv);
    draw_item(x, y, w, 1, buf, C_FG); y += LHGT + 4;

    y += 10;
    fb_str(x+12, y, "Ext input : 0-500kHz, +100mV to +10V", C_DIM, C_BG); y+=LHGT;
    fb_str(x+12, y, "Trig out  : +5V unterminated, 200ns width", C_DIM, C_BG); y+=LHGT;
    fb_str(x+12, y, "Fire single: press ENTER when source=Single Pulse", C_DIM, C_BG);

    y = LCD_H - FONT_H*3 - 20 - FONT_H;
    draw_item(x, y, w, 2, "  [Apply to processor]", C_CYAN); y += LHGT+4;
    draw_item(x, y, w, 3, "  [Return to Main Menu]", C_WARN);

    draw_status_bar();
    if (G.remote_mode) draw_remote_lock_overlay();
}

static void screen_pulse(void) {
    fb_fill(C_BG);
    draw_title_bar("[ PULSE SETTINGS ]");
    static const char *modes[]  = {"PRECISION","SLIDING"};
    static const char *pols[]   = {"Positive","Negative"};
    static const char *tops[]   = {"Flat","Tail"};
    char buf[64];
    int x = MRG, w = LCD_W - 2*MRG;
    int y = TITBH + 6;
    int i = 0;
#define PITEM(fmt,...) do { \
    snprintf(buf,sizeof(buf),fmt,##__VA_ARGS__); \
    draw_item(x,y,w,i++,buf,C_FG); y+=LHGT+2; } while(0)

    PITEM("Mode       : %s",          modes[G.cfg.op_mode]);
    PITEM("Amplitude  : %.6f V",      (double)G.cfg.ampl_uv/1e6);
    PITEM("Rate       : %lu Hz",      G.cfg.rate_hz);
    PITEM("Width      : %lu us",      G.cfg.width_ns/1000u);
    PITEM("Rise Time  : %s",          RISE_STR[G.cfg.rise_idx]);
    PITEM("Fall Time  : %s",          FALL_STR[G.cfg.fall_idx]);
    PITEM("Attenuation: %s",          ATTEN_STR[G.cfg.atten_idx]);
    PITEM("Polarity   : %s",          pols[G.cfg.polarity]);
    PITEM("Pulse Top  : %s",          tops[G.cfg.pulse_top]);
    PITEM("Clamp      : %s",          G.cfg.clamp?"ON":"OFF");
    draw_item(x,y,w,i++,"  [Apply to processor]", C_CYAN); y+=LHGT+2;
    draw_item(x,y,w,i++, G.pulse_on?"  [STOP Pulse Output]":
                                    "  [START Pulse Output]",
              G.pulse_on ? C_RED : C_GREEN); y+=LHGT+2;
    draw_item(x,y,w,i,   "  [Return to Main Menu]", C_WARN);
#undef PITEM
    draw_status_bar();
    if (G.remote_mode) draw_remote_lock_overlay();
}

static void screen_ramp(void) {
    fb_fill(C_BG);
    draw_title_bar("[ RAMP / SLIDING SETTINGS ]");
    char buf[64];
    int x = MRG, w = LCD_W - 2*MRG;
    int y = TITBH + 6, i = 0;
#define RITEM(fmt,...) do { \
    snprintf(buf,sizeof(buf),fmt,##__VA_ARGS__); \
    draw_item(x,y,w,i++,buf,C_FG); y+=LHGT+2; } while(0)

    RITEM("Start Voltage : %.3f V",   (double)G.cfg.ramp_start_uv/1e6);
    RITEM("Stop  Voltage : %.3f V",   (double)G.cfg.ramp_stop_uv/1e6);
    RITEM("Ramp Period   : %lu s",    G.cfg.ramp_time_s);
    RITEM("No. Cycles    : %u",       G.cfg.ramp_cycles);
    if (G.ramp_running)
        RITEM("Current Ampl  : %.3f V (cycle %u)",
              (double)G.ramp_ampl_uv/1e6, G.ramp_cycle);
    draw_item(x,y,w,i++,"  [Apply ramp settings]", C_CYAN); y+=LHGT+2;
    draw_item(x,y,w,i++,
              G.ramp_running?"  [STOP Ramp]":"  [EXECUTE Ramp]",
              G.ramp_running ? C_RED : C_GREEN); y+=LHGT+2;
    draw_item(x,y,w,i,   "  [Return to Main Menu]", C_WARN);
#undef RITEM
    int bar_x = MRG, bar_y = LCD_H - FONT_H*2 - 30;
    int bar_w  = LCD_W - 2*MRG, bar_h = 16;
    fb_border(bar_x, bar_y, bar_w, bar_h, C_BORDER);
    if (G.ramp_running && G.cfg.ramp_stop_uv > G.cfg.ramp_start_uv) {
        uint32_t span = G.cfg.ramp_stop_uv - G.cfg.ramp_start_uv;
        uint32_t pos  = G.ramp_ampl_uv > G.cfg.ramp_start_uv
                        ? G.ramp_ampl_uv - G.cfg.ramp_start_uv : 0;
        int fill = (int)((int64_t)(bar_w-2) * pos / span);
        fb_rect(bar_x+1, bar_y+1, fill, bar_h-2, PIXEL(0,100,200));
    }
    draw_status_bar();
    if (G.remote_mode) draw_remote_lock_overlay();
}

static void screen_scale(void) {
    fb_fill(C_BG);
    draw_title_bar("[ SCALE V/keV ]");
    char buf[64];
    int x = MRG, w = LCD_W - 2*MRG;
    int y = TITBH + 6, i = 0;
    snprintf(buf,sizeof(buf),"Display Units   : %s",
             G.cfg.display_kev?"keV":"Volts");
    draw_item(x,y,w,i++,buf,C_FG); y+=LHGT+2;
    snprintf(buf,sizeof(buf),"Full Scale keV  : %lu",  G.cfg.kev_full_scale);
    draw_item(x,y,w,i++,buf,C_FG); y+=LHGT+2;
    y += 10;
    double kev_per_v = (double)G.cfg.kev_full_scale / 10.0;
    fb_printf(x+12,y,C_DIM,C_BG,"Cs-137 662 keV  = %.4f V",662.0/kev_per_v); y+=LHGT;
    fb_printf(x+12,y,C_DIM,C_BG,"Co-60 1173 keV  = %.4f V",1173.0/kev_per_v); y+=LHGT;
    fb_printf(x+12,y,C_DIM,C_BG,"Co-60 1332 keV  = %.4f V",1332.0/kev_per_v); y+=LHGT;
    fb_printf(x+12,y,C_DIM,C_BG,"Current ampl    = %.1f keV",
              (double)G.cfg.ampl_uv/1e6 * kev_per_v);
    y += LHGT + 10;
    draw_item(x,y,w,i++,"  [Apply scale]",        C_CYAN); y+=LHGT+2;
    draw_item(x,y,w,i,   "  [Return to Main Menu]",C_WARN);
    draw_status_bar();
    if (G.remote_mode) draw_remote_lock_overlay();
}

static void screen_savrecall(void) {
    fb_fill(C_BG);
    draw_title_bar("[ SAVE / RECALL — 10 SLOTS ]");
    int cw = (LCD_W - 2*MRG - 8) / 2;
    int ch = FONT_H + 12;
    for (int s = 0; s < (int)NUM_CONFIG_SLOTS; s++) {
        int col = s % 2, row = s / 2;
        int x   = MRG + col * (cw + 8);
        int y   = TITBH + 10 + row * (ch + 4);
        bool sel = (s == M.savrecall_slot);
        bool cur = (s == (int)G.active_slot);
        uint16_t bg = sel ? PIXEL(0,60,0) : (cur ? PIXEL(0,30,0) : PIXEL(10,10,10));
        uint16_t bc = sel ? C_HL : (cur ? C_FG : C_BORDER);
        fb_rect(x, y, cw, ch, bg);
        fb_border(x, y, cw, ch, bc);
        if (NV->slots[s].valid == 0xA5u) {
            char buf[48];
            snprintf(buf,sizeof(buf),"[%d] %-10s %s %.3fV",
                     s, NV->slots[s].label,
                     NV->slots[s].op_mode==OP_PRECISION?"PRE":"SLD",
                     (double)NV->slots[s].ampl_uv/1e6);
            fb_str(x+4, y+3, buf, sel?C_HL:C_FG, bg);
        } else {
            char buf[24]; snprintf(buf,sizeof(buf),"[%d] -- empty --",s);
            fb_str(x+4, y+3, buf, C_DIM, bg);
        }
    }
    int y = TITBH + 10 + 5*(ch+4) + 8;
    int x = MRG, w = LCD_W - 2*MRG, i = 10;
    draw_item(x,y,w,i++,"  [Recall selected slot]",     C_CYAN); y+=LHGT+2;
    draw_item(x,y,w,i++,"  [Save to selected slot]",    C_FG);   y+=LHGT+2;
    draw_item(x,y,w,i++,"  [Clear selected slot]",      C_WARN); y+=LHGT+2;
    draw_item(x,y,w,i++,"  [Recall factory defaults]",  C_WARN); y+=LHGT+2;
    draw_item(x,y,w,i,   "  [Return to Main Menu]",     C_WARN);
    draw_status_bar();
    if (G.remote_mode) draw_remote_lock_overlay();
}

static void screen_opmode(void) {
    fb_fill(C_BG);
    draw_title_bar("[ OPERATING MODE ]");
    int x = MRG, w = LCD_W - 2*MRG;
    int y = TITBH + 10, i = 0;
    char buf[64];
    snprintf(buf,sizeof(buf),"Current mode : %s",
             G.remote_mode ? "REMOTE (local locked)" : "LOCAL");
    draw_item(x,y,w,i++,buf, G.remote_mode?C_RED:C_FG); y+=LHGT+2;
    draw_item(x,y,w,i++,
              G.remote_mode ? "  [Exit REMOTE — switch to LOCAL]"
                            : "  [Enter REMOTE mode]",
              G.remote_mode ? C_RED : C_WARN); y+=LHGT+2;
    y += 16;
    fb_str(x+12,y,"RS-232: 9600 baud, 8N1, no flow control",C_DIM,C_BG); y+=LHGT;
    fb_str(x+12,y,"USB CDC: virtual COM port, auto-baud",C_DIM,C_BG);    y+=LHGT;
    fb_str(x+12,y,"Escape: send  set operating mode local  (or Ctrl-L)",C_DIM,C_BG); y+=LHGT+10;
    if (!G.remote_mode) {
        fb_rect(x+MRG,y,w-MRG*2,LHGT*3+8,PIXEL(20,10,0));
        fb_border(x+MRG,y,w-MRG*2,LHGT*3+8,C_WARN);
        fb_str(x+MRG+6,y+4, "WARNING: Entering REMOTE mode disables ALL",C_WARN,PIXEL(20,10,0));
        fb_str(x+MRG+6,y+4+LHGT,"local keypad and touch input until 'set",C_WARN,PIXEL(20,10,0));
        fb_str(x+MRG+6,y+4+LHGT*2,"operating mode local' is received via serial.",C_WARN,PIXEL(20,10,0));
    }
    y = LCD_H - LHGT*2 - FONT_H - 8;
    draw_item(x,y,w,i,"  [Return to Main Menu]", C_WARN);
    draw_status_bar();
    if (G.remote_mode) draw_remote_lock_overlay();
}

static void screen_edit_value(void) {
    fb_fill(C_BG);
    draw_title_bar("[ EDIT VALUE ]");
    int x = MRG;
    int y = TITBH + 40;
    fb_str(x+12,y,"Enter value (keypad), ENTER to confirm, BACK to cancel:",C_FG,C_BG);
    y += LHGT + 10;
    int bw = 400, bh = 40;
    int bx = (LCD_W - bw)/2;
    fb_rect(bx, y, bw, bh, PIXEL(0,30,0));
    fb_border(bx, y, bw, bh, C_FG);
    fb_str(bx+8, y+10, M.edit_buf, C_HL, PIXEL(0,30,0));
    int cur_x = bx+8 + M.edit_pos*(int)FONT_W;
    fb_rect(cur_x, y+10, 2, FONT_H, C_HL);
}

static void screen_confirm(void) {
    fb_fill(C_BG);
    draw_title_bar("[ CONFIRM ]");
    int bw = 500, bh = 120;
    int bx = (LCD_W-bw)/2, by = LCD_H/2-60;
    fb_rect(bx,by,bw,bh,PIXEL(0,30,0));
    fb_border(bx,by,bw,bh,C_WARN);
    fb_str_c(bx,by+16,bw,M.confirm_msg,C_FG,PIXEL(0,30,0));
    draw_item(bx+20,by+50,200,0,"  [Yes — Confirm]",C_CYAN);
    draw_item(bx+20+220,by+50,200,1,"  [No — Cancel]",C_WARN);
}

static void render_screen(void) {
    switch (M.current) {
        case MENU_MAIN:       screen_main();      break;
        case MENU_TRIGGER:    screen_trigger();   break;
        case MENU_PULSE:      screen_pulse();     break;
        case MENU_RAMP:       screen_ramp();      break;
        case MENU_SCALE:      screen_scale();     break;
        case MENU_SAVRECALL:  screen_savrecall(); break;
        case MENU_OPMODE:     screen_opmode();    break;
        case MENU_EDIT_VALUE: screen_edit_value();break;
        case MENU_CONFIRM:    screen_confirm();   break;
        default: break;
    }
}

/* ============================================================
 *  SECTION 7 — HARDWARE PERIPHERALS
 * ============================================================ */

/* ══════════════════════════════════════════════════════════════
 *  7a — FMC SDRAM  (NEW for F429ZIT6)
 *
 *  The F429 integrates FMC with SDRAM controller on Bank 5/6.
 *  We use Bank 5 (0xC000_0000).  SDCLK = HCLK/2 = 90 MHz
 *  assuming SystemClock = 180 MHz and AHB prescaler = 1.
 *
 *  Pin assignments (all GPIO_AF12_FMC):
 *   Address A0–A11:
 *     PF0=A0  PF1=A1  PF2=A2  PF3=A3  PF4=A4  PF5=A5
 *     PF12=A6 PF13=A7 PF14=A8 PF15=A9 PG0=A10 PG1=A11
 *   Bank address BA0=PG4  BA1=PG5
 *   Data D0–D15:
 *     PD14=D0  PD15=D1  PD0=D2  PD1=D3
 *     PE7=D4   PE8=D5   PE9=D6  PE10=D7
 *     PE11=D8  PE12=D9  PE13=D10 PE14=D11 PE15=D12
 *     PD8=D13  PD9=D14  PD10=D15
 *   Byte enables: PE0=NBL0  PE1=NBL1
 *   Control: PG8=SDCLK  PG15=SDNCAS  PF11=SDNRAS
 *             PG13=SDNE0  PG14=SDNWE  (Bank 1 of SDRAM)
 *
 *  IMPORTANT: PG13 (SDNE0) and PG14 (SDNWE) are used here.
 *  On some boards SDNWE is on PC0 — check your schematic.
 *  The S/H pin has been moved from PC0 → PC1 in this file.
 * ════════════════════════════════════════════════════════════*/

static SDRAM_HandleTypeDef hsdram;
static FMC_SDRAM_TimingTypeDef sdram_timing;

static void sdram_gpio_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF12_FMC;

    /* GPIOD: D0–D3 (PD14,PD15,PD0,PD1) and D13–D15 (PD8–PD10) */
    g.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_8|GPIO_PIN_9|
            GPIO_PIN_10|GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &g);

    /* GPIOE: NBL0=PE0, NBL1=PE1, D4–D12 = PE7–PE15 */
    g.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9|
            GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|
            GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &g);

    /* GPIOF: A0–A5 = PF0–PF5, A6–A9 = PF12–PF15, NRAS=PF11 */
    g.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|
            GPIO_PIN_5|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|
            GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &g);

    /* GPIOG: A10=PG0, A11=PG1, BA0=PG4, BA1=PG5,
              SDCLK=PG8, SDNE0=PG13, SDNWE=PG14, SDNCAS=PG15 */
    g.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_4|GPIO_PIN_5|
            GPIO_PIN_8|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &g);
}

static void sdram_init(void) {
    __HAL_RCC_FMC_CLK_ENABLE();
    sdram_gpio_init();

    /* FMC SDRAM Bank 1 (Bank 5 = 0xC000_0000) */
    hsdram.Instance = FMC_SDRAM_DEVICE;
    hsdram.Init.SDBank             = FMC_SDRAM_BANK1;
    hsdram.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;   /* IS42S: 8 col */
    hsdram.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;     /* 12 row bits  */
    hsdram.Init.MemoryDataWidth    = FMC_SDRAM_MEM_BUS_WIDTH_16;
    hsdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
    hsdram.Init.CASLatency         = IS42S_CAS_LATENCY;
    hsdram.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    hsdram.Init.SDClockPeriod      = FMC_SDRAM_CLOCK_PERIOD_2;      /* SDCLK = HCLK/2 */
    hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
    hsdram.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0;

    /* Timing at 90 MHz SDCLK (1 tick ≈ 11.1 ns) — IS42S16400J */
    sdram_timing.LoadToActiveDelay    = 2;   /* tMRD: 2 clk              */
    sdram_timing.ExitSelfRefreshDelay = 7;   /* tXSR: 70 ns → 7 clk      */
    sdram_timing.SelfRefreshTime      = 4;   /* tRAS min: 42 ns → 4 clk  */
    sdram_timing.RowCycleDelay        = 7;   /* tRC:  63 ns → 7 clk      */
    sdram_timing.WriteRecoveryTime    = 2;   /* tWR:  2 clk               */
    sdram_timing.RPDelay              = 2;   /* tRP:  18 ns → 2 clk      */
    sdram_timing.RCDDelay             = 2;   /* tRCD: 18 ns → 2 clk      */

    HAL_SDRAM_Init(&hsdram, &sdram_timing);

    /* ── SDRAM Initialisation Sequence ──────────────────────── */
    FMC_SDRAM_CommandTypeDef cmd = {0};

    /* 1. Clock enable */
    cmd.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
    cmd.CommandTarget          = FMC_SDRAM_CMD_TARGET_BANK1;
    cmd.AutoRefreshNumber      = 1;
    cmd.ModeRegisterDefinition = 0;
    HAL_SDRAM_SendCommand(&hsdram, &cmd, 0x1000);
    HAL_Delay(1); /* ≥100 µs after CKE high */

    /* 2. PALL — precharge all banks */
    cmd.CommandMode = FMC_SDRAM_CMD_PALL;
    HAL_SDRAM_SendCommand(&hsdram, &cmd, 0x1000);

    /* 3. Auto-refresh × 8 */
    cmd.CommandMode       = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    cmd.AutoRefreshNumber = 8;
    HAL_SDRAM_SendCommand(&hsdram, &cmd, 0x1000);

    /* 4. Load mode register */
    cmd.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
    cmd.AutoRefreshNumber = 1;
    cmd.ModeRegisterDefinition =
        SDRAM_MODEREG_BURST_LENGTH_1       |
        SDRAM_MODEREG_BURST_TYPE_SEQUENTIAL|
        SDRAM_MODEREG_CAS_LATENCY_3        |
        SDRAM_MODEREG_OPERATING_MODE_STANDARD|
        SDRAM_MODEREG_WRITEBURST_MODE_SINGLE;
    HAL_SDRAM_SendCommand(&hsdram, &cmd, 0x1000);

    /* 5. Set refresh rate: (IS42S_REFRESH_COUNT - 20) for safety margin */
    HAL_SDRAM_ProgramRefreshRate(&hsdram, IS42S_REFRESH_COUNT - 20u);
}

/* ══════════════════════════════════════════════════════════════
 *  7b — LTDC  (updated PLLSAI for F429 @ 180 MHz SysClock)
 *
 *  F429ZIT6 PLLSAI:
 *    VCO_in  = HSE/PLLM (same as main PLL; assume HSE=8 MHz, PLLM=8 → 1 MHz)
 *    VCO_out = VCO_in × PLLSAIN
 *    LTDC_clk= VCO_out / PLLSAIR / PLLSAIDIVR
 *
 *  For ~33.3 MHz pixel clock (800×480 @ ~60 Hz):
 *    PLLSAIN=200, PLLSAIR=3, PLLSAIDIVR=2  → 200/3/2 = 33.33 MHz  ← same
 *    as F407; both use the same PLLSAI block.
 *
 *  The values below are identical to the F407 version because the
 *  PLLSAI peripheral is the same IP on both devices.
 * ════════════════════════════════════════════════════════════*/

static LTDC_HandleTypeDef hltdc;

static void ltdc_init(void) {
    __HAL_RCC_LTDC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode      = GPIO_MODE_AF_PP;
    gpio.Pull      = GPIO_NOPULL;
    gpio.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio.Alternate = GPIO_AF14_LTDC;

    /* Red:   R2=PC10 R3=PB0  R4=PA11 R5=PA12 R6=PB1  R7=PG6  */
    gpio.Pin=GPIO_PIN_10; HAL_GPIO_Init(GPIOC,&gpio);
    gpio.Pin=GPIO_PIN_0;  HAL_GPIO_Init(GPIOB,&gpio);
    gpio.Pin=GPIO_PIN_11|GPIO_PIN_12; HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_1;  HAL_GPIO_Init(GPIOB,&gpio);
    gpio.Pin=GPIO_PIN_6;  HAL_GPIO_Init(GPIOG,&gpio);
    /* Green: G2=PA6  G3=PG10 G4=PB10 G5=PB11 G6=PC7  G7=PD3  */
    gpio.Pin=GPIO_PIN_6;  HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_10; HAL_GPIO_Init(GPIOG,&gpio);
    gpio.Pin=GPIO_PIN_10|GPIO_PIN_11; HAL_GPIO_Init(GPIOB,&gpio);
    gpio.Pin=GPIO_PIN_7;  HAL_GPIO_Init(GPIOC,&gpio);
    gpio.Pin=GPIO_PIN_3;  HAL_GPIO_Init(GPIOD,&gpio);
    /* Blue:  B2=PD6  B3=PG11 B4=PG12 B5=PA3  B6=PB8  B7=PB9  */
    gpio.Pin=GPIO_PIN_6;  HAL_GPIO_Init(GPIOD,&gpio);
    gpio.Pin=GPIO_PIN_11; HAL_GPIO_Init(GPIOG,&gpio);
    gpio.Pin=GPIO_PIN_12; HAL_GPIO_Init(GPIOG,&gpio);
    gpio.Pin=GPIO_PIN_3;  HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_8|GPIO_PIN_9; HAL_GPIO_Init(GPIOB,&gpio);
    /* Control: CLK=PG7  HSYNC=PC6  VSYNC=PA4  DE=PF10 */
    gpio.Pin=GPIO_PIN_7;  HAL_GPIO_Init(GPIOG,&gpio);
    gpio.Pin=GPIO_PIN_6;  HAL_GPIO_Init(GPIOC,&gpio);
    gpio.Pin=GPIO_PIN_4;  HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_10; HAL_GPIO_Init(GPIOF,&gpio);

    /*
     * PLLSAI configuration — identical to F407 as both share same IP:
     * PLLSAIN=200, PLLSAIR=3, PLLSAIDIVR=2 → ≈33.3 MHz pixel clock.
     * If your panel needs a different rate adjust PLLSAIN accordingly:
     *   pixel_clk = (HSE/PLLM) * PLLSAIN / PLLSAIR / (2^(PLLSAIDIVR+1))
     *   With HSE=8MHz, PLLM=8: VCO_in=1MHz
     *   PLLSAIN=200: 200/3/2 = 33.33 MHz
     */
    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    clk.PLLSAI.PLLSAIN = 200;
    clk.PLLSAI.PLLSAIR = 3;
    clk.PLLSAIDivR     = RCC_PLLSAIDIVR_2;
    HAL_RCCEx_PeriphCLKConfig(&clk);

    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity        = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity        = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity        = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity        = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync    = 47;
    hltdc.Init.VerticalSync      = 2;
    hltdc.Init.AccumulatedHBP    = 215;
    hltdc.Init.AccumulatedVBP    = 65;
    hltdc.Init.AccumulatedActiveW= 1015;
    hltdc.Init.AccumulatedActiveH= 545;
    hltdc.Init.TotalWidth        = 1055;
    hltdc.Init.TotalHeigh        = 524;   /* HAL typo retained intentionally */
    memset(&hltdc.Init.Backcolor, 0, sizeof(hltdc.Init.Backcolor));
    HAL_LTDC_Init(&hltdc);

    LTDC_LayerCfgTypeDef lyr = {0};
    lyr.WindowX0 = 0;   lyr.WindowX1 = LCD_W;
    lyr.WindowY0 = 0;   lyr.WindowY1 = LCD_H;
    lyr.PixelFormat      = LTDC_PIXEL_FORMAT_RGB565;
    lyr.Alpha            = 255; lyr.Alpha0 = 0;
    lyr.BlendingFactor1  = LTDC_BLENDING_FACTOR1_CA;
    lyr.BlendingFactor2  = LTDC_BLENDING_FACTOR2_CA;
    lyr.FBStartAdress    = FRAMEBUF_BASE;
    lyr.ImageWidth       = LCD_W;
    lyr.ImageHeight      = LCD_H;
    HAL_LTDC_ConfigLayer(&hltdc, &lyr, 0);
}

/* ══════════════════════════════════════════════════════════════
 *  7c — SPI2  (AD5791 DAC + XPT2046 touch)
 *
 *  NOTE: S/H pin moved from PC0 → PC1 because PC0 conflicts
 *  with FMC SDNWE on some F429 pin-out variants.
 *  Adjust back to PC0 if your board uses PG14 for SDNWE instead.
 * ════════════════════════════════════════════════════════════*/

static SPI_HandleTypeDef hspi2;
#define DAC_CS_PORT  GPIOB
#define DAC_CS_PIN   GPIO_PIN_12
#define TCH_CS_PORT  GPIOA
#define TCH_CS_PIN   GPIO_PIN_4
#define TCH_IRQ_PORT GPIOB
#define TCH_IRQ_PIN  GPIO_PIN_0

/* S/H moved PC0 → PC1 to avoid FMC SDNWE conflict */
#define SH_PORT      GPIOC
#define SH_PIN       GPIO_PIN_1   /* ← changed from PIN_0 */

static void spi2_init(void) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_HIGH;
    g.Alternate = GPIO_AF5_SPI2;
    g.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
    HAL_GPIO_Init(GPIOB, &g); /* SCK MISO MOSI */

    g.Mode = GPIO_MODE_OUTPUT_PP; g.Alternate = 0;
    g.Pin = DAC_CS_PIN; HAL_GPIO_Init(DAC_CS_PORT, &g);
    g.Pin = TCH_CS_PIN; HAL_GPIO_Init(TCH_CS_PORT, &g);
    HAL_GPIO_WritePin(DAC_CS_PORT, DAC_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TCH_CS_PORT, TCH_CS_PIN, GPIO_PIN_SET);

    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    g.Pin = TCH_IRQ_PIN; HAL_GPIO_Init(TCH_IRQ_PORT, &g);

    g.Mode = GPIO_MODE_OUTPUT_PP; g.Pull = GPIO_NOPULL;
    g.Pin  = SH_PIN; HAL_GPIO_Init(SH_PORT, &g);
    HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_RESET);

    hspi2.Instance               = SPI2;
    hspi2.Init.Mode              = SPI_MODE_MASTER;
    hspi2.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase          = SPI_PHASE_2EDGE;   /* AD5791 CPOL0 CPHA1 */
    hspi2.Init.NSS               = SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
    hspi2.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi2);
}

/* ── AD5791 18-bit DAC driver (unchanged) ─────────────────── */
static void dac_write_reg(uint8_t addr, uint32_t val) {
    uint8_t tx[3];
    uint32_t word = ((uint32_t)(addr & 0x07u) << 20u) | (val & 0xFFFFFu);
    tx[0] = (uint8_t)(word >> 16u);
    tx[1] = (uint8_t)(word >>  8u);
    tx[2] = (uint8_t)(word);
    HAL_GPIO_WritePin(DAC_CS_PORT, DAC_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, tx, 3, 10);
    HAL_GPIO_WritePin(DAC_CS_PORT, DAC_CS_PIN, GPIO_PIN_SET);
}
#define DAC_REG_DAC    0x01u
#define DAC_REG_CTRL   0x02u
#define DAC_CTRL_RBUF  (1u<<1u)
#define DAC_CTRL_BIN   (1u<<8u)

static void dac_init(void) {
    dac_write_reg(DAC_REG_CTRL, DAC_CTRL_RBUF | DAC_CTRL_BIN);
    dac_write_reg(DAC_REG_DAC, 0u);
}

static void dac_set_uv(uint32_t uv) {
    if (G.cfg.polarity == 1u)
        uv = (uv <= 10000000u) ? (10000000u - uv) : 0u;
    uint32_t code = (uint32_t)((uint64_t)uv * 262143u / 10000000u);
    if (code > 262143u) code = 262143u;
    dac_write_reg(DAC_REG_DAC, code);
}

/* ── USART2 (RS-232) — unchanged ─────────────────────────── */
static UART_HandleTypeDef huart;
static volatile uint8_t  uart_rx_buf[UART_RX_BUF];
static volatile uint16_t uart_rx_head = 0, uart_rx_tail = 0;

static void uart_init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin       = GPIO_PIN_2|GPIO_PIN_3;
    g.Mode      = GPIO_MODE_AF_PP;
    g.Alternate = GPIO_AF7_USART2;
    g.Speed     = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &g);

    huart.Instance          = USART2;
    huart.Init.BaudRate     = UART_BAUD;
    huart.Init.WordLength   = UART_WORDLENGTH_8B;
    huart.Init.StopBits     = UART_STOPBITS_1;
    huart.Init.Parity       = UART_PARITY_NONE;
    huart.Init.Mode         = UART_MODE_TX_RX;
    huart.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart);

    __HAL_UART_ENABLE_IT(&huart, UART_IT_RXNE);
    HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
}

void USART2_IRQHandler(void) {
    if (__HAL_UART_GET_FLAG(&huart, UART_FLAG_RXNE)) {
        uint8_t byte = (uint8_t)(huart.Instance->DR & 0xFFu);
        uint16_t next = (uart_rx_head + 1u) % UART_RX_BUF;
        if (next != uart_rx_tail) {
            uart_rx_buf[uart_rx_head] = byte;
            uart_rx_head = next;
        }
    }
}

static bool uart_getchar(uint8_t *ch) {
    if (uart_rx_head == uart_rx_tail) return false;
    *ch = uart_rx_buf[uart_rx_tail];
    uart_rx_tail = (uart_rx_tail + 1u) % UART_RX_BUF;
    return true;
}

static void uart_puts(const char *s) {
    HAL_UART_Transmit(&huart, (uint8_t *)s, strlen(s), 100);
}

/* ── Keypad (4×4 matrix) — unchanged ───────────────────────
 *  NOTE: PD0–PD7 are shared with FMC data lines D2,D3 and
 *  SDRAM D13–D15.  This is only safe if keypad is scanned
 *  while FMC is idle, or if you remap keypad to another port.
 *  For a production design move keypad to PE or PH.
 * ─────────────────────────────────────────────────────────── */
static const uint8_t KP_MAP[KP_ROWS][KP_COLS] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};

static void keypad_init(void) {
    /* PD0–PD3 and PD4–PD7 are also FMC data lines.
     * Re-init here as GPIO overrides FMC AF; safe only when
     * FMC transactions are not in progress at scan time.
     * For production, remap to PH0–PH7 on F429 LQFP144. */
    __HAL_RCC_GPIOD_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &g);
    g.Pin  = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOD, &g);
    GPIOD->ODR |= 0x0Fu;
}

static char keypad_scan(void) {
    for (int r = 0; r < KP_ROWS; r++) {
        GPIOD->ODR = (GPIOD->ODR | 0x0Fu) & ~(1u << r);
        HAL_Delay(1);
        uint16_t cols = (GPIOD->IDR >> 4u) & 0x0Fu;
        GPIOD->ODR |= 0x0Fu;
        for (int c = 0; c < KP_COLS; c++) {
            if (!(cols & (1u << c))) return KP_MAP[r][c];
        }
    }
    return 0;
}

static char keypad_get_key(void) {
    static char last = 0;
    char k = keypad_scan();
    if (k && k != last) { last = k; return k; }
    if (!k) last = 0;
    return 0;
}

/* ── Spinner encoder (PE0=A, PE1=B, PE2=SW) ─────────────────
 *  PE0/PE1 are also FMC NBL0/NBL1 — same caveat as keypad.
 *  For production remap to PI0–PI2 or PH pins on F429.
 * ─────────────────────────────────────────────────────────── */
static volatile int8_t  spinner_delta = 0;
static volatile bool    spinner_sw    = false;

static void spinner_init(void) {
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin  = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &g);

    g.Mode = GPIO_MODE_IT_RISING_FALLING; g.Pin = GPIO_PIN_0;
    HAL_GPIO_Init(GPIOE, &g);
    HAL_NVIC_SetPriority(EXTI0_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);

    g.Mode = GPIO_MODE_IT_FALLING; g.Pin = GPIO_PIN_2;
    HAL_GPIO_Init(GPIOE, &g);
    HAL_NVIC_SetPriority(EXTI2_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void EXTI0_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_0);
    int a = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_0);
    int b = HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_1);
    spinner_delta += (a != b) ? 1 : -1;
}
void EXTI2_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    static uint32_t last_sw = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_sw > 200u) { spinner_sw = true; last_sw = now; }
}

/* ── XPT2046 Touch — unchanged ─────────────────────────────  */
typedef struct { int16_t x; int16_t y; bool valid; } TouchPt_t;

static uint16_t xpt2046_read(uint8_t cmd) {
    uint8_t tx[3] = {cmd, 0, 0}, rx[3] = {0};
    HAL_GPIO_WritePin(TCH_CS_PORT, TCH_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3, 10);
    HAL_GPIO_WritePin(TCH_CS_PORT, TCH_CS_PIN, GPIO_PIN_SET);
    return ((uint16_t)rx[1] << 5u) | (rx[2] >> 3u);
}

static TouchPt_t touch_read(void) {
    TouchPt_t pt = {0, 0, false};
    if (HAL_GPIO_ReadPin(TCH_IRQ_PORT, TCH_IRQ_PIN)) return pt;
    uint16_t rx = xpt2046_read(0xD0u);
    uint16_t ry = xpt2046_read(0x90u);
    pt.x = (int16_t)((int32_t)(rx - 200) * LCD_W  / 3700);
    pt.y = (int16_t)((int32_t)(ry - 200) * LCD_H  / 3700);
    pt.x = (pt.x < 0) ? 0 : (pt.x >= (int16_t)LCD_W)  ? (int16_t)(LCD_W-1)  : pt.x;
    pt.y = (pt.y < 0) ? 0 : (pt.y >= (int16_t)LCD_H) ? (int16_t)(LCD_H-1) : pt.y;
    pt.valid = true;
    return pt;
}

/* ── Timers (TIM2 / TIM4) — unchanged ──────────────────────  */
static TIM_HandleTypeDef htim2, htim3, htim4;

static void timers_init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Pin = GPIO_PIN_0; g.Mode = GPIO_MODE_AF_PP;
    g.Alternate = GPIO_AF1_TIM2; g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    /*
     * F429ZIT6 APB1 clock:
     *   SysClock=180 MHz, AHB=180 MHz, APB1 prescaler=4 → APB1=45 MHz.
     *   TIM2 clock = 2 × APB1 = 90 MHz (when APB1 prescaler > 1).
     *   Prescaler=89 → 90 MHz / 90 = 1 MHz tick.
     *   If your SystemClock_Config sets APB1 differently, adjust Prescaler
     *   so that (APB1_TIM_CLK / (Prescaler+1)) == 1 MHz.
     */
    htim2.Instance             = TIM2;
    htim2.Init.Prescaler       = 89;     /* ← 90 MHz TIM2 clk → 1 MHz tick */
    htim2.Init.CounterMode     = TIM_COUNTERMODE_UP;
    htim2.Init.Period          = 1000000u / 10000u - 1u;
    htim2.Init.ClockDivision   = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef oc = {0};
    oc.OCMode      = TIM_OCMODE_PWM1;
    oc.Pulse       = 10u;
    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;
    oc.OCFastMode  = TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2, &oc, TIM_CHANNEL_1);

    /*
     * TIM4 ramp tick:
     *   TIM4 is on APB1 bus.  Same 90 MHz TIM4 clock.
     *   Prescaler=8999 → 90 MHz / 9000 = 10 kHz base tick.
     */
    htim4.Instance           = TIM4;
    htim4.Init.Prescaler     = 8999;    /* ← was 8399 for 84 MHz; now 90 MHz */
    htim4.Init.CounterMode   = TIM_COUNTERMODE_UP;
    htim4.Init.Period        = 9999u;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);
    HAL_NVIC_SetPriority(TIM4_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

#define RAMP_STEPS 4096u
static volatile uint32_t ramp_step = 0;

void TIM4_IRQHandler(void) {
    __HAL_TIM_CLEAR_IT(&htim4, TIM_IT_UPDATE);
    if (!G.ramp_running) return;
    ramp_step++;
    if (ramp_step > RAMP_STEPS) {
        ramp_step = 0;
        G.ramp_cycle++;
        if (G.ramp_cycle >= G.cfg.ramp_cycles) {
            G.ramp_running = false;
            HAL_TIM_Base_Stop_IT(&htim4);
            return;
        }
    }
    uint32_t span = G.cfg.ramp_stop_uv > G.cfg.ramp_start_uv
                  ? G.cfg.ramp_stop_uv - G.cfg.ramp_start_uv : 0u;
    G.ramp_ampl_uv = G.cfg.ramp_start_uv +
                     (uint32_t)((uint64_t)span * ramp_step / RAMP_STEPS);
    HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_SET);
    dac_set_uv(G.ramp_ampl_uv);
    for (volatile int d = 0; d < 200; d++);
    HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_RESET);
}

static void output_apply(void) {
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
    if (!G.pulse_on) {
        dac_set_uv(0);
        HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_RESET);
        return;
    }
    uint32_t arr = 1000000u / G.cfg.rate_hz;
    if (arr == 0u) arr = 1u;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr - 1u);
    uint32_t ccr = G.cfg.width_ns / 1000u;
    if (ccr >= arr) ccr = arr - 1u;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);
    if (G.cfg.op_mode == OP_PRECISION) {
        HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_SET);
        dac_set_uv(G.cfg.ampl_uv);
    }
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
}

static void ramp_start(void) {
    G.ramp_running = true;
    G.ramp_cycle   = 0;
    ramp_step      = 0;
    G.ramp_ampl_uv = G.cfg.ramp_start_uv;
    G.cfg.op_mode  = OP_SLIDING;
    G.pulse_on     = true;
    output_apply();
    uint32_t ticks_per_step = (G.cfg.ramp_time_s * 10000u) / RAMP_STEPS;
    if (ticks_per_step == 0u) ticks_per_step = 1u;
    __HAL_TIM_SET_AUTORELOAD(&htim4, ticks_per_step - 1u);
    HAL_TIM_Base_Start_IT(&htim4);
}

static void ramp_stop(void) {
    G.ramp_running = false;
    HAL_TIM_Base_Stop_IT(&htim4);
    G.pulse_on = false;
    output_apply();
}

/* ============================================================
 *  SECTION 8 — NON-VOLATILE CONFIG (Backup SRAM)  — unchanged
 * ============================================================ */
static void nv_init(void) {
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (NV->magic != 0xDEADBEEFu) {
        memset(NV, 0, sizeof(*NV));
        NV->magic = 0xDEADBEEFu;
        PulseConfig_t *s = &NV->slots[0];
        s->valid            = 0xA5u;
        strncpy(s->label, "Factory Default", sizeof(s->label)-1);
        s->op_mode          = OP_PRECISION;
        s->trig_src         = TRIG_INTERNAL;
        s->trig_threshold_mv= 1000u;
        s->ampl_uv          = 5000000u;
        s->width_ns         = 10000u;
        s->rate_hz          = 10000u;
        s->rise_idx         = 0u;
        s->fall_idx         = 4u;
        s->atten_idx        = 0u;
        s->polarity         = 0u;
        s->pulse_top        = 0u;
        s->clamp            = 0u;
        s->ramp_start_uv    = 0u;
        s->ramp_stop_uv     = 10000000u;
        s->ramp_time_s      = 30u;
        s->ramp_cycles      = 999u;
        s->display_kev      = 0u;
        s->kev_full_scale   = 9999u;
    }
}

static void nv_load_slot(uint8_t slot) {
    if (slot >= NUM_CONFIG_SLOTS) return;
    if (NV->slots[slot].valid != 0xA5u) return;
    G.cfg = NV->slots[slot];
    G.active_slot = slot;
}

static void nv_save_slot(uint8_t slot) {
    if (slot >= NUM_CONFIG_SLOTS) return;
    G.cfg.valid = 0xA5u;
    NV->slots[slot] = G.cfg;
    G.active_slot = slot;
}

static void nv_clear_slot(uint8_t slot) {
    if (slot == 0u) return;
    memset(&NV->slots[slot], 0, sizeof(PulseConfig_t));
    if (G.active_slot == slot) G.active_slot = 0u;
}

static void nv_recall_defaults(void) {
    nv_load_slot(0u);
    G.active_slot = 0u;
}

/* ============================================================
 *  SECTION 9 — RS-232 COMMAND PARSER  — unchanged
 * ============================================================ */
static char cmd_buf[CMD_MAX_LEN + 1];
static uint8_t cmd_len = 0;

static void cmd_respond(const char *msg) { uart_puts(msg); uart_puts("\r\n"); }
static void cmd_ok(void)  { cmd_respond("OK"); }
static void cmd_err(void) { cmd_respond("ERROR"); }

static void cmd_dispatch(char *cmd) {
    int len = strlen(cmd);
    while (len > 0 && (cmd[len-1]=='\r'||cmd[len-1]=='\n'||cmd[len-1]==' '))
        cmd[--len] = '\0';

    if (strncmp(cmd,"set trigger mode ",17)==0) {
        const char *m = cmd+17;
        if      (!strcmp(m,"internal"))  G.cfg.trig_src=TRIG_INTERNAL;
        else if (!strcmp(m,"external"))  G.cfg.trig_src=TRIG_EXTERNAL;
        else if (!strcmp(m,"gated"))     G.cfg.trig_src=TRIG_GATED;
        else if (!strcmp(m,"one pulse")) G.cfg.trig_src=TRIG_ONE_PULSE;
        else { cmd_err(); return; }
        cmd_ok();
    }
    else if (strncmp(cmd,"set threshold ",14)==0) {
        float v = strtof(cmd+14, NULL);
        G.cfg.trig_threshold_mv = (uint16_t)(v * 1000.0f);
        if (G.cfg.trig_threshold_mv < 100u)  G.cfg.trig_threshold_mv = 100u;
        if (G.cfg.trig_threshold_mv > 3500u) G.cfg.trig_threshold_mv = 3500u;
        cmd_ok();
    }
    else if (strncmp(cmd,"set amplitude ",14)==0) {
        float v = strtof(cmd+14, NULL);
        if (v < 0.0f) v = 0.0f;
        if (v > 10.0f) v = 10.0f;
        G.cfg.ampl_uv = (uint32_t)(v * 1e6f);
        if (G.pulse_on && G.cfg.op_mode==OP_PRECISION) dac_set_uv(G.cfg.ampl_uv);
        cmd_ok();
    }
    else if (strncmp(cmd,"set rep rate ",13)==0) {
        uint32_t r = (uint32_t)atoi(cmd+13);
        if (r < 1u) r=1u; if (r > 100000u) r=100000u;
        G.cfg.rate_hz = r;
        if (G.pulse_on) output_apply();
        cmd_ok();
    }
    else if (strncmp(cmd,"set width ",10)==0) {
        uint32_t w = (uint32_t)atoi(cmd+10);
        if (w < 1000u) w=1000u; if (w > 1000000u) w=1000000u;
        G.cfg.width_ns = w;
        if (G.pulse_on) output_apply();
        cmd_ok();
    }
    else if (strncmp(cmd,"set rise time ",14)==0) {
        uint8_t idx = (uint8_t)atoi(cmd+14);
        if (idx >= N_RISE) idx = N_RISE-1u;
        G.cfg.rise_idx = idx; cmd_ok();
    }
    else if (strncmp(cmd,"set fall time ",14)==0) {
        uint8_t idx = (uint8_t)atoi(cmd+14);
        if (idx >= N_FALL) idx = N_FALL-1u;
        G.cfg.fall_idx = idx; cmd_ok();
    }
    else if (strncmp(cmd,"set attenuation ",16)==0) {
        uint8_t idx = (uint8_t)atoi(cmd+16);
        if (idx >= N_ATTEN) idx = N_ATTEN-1u;
        G.cfg.atten_idx = idx; cmd_ok();
    }
    else if (strncmp(cmd,"set power on ",13)==0) {
        G.pulse_on = (cmd[13]=='1'); output_apply(); cmd_ok();
    }
    else if (strncmp(cmd,"set polarity positive ",22)==0) {
        G.cfg.polarity = (cmd[22]=='0') ? 1u : 0u; cmd_ok();
    }
    else if (strncmp(cmd,"set tail pulse ",15)==0) {
        G.cfg.pulse_top = (cmd[15]=='1') ? 1u : 0u; cmd_ok();
    }
    else if (strncmp(cmd,"clamp baseline ",15)==0) {
        G.cfg.clamp = (cmd[15]=='1') ? 1u : 0u; cmd_ok();
    }
    else if (strncmp(cmd,"set ramp startv ",16)==0) {
        G.cfg.ramp_start_uv = (uint32_t)(strtof(cmd+16,NULL)*1e6f); cmd_ok();
    }
    else if (strncmp(cmd,"set ramp stopv ",15)==0) {
        G.cfg.ramp_stop_uv = (uint32_t)(strtof(cmd+15,NULL)*1e6f); cmd_ok();
    }
    else if (strncmp(cmd,"set ramp startev ",17)==0) {
        float kev = strtof(cmd+17,NULL);
        G.cfg.ramp_start_uv = (uint32_t)(kev / G.cfg.kev_full_scale * 10e6f); cmd_ok();
    }
    else if (strncmp(cmd,"set ramp stopev ",16)==0) {
        float kev = strtof(cmd+16,NULL);
        G.cfg.ramp_stop_uv = (uint32_t)(kev / G.cfg.kev_full_scale * 10e6f); cmd_ok();
    }
    else if (strncmp(cmd,"set ramp time ",14)==0) {
        uint32_t t = (uint32_t)atoi(cmd+14);
        if (t < 30u) t=30u; if (t > 900u) t=900u;
        G.cfg.ramp_time_s = t; cmd_ok();
    }
    else if (strncmp(cmd,"set ramp cycles ",16)==0) {
        uint16_t c = (uint16_t)atoi(cmd+16);
        if (c < 1u) c=1u; if (c > 9999u) c=9999u;
        G.cfg.ramp_cycles = c; cmd_ok();
    }
    else if (!strcmp(cmd,"execute ramp") || !strcmp(cmd,"exrcute ramp")) {
        if (!G.ramp_running) ramp_start(); cmd_ok();
    }
    else if (!strcmp(cmd,"stop ramp")) {
        if (G.ramp_running) ramp_stop(); cmd_ok();
    }
    else if (!strcmp(cmd,"trigger one pulse")) {
        G.pulse_on = true; output_apply();
        HAL_Delay(1);
        G.pulse_on = false; output_apply(); cmd_ok();
    }
    else if (strncmp(cmd,"set display kev ",16)==0) {
        G.cfg.display_kev = (cmd[16]=='1') ? 1u : 0u; cmd_ok();
    }
    else if (strncmp(cmd,"set equivalent kev ",19)==0) {
        G.cfg.kev_full_scale = (uint32_t)atoi(cmd+19); cmd_ok();
    }
    else if (strncmp(cmd,"save config ",12)==0) {
        nv_save_slot((uint8_t)atoi(cmd+12)); cmd_ok();
    }
    else if (strncmp(cmd,"recall config ",14)==0) {
        nv_load_slot((uint8_t)atoi(cmd+14)); output_apply(); cmd_ok();
    }
    else if (!strcmp(cmd,"recall factory defaults")) {
        nv_recall_defaults(); output_apply(); cmd_ok();
    }
    else if (!strcmp(cmd,"set operating mode local")) {
        G.remote_mode = false;
        cmd_respond("OK - LOCAL mode active, local input restored");
        render_screen();
    }
    else if (!strcmp(cmd,"set operating mode remote")) {
        G.remote_mode = true;
        cmd_respond("OK - REMOTE mode active, local input locked");
        render_screen();
    }
    else if (!strcmp(cmd,"help")) {
        cmd_respond("=== Pulse Generator RS-232 Commands ===");
        cmd_respond("set trigger mode [internal|external|gated|one pulse]");
        cmd_respond("set threshold <volts>           (0.1 - 3.5)");
        cmd_respond("trigger one pulse");
        cmd_respond("set power on [1|0]");
        cmd_respond("set amplitude <volts>           (0.0 - 10.0)");
        cmd_respond("set rep rate <hz>               (1 - 100000)");
        cmd_respond("set width <ns>                  (1000 - 1000000)");
        cmd_respond("set rise time <index>           (0-7)");
        cmd_respond("set fall time <index>           (0-10)");
        cmd_respond("set attenuation <index>         (0-9)");
        cmd_respond("set polarity positive [1|0]     (1=pos,0=neg)");
        cmd_respond("set tail pulse [1|0]");
        cmd_respond("clamp baseline [1|0]");
        cmd_respond("set ramp startv <volts>");
        cmd_respond("set ramp stopv  <volts>");
        cmd_respond("set ramp startev <kev>");
        cmd_respond("set ramp stopev  <kev>");
        cmd_respond("set ramp time <seconds>         (30-900)");
        cmd_respond("set ramp cycles <n>             (1-9999)");
        cmd_respond("execute ramp / stop ramp");
        cmd_respond("set display kev [1|0]");
        cmd_respond("set equivalent kev <value>");
        cmd_respond("save config <slot>              (0-9)");
        cmd_respond("recall config <slot>            (0-9)");
        cmd_respond("recall factory defaults");
        cmd_respond("set operating mode [local|remote]");
        cmd_respond("MCU: STM32F429ZIT6  FW: F429-port");
        cmd_respond("help");
    }
    else {
        cmd_err();
    }
}

static void uart_process_byte(uint8_t byte) {
    if (byte == REMOTE_ESCAPE_CHAR) {
        G.remote_mode = false;
        cmd_respond("OK - LOCAL mode restored via Ctrl-L");
        render_screen();
        return;
    }
    if (byte == '\r' || byte == '\n') {
        if (cmd_len > 0u) {
            cmd_buf[cmd_len] = '\0';
            for (int i = 0; i < (int)cmd_len; i++)
                cmd_buf[i] = (char)tolower((unsigned char)cmd_buf[i]);
            cmd_dispatch(cmd_buf);
            cmd_len = 0u;
        }
    } else if (byte == '\b' || byte == 127u) {
        if (cmd_len > 0u) cmd_len--;
    } else if (cmd_len < CMD_MAX_LEN) {
        cmd_buf[cmd_len++] = (char)byte;
    }
}

/* ============================================================
 *  SECTION 10 — INPUT PROCESSOR  — unchanged
 * ============================================================ */
static int touch_to_item(int y, int first_item_y, int item_h) {
    if (y < first_item_y) return -1;
    return (y - first_item_y) / item_h;
}

static bool input_process(void) {
    uint8_t byte;
    bool dirty = false;
    while (uart_getchar(&byte)) { uart_process_byte(byte); dirty = true; }

    if (G.remote_mode) {
        spinner_delta = 0; spinner_sw = false;
        keypad_get_key();
        return dirty;
    }

    int8_t delta = 0;
    __disable_irq();
    delta = spinner_delta; spinner_delta = 0;
    __enable_irq();
    bool sw = spinner_sw; spinner_sw = false;

    if (delta != 0) {
        if (M.current != MENU_EDIT_VALUE) {
            M.cursor += (delta > 0) ? 1 : -1;
            if (M.cursor < 0) M.cursor = 0;
        }
        dirty = true;
    }
    if (sw) goto handle_enter;

    {
        char k = keypad_get_key();
        if (k) {
            dirty = true;
            if (k >= '0' && k <= '9') {
                if (M.current == MENU_EDIT_VALUE && M.edit_pos < 20) {
                    M.edit_buf[M.edit_pos++] = k;
                    M.edit_buf[M.edit_pos]   = '\0';
                }
            } else if (k == '*') {
                if (M.current == MENU_EDIT_VALUE && M.edit_pos < 20) {
                    M.edit_buf[M.edit_pos++] = '.';
                    M.edit_buf[M.edit_pos]   = '\0';
                }
            } else if (k == 'A') {
                M.cursor = (M.cursor > 0) ? M.cursor - 1 : M.cursor;
            } else if (k == 'B') {
                M.cursor++;
            } else if (k == 'D') {
                if (M.current == MENU_EDIT_VALUE) {
                    if (M.edit_pos > 0) { M.edit_buf[--M.edit_pos] = '\0'; }
                    else { M.current = M.return_to; M.cursor = M.edit_field; }
                } else { M.current = MENU_MAIN; M.cursor = 0; }
            } else if (k == 'C') {
                handle_enter:;
                switch (M.current) {

                case MENU_MAIN:
                    M.cursor = (M.cursor > 5) ? 5 : M.cursor;
                    M.current = (MenuID_t)(MENU_TRIGGER + M.cursor);
                    M.cursor = 0;
                    break;

                case MENU_TRIGGER:
                    switch (M.cursor) {
                    case 0: G.cfg.trig_src = (G.cfg.trig_src + 1u) % 4u; break;
                    case 1:
                        M.return_to = MENU_TRIGGER; M.edit_field = 1;
                        M.current = MENU_EDIT_VALUE; M.edit_pos = 0;
                        snprintf(M.edit_buf,sizeof(M.edit_buf),"%u",G.cfg.trig_threshold_mv);
                        break;
                    case 2: cmd_respond("TRIG APPLIED"); break;
                    case 3: M.current = MENU_MAIN; M.cursor = 0; break;
                    }
                    break;

                case MENU_EDIT_VALUE: {
                    float v = strtof(M.edit_buf, NULL);
                    if (M.return_to == MENU_TRIGGER) {
                        G.cfg.trig_threshold_mv = (uint16_t)v;
                        if (G.cfg.trig_threshold_mv < 100u)  G.cfg.trig_threshold_mv = 100u;
                        if (G.cfg.trig_threshold_mv > 3500u) G.cfg.trig_threshold_mv = 3500u;
                    } else if (M.return_to == MENU_PULSE) {
                        switch (M.edit_field) {
                        case 1: G.cfg.ampl_uv  = (uint32_t)(v*1e6f); break;
                        case 2: G.cfg.rate_hz  = (uint32_t)v; break;
                        case 3: G.cfg.width_ns = (uint32_t)(v*1000.0f); break;
                        }
                        if (G.pulse_on) output_apply();
                    } else if (M.return_to == MENU_RAMP) {
                        switch (M.edit_field) {
                        case 0: G.cfg.ramp_start_uv = (uint32_t)(v*1e6f); break;
                        case 1: G.cfg.ramp_stop_uv  = (uint32_t)(v*1e6f); break;
                        case 2: G.cfg.ramp_time_s   = (uint32_t)v; break;
                        case 3: G.cfg.ramp_cycles   = (uint16_t)v; break;
                        }
                    } else if (M.return_to == MENU_SCALE) {
                        if (M.edit_field == 1) G.cfg.kev_full_scale = (uint32_t)v;
                    }
                    M.current = M.return_to; M.cursor = M.edit_field;
                    break;
                }

                case MENU_PULSE:
                    switch (M.cursor) {
                    case 0: G.cfg.op_mode=(G.cfg.op_mode==OP_PRECISION)?OP_SLIDING:OP_PRECISION; break;
                    case 1:
                        M.return_to=MENU_PULSE; M.edit_field=1;
                        M.current=MENU_EDIT_VALUE; M.edit_pos=0;
                        snprintf(M.edit_buf,sizeof(M.edit_buf),"%.6f",(double)G.cfg.ampl_uv/1e6);
                        break;
                    case 2:
                        M.return_to=MENU_PULSE; M.edit_field=2;
                        M.current=MENU_EDIT_VALUE; M.edit_pos=0;
                        snprintf(M.edit_buf,sizeof(M.edit_buf),"%lu",G.cfg.rate_hz);
                        break;
                    case 3:
                        M.return_to=MENU_PULSE; M.edit_field=3;
                        M.current=MENU_EDIT_VALUE; M.edit_pos=0;
                        snprintf(M.edit_buf,sizeof(M.edit_buf),"%lu",G.cfg.width_ns/1000u);
                        break;
                    case 4: G.cfg.rise_idx=(G.cfg.rise_idx+1u)%N_RISE; break;
                    case 5: G.cfg.fall_idx=(G.cfg.fall_idx+1u)%N_FALL; break;
                    case 6: G.cfg.atten_idx=(G.cfg.atten_idx+1u)%N_ATTEN; break;
                    case 7: G.cfg.polarity ^= 1u; break;
                    case 8: G.cfg.pulse_top ^= 1u; break;
                    case 9: G.cfg.clamp ^= 1u; break;
                    case 10: output_apply(); cmd_respond("PULSE SETTINGS APPLIED"); break;
                    case 11: G.pulse_on=!G.pulse_on; output_apply(); break;
                    case 12: M.current=MENU_MAIN; M.cursor=1; break;
                    }
                    break;

                case MENU_RAMP:
                    switch (M.cursor) {
                    case 0: M.return_to=MENU_RAMP;M.edit_field=0;M.current=MENU_EDIT_VALUE;
                            M.edit_pos=0;snprintf(M.edit_buf,sizeof(M.edit_buf),"%.3f",
                            (double)G.cfg.ramp_start_uv/1e6);break;
                    case 1: M.return_to=MENU_RAMP;M.edit_field=1;M.current=MENU_EDIT_VALUE;
                            M.edit_pos=0;snprintf(M.edit_buf,sizeof(M.edit_buf),"%.3f",
                            (double)G.cfg.ramp_stop_uv/1e6);break;
                    case 2: M.return_to=MENU_RAMP;M.edit_field=2;M.current=MENU_EDIT_VALUE;
                            M.edit_pos=0;snprintf(M.edit_buf,sizeof(M.edit_buf),"%lu",
                            G.cfg.ramp_time_s);break;
                    case 3: M.return_to=MENU_RAMP;M.edit_field=3;M.current=MENU_EDIT_VALUE;
                            M.edit_pos=0;snprintf(M.edit_buf,sizeof(M.edit_buf),"%u",
                            G.cfg.ramp_cycles);break;
                    case 4: cmd_respond("RAMP SETTINGS APPLIED"); break;
                    case 5: if(G.ramp_running)ramp_stop();else ramp_start(); break;
                    case 6: M.current=MENU_MAIN;M.cursor=2; break;
                    }
                    break;

                case MENU_SCALE:
                    switch (M.cursor) {
                    case 0: G.cfg.display_kev ^= 1u; break;
                    case 1: M.return_to=MENU_SCALE;M.edit_field=1;M.current=MENU_EDIT_VALUE;
                            M.edit_pos=0;snprintf(M.edit_buf,sizeof(M.edit_buf),"%lu",
                            G.cfg.kev_full_scale);break;
                    case 2: cmd_respond("SCALE APPLIED"); break;
                    case 3: M.current=MENU_MAIN;M.cursor=3; break;
                    }
                    break;

                case MENU_SAVRECALL:
                    if (M.cursor < 10) {
                        M.savrecall_slot = M.cursor;
                    } else switch (M.cursor) {
                    case 10: nv_load_slot((uint8_t)M.savrecall_slot); output_apply();
                             cmd_respond("RECALLED"); break;
                    case 11: nv_save_slot((uint8_t)M.savrecall_slot);
                             cmd_respond("SAVED"); break;
                    case 12: nv_clear_slot((uint8_t)M.savrecall_slot);
                             cmd_respond("CLEARED"); break;
                    case 13: nv_recall_defaults(); output_apply();
                             cmd_respond("DEFAULTS RECALLED"); break;
                    case 14: M.current=MENU_MAIN;M.cursor=4; break;
                    }
                    break;

                case MENU_OPMODE:
                    switch (M.cursor) {
                    case 0: break;
                    case 1:
                        if (G.remote_mode) {
                            G.remote_mode = false;
                            cmd_respond("OK - LOCAL mode restored via front panel");
                        } else {
                            G.remote_mode = true;
                            cmd_respond("OK - REMOTE mode active, local locked");
                        }
                        break;
                    case 2: M.current=MENU_MAIN;M.cursor=5; break;
                    }
                    break;

                case MENU_CONFIRM:
                    M.confirm_yes = (M.cursor == 0);
                    M.current = M.return_to;
                    break;

                default: break;
                }
            }
        }
    }

    {
        static bool was_pressed = false;
        TouchPt_t pt = touch_read();
        if (pt.valid && !was_pressed) {
            was_pressed = true; dirty = true;
            int first_y, item_h;
            switch (M.current) {
            case MENU_MAIN:
                first_y = TITBH + 30; item_h = LHGT + 6;
                M.cursor = touch_to_item(pt.y, first_y, item_h);
                if (M.cursor >= 0 && M.cursor <= 5) {
                    M.current = (MenuID_t)(MENU_TRIGGER + M.cursor);
                    M.cursor = 0;
                }
                break;
            case MENU_SAVRECALL: {
                int ch2 = FONT_H + 12;
                int cw  = (LCD_W - 2*MRG - 8) / 2;
                int row = (pt.y - (TITBH+10)) / (ch2+4);
                int col = (pt.x - MRG) / (cw+8);
                int slot = row*2 + col;
                if (slot >= 0 && slot < (int)NUM_CONFIG_SLOTS) {
                    M.savrecall_slot = slot; M.cursor = slot;
                } else {
                    first_y = TITBH + 10 + 5*(ch2+4) + 8;
                    M.cursor = 10 + touch_to_item(pt.y, first_y, LHGT+2);
                }
                break;
            }
            default:
                first_y = TITBH + 6; item_h = LHGT + 2;
                M.cursor = touch_to_item(pt.y, first_y, item_h);
                break;
            }
        } else if (!pt.valid) {
            was_pressed = false;
        }
    }

    return dirty;
}

/* ============================================================
 *  SECTION 11 — TEMPERATURE COMPENSATION  (unchanged)
 * ============================================================ */
static void temp_comp_update(void) { G.temp_comp_due = false; }

static uint32_t last_temp_tick = 0u;
static void temp_comp_tick(void) {
    if (HAL_GetTick() - last_temp_tick >= 300000u) {
        last_temp_tick = HAL_GetTick();
        G.temp_comp_due = true;
    }
}

/* ============================================================
 *  SECTION 12 — MAIN ENTRY POINT
 * ============================================================ */
void pulser_run(void) {
    /* ── Hardware init ───────────────────────────────────────
     * ORDER MATTERS:
     *   1. SDRAM first — frame buffer lives there
     *   2. LTDC next — starts reading from SDRAM
     *   3. Rest in any order
     * ─────────────────────────────────────────────────────── */
    sdram_init();   /* ← NEW: F429 FMC SDRAM must be up before LTDC */
    ltdc_init();
    spi2_init();
    dac_init();
    uart_init();
    keypad_init();
    spinner_init();
    timers_init();
    nv_init();

    nv_load_slot(0u);
    G.pulse_on     = false;
    G.ramp_running = false;
    G.remote_mode  = false;
    G.active_slot  = 0u;

    memset(&M, 0, sizeof(M));
    M.current        = MENU_MAIN;
    M.cursor         = 0;
    M.savrecall_slot = 0;

    /* Splash screen */
    fb_fill(C_BG);
    draw_title_bar("Sliding-Precision Pulse Generator — STM32F429ZIT6");
    fb_str_c(0, LCD_H/2 - FONT_H,    LCD_W, "Initialising hardware...",         C_FG,  C_BG);
    fb_str_c(0, LCD_H/2,             LCD_W, "DAC: AD5791 18-bit  |  FMC SDRAM", C_DIM, C_BG);
    fb_str_c(0, LCD_H/2 + FONT_H,    LCD_W, "UART: 9600 baud  |  10 config slots", C_DIM, C_BG);
    fb_str_c(0, LCD_H/2 + FONT_H*2,  LCD_W, "Always boots in LOCAL mode",        C_WARN,C_BG);
    HAL_Delay(2000u);

    render_screen();

    uint32_t last_render = 0u;
    for (;;) {
        bool dirty = input_process();
        temp_comp_tick();
        if (G.temp_comp_due) temp_comp_update();
        uint32_t now = HAL_GetTick();
        if (dirty || (now - last_render >= 50u)) {
            render_screen();
            last_render = now;
        }
        HAL_Delay(5u);
    }
}
