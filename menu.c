/**
 * @file    pulser_main.c
 * @brief   Sliding-Precision Pulse Generator — Complete Firmware
 *          MCU  : STM32F429ZIT6  (LQFP144, Cortex-M4, 180 MHz)
 *          LCD  : LCD-OlinuXino-7TS (800x480, RGB565, LTDC parallel RGB)
 *          Touch: XPT2046 (SPI touch controller on OlinuXino-7TS)
 *          DAC  : AD5791 18-bit SPI DAC (amplitude)
 *          Timer: TIM2 (pulse rep-rate), TIM3 (pulse width), TIM4 (ramp tick)
 *          UART : USART2 (RS-232, PA2/PA3)
 *          NVRAM: Backup SRAM (4KB, VBAT-retained, 10 config slots)
 *          SDRAM: External 16-bit SDRAM via FMC Bank 1 (0xC0000000,
 *                 frame buffer).  Sized for IS42S16400J or equivalent.
 *
 * ── Architecture ────────────────────────────────────────────────────────────
 *  The firmware runs a cooperative state machine in the main loop:
 *    1. Input layer  – reads keypad matrix, spinner encoder, touch panel,
 *                      and UART RX ring buffer.
 *    2. Menu engine  – translates inputs into menu navigation and parameter
 *                      edits.  Each menu screen has an enter/exit callback.
 *    3. Command layer– RS-232 commands (identical to BNC PB-5 set + extras)
 *                      are parsed and dispatched to the same handlers as the
 *                      local UI.
 *    4. Output layer – applies live PulseConfig_t to TIM2/TIM3, AD5791 DAC,
 *                      and sample-and-hold switch.
 *
 * ── Remote Lock ─────────────────────────────────────────────────────────────
 *  When REMOTE mode is active:
 *    • All keypad and touch events are swallowed (ignored) in input_process().
 *    • The spinner encoder ISR sets a flag but input_process() discards it.
 *    • The LCD shows a "REMOTE ACTIVE — LOCAL LOCKED" banner over the menu.
 *    • The only way to return to LOCAL is:
 *        RS-232 command: "set operating mode local"   (or Ctrl-L = 0x0C)
 *    • Power-cycle also exits remote, but the mode is NOT saved to NVRAM on
 *      remote-exit (it always boots LOCAL per specification).
 *
 * ── Call from main.c ────────────────────────────────────────────────────────
 *    extern void pulser_run(void);
 *    // After HAL_Init(), SystemClock_Config(), MX_GPIO_Init(), etc.:
 *    pulser_run();   // never returns
 *
 * ── Pin map (STM32F429ZIT6) ─────────────────────────────────────────────────
 *  LTDC RGB    : see ltdc_init() — note PB0/PB1/PG10/PG12 use AF9, rest AF14
 *  Touch CS    : PA8    Touch CLK: PB13  Touch MISO: PB14  Touch MOSI: PB15
 *  Touch IRQ   : PB5    (SPI2 shared bus)
 *  AD5791 CS   : PB12   AD5791 CLK: PB13  MISO: PB14  MOSI: PB15 (SPI2)
 *  S/H HOLD    : PC0    (high = sample, low = hold)
 *  PULSE OUT   : TIM2 CH1 (PA0) via output stage
 *  USART2 TX   : PA2    USART2 RX : PA3
 *  Keypad rows : PC8, PC9, PC11, PC12   cols: PF6–PF9
 *  Spinner A   : PB3 (EXTI3)  Spinner B: PB4   Spinner SW: PE2 (EXTI2)
 *  FMC SDRAM   : Bank1 (0xC0000000) — D0–D15, A0–A11, SDCLK/SDNWE/…
 *                see sdram_fmc_init() for full GPIO list
 * ──────────────────────────────────────────────────────────────────────────*/

#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <ctype.h>

/* GPIO_AF9_LTDC: On STM32F429 certain LTDC pins (PB0, PB1, PG10, PG12) are
 * wired to alternate function 9, not 14.  The STM32F4 HAL names AF9 after
 * TIM14 (also on AF9), but the numeric value 9 is what the AFR register
 * needs.  We define a clear alias so the LTDC code documents its intent. */
#ifndef GPIO_AF9_LTDC
#define GPIO_AF9_LTDC  GPIO_AF9_TIM14
#endif

/* ============================================================
 *  SECTION 1 — COMPILE-TIME CONFIGURATION
 * ============================================================ */
#define LCD_W               800u
#define LCD_H               480u
#define FRAMEBUF_BASE       0xC0000000u   /* External SDRAM */
#define BKPSRAM_BASE        0x40024000u   /* Backup SRAM base */

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
#define C_STATBG    PIXEL(  0,  25,   0)

static volatile uint16_t * const FB = (uint16_t *)FRAMEBUF_BASE;

/* ============================================================
 *  SECTION 3 — DATA STRUCTURES
 * ============================================================ */

/* Rise-time option table (index → nanoseconds) */
static const uint32_t RISE_NS[] = {50,100,200,500,1000,2000,5000,10000};
static const char    *RISE_STR[]= {"50ns","100ns","200ns","500ns",
                                    "1us","2us","5us","10us"};
#define N_RISE 8u

/* Fall-time option table (index → nanoseconds) */
static const uint32_t FALL_NS[] = {500,1000,2000,5000,10000,20000,
                                    50000,100000,200000,500000,1000000};
static const char    *FALL_STR[]= {"500ns","1us","2us","5us","10us","20us",
                                    "50us","100us","200us","500us","1ms"};
#define N_FALL 11u

/* Attenuation table */
static const uint16_t ATTEN_VAL[]= {1,2,5,10,20,50,100,200,500,1000};
static const char    *ATTEN_STR[]= {"1x","2x","5x","10x","20x","50x",
                                     "100x","200x","500x","1000x"};
#define N_ATTEN 10u

/* Trigger source enum */
typedef enum {
    TRIG_INTERNAL = 0,
    TRIG_EXTERNAL,
    TRIG_GATED,
    TRIG_ONE_PULSE
} TrigSrc_t;

/* Operating mode */
typedef enum {
    OP_PRECISION = 0,
    OP_SLIDING
} OpMode_t;

/* Full pulse configuration (one saveable slot) */
typedef struct __attribute__((packed)) {
    uint8_t   valid;            /* 0xA5 = valid, else empty          */
    char      label[16];        /* user label, null-terminated        */
    uint8_t   op_mode;          /* OpMode_t                           */
    uint8_t   trig_src;         /* TrigSrc_t                          */
    uint16_t  trig_threshold_mv;/* 100–3500 mV in 100 mV steps        */
    uint32_t  ampl_uv;          /* amplitude µV  (0–10 000 000)       */
    uint32_t  width_ns;         /* pulse width ns (1000–1 000 000)    */
    uint32_t  rate_hz;          /* rep rate Hz (1–100 000)            */
    uint8_t   rise_idx;         /* index into RISE_NS[]               */
    uint8_t   fall_idx;         /* index into FALL_NS[]               */
    uint8_t   atten_idx;        /* index into ATTEN_VAL[]             */
    uint8_t   polarity;         /* 0=positive, 1=negative             */
    uint8_t   pulse_top;        /* 0=flat, 1=tail                     */
    uint8_t   clamp;            /* 0=off, 1=on                        */
    uint32_t  ramp_start_uv;    /* sliding start µV                   */
    uint32_t  ramp_stop_uv;     /* sliding stop  µV                   */
    uint32_t  ramp_time_s;      /* ramp period seconds (30–900)       */
    uint16_t  ramp_cycles;      /* 1–9999                             */
    uint8_t   display_kev;      /* 0=volts, 1=keV                     */
    uint32_t  kev_full_scale;   /* keV value at 10V                   */
    uint8_t   _pad[3];          /* alignment padding                  */
} PulseConfig_t;                /* total ≤ 64 bytes per slot          */

/* Backup SRAM layout */
typedef struct __attribute__((packed)) {
    uint32_t      magic;                    /* 0xDEADBEEF = valid     */
    PulseConfig_t slots[NUM_CONFIG_SLOTS];
} NVData_t;

/* Live state (RAM copy of active config + runtime flags) */
typedef struct {
    PulseConfig_t cfg;          /* current live parameters            */
    uint8_t       active_slot;  /* which NV slot was last loaded      */
    bool          pulse_on;     /* is output running?                 */
    bool          ramp_running; /* is ramp sweep active?              */
    bool          remote_mode;  /* REMOTE LOCK active?                */
    uint32_t      ramp_ampl_uv; /* current ramp amplitude (updated by TIM4 ISR) */
    uint16_t      ramp_cycle;   /* cycles completed                   */
    bool          temp_comp_due;/* flag set every 5 min               */
} LiveState_t;

static LiveState_t  G;          /* global live state                  */
static NVData_t    *NV = (NVData_t *)BKPSRAM_BASE;

/* ============================================================
 *  SECTION 4 — 8×16 BITMAP FONT  (ASCII 0x20–0x7E)
 * ============================================================ */
#define FONT_FIRST  0x20u
#define FONT_W      8u
#define FONT_H      16u

/* Standard 8×16 VGA ROM font – rows top→bottom, bit7 = leftmost pixel */
static const uint8_t FONT[96][16] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ' ' */
    {0x00,0x30,0x78,0x78,0x78,0x30,0x30,0x00,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00}, /* '!' */
    {0x00,0x6C,0x6C,0x6C,0x28,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '"' */
    {0x00,0x6C,0x6C,0xFE,0x6C,0x6C,0xFE,0x6C,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '#' */
    {0x18,0x18,0x7C,0xC6,0xC2,0xC0,0x7C,0x06,0x86,0xC6,0x7C,0x18,0x18,0x00,0x00,0x00}, /* '$' */
    {0x00,0x00,0xC2,0xC6,0x0C,0x18,0x30,0x60,0xC6,0x86,0x00,0x00,0x00,0x00,0x00,0x00}, /* '%' */
    {0x00,0x38,0x6C,0x6C,0x38,0x76,0xDC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00,0x00,0x00}, /* '&' */
    {0x00,0x30,0x30,0x30,0x60,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ''' */
    {0x00,0x0C,0x18,0x30,0x60,0x60,0x60,0x60,0x30,0x18,0x0C,0x00,0x00,0x00,0x00,0x00}, /* '(' */
    {0x00,0x60,0x30,0x18,0x0C,0x0C,0x0C,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00,0x00}, /* ')' */
    {0x00,0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '*' */
    {0x00,0x00,0x18,0x18,0xFF,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '+' */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00}, /* ',' */
    {0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '-' */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00}, /* '.' */
    {0x00,0x03,0x06,0x0C,0x18,0x30,0x60,0xC0,0x80,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '/' */
    {0x00,0x7C,0xC6,0xCE,0xD6,0xD6,0xE6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00}, /* '0' */
    {0x00,0x18,0x38,0x78,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00,0x00,0x00}, /* '1' */
    {0x00,0x7C,0xC6,0x06,0x0C,0x18,0x30,0x60,0xC6,0xFE,0x00,0x00,0x00,0x00,0x00,0x00}, /* '2' */
    {0x00,0x7C,0xC6,0x06,0x3C,0x06,0x06,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '3' */
    {0x00,0x0E,0x1E,0x36,0x66,0xC6,0xFF,0x06,0x06,0x0F,0x00,0x00,0x00,0x00,0x00,0x00}, /* '4' */
    {0x00,0xFE,0xC0,0xC0,0xFC,0x06,0x06,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '5' */
    {0x00,0x38,0x60,0xC0,0xFC,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '6' */
    {0x00,0xFE,0xC6,0x06,0x0C,0x18,0x30,0x30,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '7' */
    {0x00,0x7C,0xC6,0xC6,0x7C,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '8' */
    {0x00,0x7C,0xC6,0xC6,0x7E,0x06,0x06,0x0C,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '9' */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ':' */
    {0x00,0x18,0x18,0x00,0x00,0x18,0x18,0x30,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ';' */
    {0x00,0x06,0x0C,0x18,0x30,0x60,0x30,0x18,0x0C,0x06,0x00,0x00,0x00,0x00,0x00,0x00}, /* '<' */
    {0x00,0x00,0xFF,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '=' */
    {0x00,0x60,0x30,0x18,0x0C,0x06,0x0C,0x18,0x30,0x60,0x00,0x00,0x00,0x00,0x00,0x00}, /* '>' */
    {0x00,0x7C,0xC6,0x0C,0x18,0x18,0x00,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '?' */
    {0x00,0x7C,0xC6,0xDE,0xDE,0xDE,0xDC,0xC0,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '@' */
    {0x00,0x10,0x38,0x6C,0xC6,0xFE,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'A' */
    {0x00,0xFC,0x66,0x66,0x7C,0x66,0x66,0x66,0xFC,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'B' */
    {0x00,0x3C,0x66,0xC2,0xC0,0xC0,0xC2,0x66,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'C' */
    {0x00,0xF8,0x6C,0x66,0x66,0x66,0x66,0x6C,0xF8,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'D' */
    {0x00,0xFE,0x62,0x60,0x7C,0x60,0x62,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'E' */
    {0x00,0xFE,0x66,0x62,0x78,0x60,0x60,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'F' */
    {0x00,0x3C,0x66,0xC2,0xC0,0xDE,0xC6,0x66,0x3A,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'G' */
    {0x00,0xC6,0xC6,0xC6,0xFE,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'H' */
    {0x00,0x3C,0x18,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'I' */
    {0x00,0x1E,0x0C,0x0C,0x0C,0xCC,0xCC,0xCC,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'J' */
    {0x00,0xE6,0x66,0x6C,0x78,0x78,0x6C,0x66,0xE6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'K' */
    {0x00,0xF0,0x60,0x60,0x60,0x62,0x66,0x66,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'L' */
    {0x00,0xC6,0xEE,0xFE,0xD6,0xC6,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'M' */
    {0x00,0xC6,0xE6,0xF6,0xDE,0xCE,0xC6,0xC6,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'N' */
    {0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'O' */
    {0x00,0xFC,0x66,0x66,0x7C,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'P' */
    {0x00,0x7C,0xC6,0xC6,0xC6,0xD6,0xDE,0x7C,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'Q' */
    {0x00,0xFC,0x66,0x66,0x7C,0x6C,0x66,0x66,0xE6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'R' */
    {0x00,0x7C,0xC6,0x60,0x38,0x0C,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'S' */
    {0x00,0xFF,0xDB,0x18,0x18,0x18,0x18,0x18,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'T' */
    {0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'U' */
    {0x00,0xC6,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'V' */
    {0x00,0xC6,0xC6,0xC6,0xD6,0xD6,0xFE,0x6C,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'W' */
    {0x00,0xC6,0x6C,0x38,0x38,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'X' */
    {0x00,0xCC,0xCC,0xCC,0x78,0x30,0x30,0x30,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'Y' */
    {0x00,0xFE,0xC6,0x8C,0x18,0x30,0x60,0xC6,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'Z' */
    {0x00,0x3C,0x30,0x30,0x30,0x30,0x30,0x30,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '[' */
    {0x00,0x80,0xC0,0x60,0x30,0x18,0x0C,0x06,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '\' */
    {0x00,0x3C,0x0C,0x0C,0x0C,0x0C,0x0C,0x0C,0x3C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* ']' */
    {0x10,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '^' */
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xFF,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '_' */
    {0x30,0x30,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '`' */
    {0x00,0x00,0x78,0x0C,0x7C,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'a' */
    {0x00,0xE0,0x60,0x7C,0x66,0x66,0x66,0x66,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'b' */
    {0x00,0x00,0x7C,0xC6,0xC0,0xC0,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'c' */
    {0x00,0x1C,0x0C,0x7C,0xCC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'd' */
    {0x00,0x00,0x7C,0xC6,0xFE,0xC0,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'e' */
    {0x00,0x1C,0x36,0x30,0xFC,0x30,0x30,0x30,0x78,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'f' */
    {0x00,0x00,0x76,0xCC,0xCC,0xCC,0x7C,0x0C,0xCC,0x78,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'g' */
    {0x00,0xE0,0x60,0x6C,0x76,0x66,0x66,0x66,0xE6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'h' */
    {0x00,0x18,0x00,0x78,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'i' */
    {0x00,0x06,0x00,0x1E,0x06,0x06,0x06,0x06,0x66,0x3C,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'j' */
    {0x00,0xE0,0x60,0x66,0x6C,0x78,0x6C,0x66,0xE6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'k' */
    {0x00,0x78,0x18,0x18,0x18,0x18,0x18,0x18,0x7E,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'l' */
    {0x00,0x00,0xEC,0xFE,0xD6,0xD6,0xD6,0xD6,0xD6,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'm' */
    {0x00,0x00,0xDC,0x66,0x66,0x66,0x66,0x66,0x66,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'n' */
    {0x00,0x00,0x7C,0xC6,0xC6,0xC6,0xC6,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'o' */
    {0x00,0x00,0xDC,0x66,0x66,0x66,0x7C,0x60,0x60,0xF0,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'p' */
    {0x00,0x00,0x76,0xCC,0xCC,0xCC,0x7C,0x0C,0x0C,0x1E,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'q' */
    {0x00,0x00,0xDC,0x76,0x62,0x60,0x60,0x60,0xF0,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'r' */
    {0x00,0x00,0x7C,0xC6,0x60,0x38,0x0C,0xC6,0x7C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 's' */
    {0x00,0x10,0x30,0xFC,0x30,0x30,0x30,0x36,0x1C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 't' */
    {0x00,0x00,0xCC,0xCC,0xCC,0xCC,0xCC,0xCC,0x76,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'u' */
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0x6C,0x38,0x10,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'v' */
    {0x00,0x00,0xC6,0xC6,0xD6,0xD6,0xFE,0xEE,0x6C,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'w' */
    {0x00,0x00,0xC6,0x6C,0x38,0x38,0x6C,0xC6,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'x' */
    {0x00,0x00,0xC6,0xC6,0xC6,0xC6,0x7E,0x06,0x0C,0xF8,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'y' */
    {0x00,0x00,0xFE,0xCC,0x18,0x30,0x60,0xC6,0xFE,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* 'z' */
    {0x00,0x0E,0x18,0x18,0x70,0x18,0x18,0x0E,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '{' */
    {0x00,0x18,0x18,0x18,0x00,0x18,0x18,0x18,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '|' */
    {0x00,0x70,0x18,0x18,0x0E,0x18,0x18,0x70,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '}' */
    {0x76,0xDC,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, /* '~' */
};

/* ============================================================
 *  SECTION 5 — FRAME BUFFER PRIMITIVES
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
/* Print formatted string; returns width used */
static int fb_printf(int px, int py, uint16_t fg, uint16_t bg,
                     const char *fmt, ...) {
    char buf[64];
    va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
    return fb_str(px, py, buf, fg, bg) - px;
}

/* ============================================================
 *  SECTION 6 — MENU SYSTEM  (state machine)
 * ============================================================ */

/* Menu IDs */
typedef enum {
    MENU_MAIN = 0,
    MENU_TRIGGER,
    MENU_PULSE,
    MENU_RAMP,
    MENU_SCALE,
    MENU_SAVRECALL,
    MENU_OPMODE,
    MENU_EDIT_VALUE,   /* generic numeric editor */
    MENU_CONFIRM,      /* yes/no dialog          */
    MENU_REMOTE_LOCK,  /* overlay shown when remote */
} MenuID_t;

typedef struct {
    MenuID_t   current;
    int        cursor;         /* highlighted item index        */
    int        edit_field;     /* which field is being edited   */
    char       edit_buf[24];   /* numeric entry buffer          */
    int        edit_pos;       /* cursor in edit_buf            */
    MenuID_t   return_to;      /* after confirm/edit dialog     */
    char       confirm_msg[48];
    bool       confirm_yes;
    int        savrecall_slot; /* selected slot in save/recall  */
} MenuState_t;

static MenuState_t M;

/* Layout constants */
#define MRG   8
#define TITBH 36
#define LHGT  (FONT_H + 4)   /* line height */
#define IPAD  6               /* inner padding */

/* ──────────────────────────────────────────────────────────────
 *  DRAW HELPERS
 * ────────────────────────────────────────────────────────────── */
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

/* Draw a menu item line; highlighted if selected */
static void draw_item(int x, int y, int w, int idx, const char *text,
                      uint16_t normal_fg) {
    bool sel = (idx == M.cursor);
    uint16_t bg = sel ? PIXEL(0,80,0) : C_BG;
    uint16_t fg = sel ? C_HL          : normal_fg;
    fb_rect(x, y, w, LHGT, bg);
    if (sel) {
        fb_str(x+2, y+2, ">", C_HL, bg);
    }
    fb_str(x+12, y+2, text, fg, bg);
}

/* Remote lock overlay */
static void draw_remote_lock_overlay(void) {
    /* semi-opaque red banner in the centre */
    int bw = 500, bh = 70;
    int bx = (LCD_W - bw)/2, by = (LCD_H - bh)/2;
    fb_rect(bx, by, bw, bh, PIXEL(60,0,0));
    fb_border(bx, by, bw, bh, C_RED);
    fb_str_c(bx, by+6,   bw, "** REMOTE MODE ACTIVE **",       C_RED,  PIXEL(60,0,0));
    fb_str_c(bx, by+26,  bw, "LOCAL INPUT IS DISABLED",        C_WARN, PIXEL(60,0,0));
    fb_str_c(bx, by+46,  bw, "Send: set operating mode local", C_DIM,  PIXEL(60,0,0));
}

/* ──────────────────────────────────────────────────────────────
 *  SCREEN RENDERERS
 * ────────────────────────────────────────────────────────────── */

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

    /* Source selector */
    snprintf(buf, sizeof(buf), "Source : %s", src_names[G.cfg.trig_src]);
    draw_item(x, y, w, 0, buf, C_FG); y += LHGT + 4;

    /* Threshold */
    snprintf(buf, sizeof(buf), "Threshold : %u mV  (100-3500, step 100)",
             G.cfg.trig_threshold_mv);
    draw_item(x, y, w, 1, buf, C_FG); y += LHGT + 4;

    /* Info */
    y += 10;
    fb_str(x+12, y, "Ext input : 0-500kHz, +100mV to +10V", C_DIM, C_BG); y+=LHGT;
    fb_str(x+12, y, "Trig out  : +5V unterminated, 200ns width", C_DIM, C_BG); y+=LHGT;
    fb_str(x+12, y, "Fire single: press ENTER when source=Single Pulse", C_DIM, C_BG);

    /* Action items */
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
    /* Ramp visualisation bar */
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
    /* Computed equivalences */
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
    /* Remote lock explanation box */
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

/* Generic numeric value editor */
static void screen_edit_value(void) {
    fb_fill(C_BG);
    draw_title_bar("[ EDIT VALUE ]");
    int x = MRG;
    int y = TITBH + 40;
    fb_str(x+12,y,"Enter value (keypad), ENTER to confirm, BACK to cancel:",C_FG,C_BG);
    y += LHGT + 10;
    /* Draw edit box */
    int bw = 400, bh = 40;
    int bx = (LCD_W - bw)/2;
    fb_rect(bx, y, bw, bh, PIXEL(0,30,0));
    fb_border(bx, y, bw, bh, C_FG);
    fb_str(bx+8, y+10, M.edit_buf, C_HL, PIXEL(0,30,0));
    /* cursor blink approximation (just always show it) */
    int cur_x = bx+8 + M.edit_pos*(int)FONT_W;
    fb_rect(cur_x, y+10, 2, FONT_H, C_HL);
}

/* Confirm dialog */
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

/* Master screen dispatcher */
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

/* ── FMC SDRAM (frame buffer memory at 0xC0000000) ─────────
 *
 *  Targets a 16-bit, 4-bank SDRAM with 12-bit row / 8-bit column
 *  addressing (e.g. IS42S16400J-7 or equivalent 64 Mbit device).
 *  FMC Bank 1  →  base address 0xC0000000.
 *
 *  GPIO assignments (all AF12 = FMC):
 *   Address  A0–A5   : PF0–PF5      A6–A9  : PF12–PF15
 *            A10–A11 : PG0–PG1
 *   Data     D0–D1   : PD14–PD15    D2–D3  : PD0–PD1
 *            D4–D12  : PE7–PE15     D13–D15: PD8–PD10
 *   Byte en. NBL0    : PE0          NBL1   : PE1
 *   Control  SDCLK   : PG8          SDNCAS : PG15
 *            SDNRAS  : PF11         SDNWE  : PH5
 *            SDCKE0  : PH2          SDNE0  : PH3
 *
 *  NOTE: Adjust timing constants if your SDRAM chip differs.
 *        Assumes HCLK = 180 MHz → SDCLK = HCLK/2 = 90 MHz.
 * ─────────────────────────────────────────────────────────── */
static SDRAM_HandleTypeDef hsdram;

static void sdram_fmc_init(void) {
    __HAL_RCC_FMC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE(); __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE(); __HAL_RCC_GPIOG_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode      = GPIO_MODE_AF_PP;
    g.Pull      = GPIO_NOPULL;
    g.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
    g.Alternate = GPIO_AF12_FMC;

    /* GPIOD: D0=PD14, D1=PD15, D2=PD0, D3=PD1, D13=PD8, D14=PD9, D15=PD10 */
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10
          | GPIO_PIN_14 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOD, &g);

    /* GPIOE: NBL0=PE0, NBL1=PE1, D4–D12=PE7–PE15 */
    g.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_7  | GPIO_PIN_8  | GPIO_PIN_9
          | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
          | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOE, &g);

    /* GPIOF: A0–A5=PF0–PF5, SDNRAS=PF11, A6–A9=PF12–PF15 */
    g.Pin = GPIO_PIN_0  | GPIO_PIN_1  | GPIO_PIN_2  | GPIO_PIN_3  | GPIO_PIN_4
          | GPIO_PIN_5  | GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14
          | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOF, &g);

    /* GPIOG: A10=PG0, A11=PG1, SDCLK=PG8, SDNCAS=PG15 */
    g.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_8 | GPIO_PIN_15;
    HAL_GPIO_Init(GPIOG, &g);

    /* GPIOH: SDCKE0=PH2, SDNE0=PH3, SDNWE=PH5
     *  Using PH5 for SDNWE avoids the PC0 conflict with the S/H switch. */
    g.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_5;
    HAL_GPIO_Init(GPIOH, &g);

    /* ── FMC SDRAM controller ── */
    FMC_SDRAM_TimingTypeDef timing = {0};

    hsdram.Instance                = FMC_Bank5_6_R;
    hsdram.Init.SDBank             = FMC_SDRAM_BANK1;
    hsdram.Init.ColumnBitsNumber   = FMC_SDRAM_COLUMN_BITS_NUM_8;
    hsdram.Init.RowBitsNumber      = FMC_SDRAM_ROW_BITS_NUM_12;
    hsdram.Init.MemoryDataWidth    = FMC_SDRAM_MEM_BUS_WIDTH_16;
    hsdram.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
    hsdram.Init.CASLatency         = FMC_SDRAM_CAS_LATENCY_3;
    hsdram.Init.WriteProtection    = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
    hsdram.Init.SDClockPeriod      = FMC_SDRAM_CLOCK_PERIOD_2; /* 90 MHz */
    hsdram.Init.ReadBurst          = FMC_SDRAM_RBURST_ENABLE;
    hsdram.Init.ReadPipeDelay      = FMC_SDRAM_RPIPE_DELAY_0;

    /* Timing for IS42S16400J-7 @ 90 MHz SDCLK (11.1 ns/cycle) */
    timing.LoadToActiveDelay    = 2;  /* TMRD  ≥ 2 clk           */
    timing.ExitSelfRefreshDelay = 7;  /* TXSR  ≥ 70 ns → 7 clk   */
    timing.SelfRefreshTime      = 4;  /* TRAS  ≥ 42 ns → 4 clk   */
    timing.RowCycleDelay        = 7;  /* TRC   ≥ 63 ns → 7 clk   */
    timing.WriteRecoveryTime    = 2;  /* TWR   ≥ 1clk+7ns → 2 clk*/
    timing.RPDelay              = 2;  /* TRP   ≥ 18 ns → 2 clk   */
    timing.RCDDelay             = 2;  /* TRCD  ≥ 18 ns → 2 clk   */

    HAL_SDRAM_Init(&hsdram, &timing);

    /* ── SDRAM initialisation sequence ── */
    FMC_SDRAM_CommandTypeDef cmd = {0};
    cmd.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;

    /* 1. Enable clock */
    cmd.CommandMode            = FMC_SDRAM_CMD_CLK_ENABLE;
    cmd.AutoRefreshNumber      = 1;
    cmd.ModeRegisterDefinition = 0;
    HAL_SDRAM_SendCommand(&hsdram, &cmd, 0xFFFF);
    HAL_Delay(1); /* wait ≥ 100 µs */

    /* 2. Precharge all banks */
    cmd.CommandMode = FMC_SDRAM_CMD_PALL;
    HAL_SDRAM_SendCommand(&hsdram, &cmd, 0xFFFF);

    /* 3. Eight auto-refresh cycles */
    cmd.CommandMode       = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
    cmd.AutoRefreshNumber = 8;
    HAL_SDRAM_SendCommand(&hsdram, &cmd, 0xFFFF);

    /* 4. Load mode register: CAS=3, burst=1, sequential, single-write */
    cmd.CommandMode            = FMC_SDRAM_CMD_LOAD_MODE;
    cmd.AutoRefreshNumber      = 1;
    cmd.ModeRegisterDefinition = 0x0230; /* CAS=3, BL=1, write-burst=single */
    HAL_SDRAM_SendCommand(&hsdram, &cmd, 0xFFFF);

    /* 5. Refresh rate: 64 ms / 4096 rows = 15.625 µs/row
     *    At 90 MHz: 15.625e-6 × 90e6 ≈ 1406 cycles → subtract 20 for margin */
    HAL_SDRAM_ProgramRefreshRate(&hsdram, 1386);
}

/* ── LTDC ─────────────────────────────────────────────────── */
static LTDC_HandleTypeDef hltdc;

static void ltdc_init(void) {
    __HAL_RCC_LTDC_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE(); __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE(); __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE(); __HAL_RCC_GPIOF_CLK_ENABLE();
    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Mode = GPIO_MODE_AF_PP; gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH; gpio.Alternate = GPIO_AF14_LTDC;
    /* Red   R2=PC10  R4=PA11  R5=PA12  R7=PG6  (all AF14)             */
    gpio.Pin=GPIO_PIN_10; HAL_GPIO_Init(GPIOC,&gpio);
    gpio.Pin=GPIO_PIN_11|GPIO_PIN_12; HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_6;  HAL_GPIO_Init(GPIOG,&gpio);
    /* Red   R3=PB0  R6=PB1  — AF9 on STM32F429 (not AF14)             */
    gpio.Alternate=GPIO_AF9_LTDC; /* AF9 on STM32F429 */
    gpio.Pin=GPIO_PIN_0;  HAL_GPIO_Init(GPIOB,&gpio);  /* R3 */
    gpio.Pin=GPIO_PIN_1;  HAL_GPIO_Init(GPIOB,&gpio);  /* R6 */
    gpio.Alternate=GPIO_AF14_LTDC;
    /* Green G2=PA6  G4=PB10  G5=PB11  G6=PC7  G7=PD3  (all AF14)     */
    gpio.Pin=GPIO_PIN_6;  HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_10|GPIO_PIN_11; HAL_GPIO_Init(GPIOB,&gpio);
    gpio.Pin=GPIO_PIN_7;  HAL_GPIO_Init(GPIOC,&gpio);
    gpio.Pin=GPIO_PIN_3;  HAL_GPIO_Init(GPIOD,&gpio);
    /* Green G3=PG10 — AF9 on STM32F429                                 */
    gpio.Alternate=GPIO_AF9_LTDC; /* AF9 on STM32F429 */
    gpio.Pin=GPIO_PIN_10; HAL_GPIO_Init(GPIOG,&gpio);  /* G3 */
    gpio.Alternate=GPIO_AF14_LTDC;
    /* Blue  B2=PD6  B3=PG11  B5=PA3  B6=PB8  B7=PB9  (all AF14)      */
    gpio.Pin=GPIO_PIN_6;  HAL_GPIO_Init(GPIOD,&gpio);
    gpio.Pin=GPIO_PIN_11; HAL_GPIO_Init(GPIOG,&gpio);
    gpio.Pin=GPIO_PIN_3;  HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_8|GPIO_PIN_9; HAL_GPIO_Init(GPIOB,&gpio);
    /* Blue  B4=PG12 — AF9 on STM32F429                                 */
    gpio.Alternate=GPIO_AF9_LTDC; /* AF9 on STM32F429 */
    gpio.Pin=GPIO_PIN_12; HAL_GPIO_Init(GPIOG,&gpio);  /* B4 */
    gpio.Alternate=GPIO_AF14_LTDC;
    /* Control CLK=PG7 HSYNC=PC6 VSYNC=PA4 DE=PF10 */
    gpio.Pin=GPIO_PIN_7;  HAL_GPIO_Init(GPIOG,&gpio);
    gpio.Pin=GPIO_PIN_6;  HAL_GPIO_Init(GPIOC,&gpio);
    gpio.Pin=GPIO_PIN_4;  HAL_GPIO_Init(GPIOA,&gpio);
    gpio.Pin=GPIO_PIN_10; HAL_GPIO_Init(GPIOF,&gpio);

    /* PLLSAI → 33.3 MHz pixel clock: PLLSAIN=200, PLLSAIR=3, DIV=2 */
    RCC_PeriphCLKInitTypeDef clk = {0};
    clk.PeriphClockSelection = RCC_PERIPHCLK_LTDC;
    clk.PLLSAI.PLLSAIN = 200; clk.PLLSAI.PLLSAIR = 3;
    clk.PLLSAIDivR = RCC_PLLSAIDIVR_2;
    HAL_RCCEx_PeriphCLKConfig(&clk);

    hltdc.Instance = LTDC;
    hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
    hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
    hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
    hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
    hltdc.Init.HorizontalSync = 47; hltdc.Init.VerticalSync = 2;
    hltdc.Init.AccumulatedHBP = 215; hltdc.Init.AccumulatedVBP = 65;
    hltdc.Init.AccumulatedActiveW = 1015; hltdc.Init.AccumulatedActiveH = 545;
    hltdc.Init.TotalWidth = 1055; hltdc.Init.TotalHeigh = 567; /* VFP=22 lines; 'TotalHeigh' is ST HAL's field name */
    memset(&hltdc.Init.Backcolor, 0, sizeof(hltdc.Init.Backcolor));
    HAL_LTDC_Init(&hltdc);

    LTDC_LayerCfgTypeDef lyr = {0};
    lyr.WindowX0=0; lyr.WindowX1=LCD_W; lyr.WindowY0=0; lyr.WindowY1=LCD_H;
    lyr.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
    lyr.Alpha=255; lyr.Alpha0=0;
    lyr.BlendingFactor1=LTDC_BLENDING_FACTOR1_CA;
    lyr.BlendingFactor2=LTDC_BLENDING_FACTOR2_CA;
    lyr.FBStartAdress = FRAMEBUF_BASE;
    lyr.ImageWidth=LCD_W; lyr.ImageHeight=LCD_H;
    HAL_LTDC_ConfigLayer(&hltdc, &lyr, 0);
}

/* ── SPI2 (AD5791 DAC + XPT2046 touch — both on same bus, diff CS) ── */
static SPI_HandleTypeDef hspi2;
#define DAC_CS_PORT  GPIOB
#define DAC_CS_PIN   GPIO_PIN_12
/* TCH_CS moved from PA4 to PA8: PA4 = LTDC_VSYNC (AF14) on STM32F429 */
#define TCH_CS_PORT  GPIOA
#define TCH_CS_PIN   GPIO_PIN_8
/* TCH_IRQ moved from PB0 to PB5: PB0 = LTDC_R3 (AF9) on STM32F429 */
#define TCH_IRQ_PORT GPIOB
#define TCH_IRQ_PIN  GPIO_PIN_5
#define SH_PORT      GPIOC
#define SH_PIN       GPIO_PIN_0

static void spi2_init(void) {
    __HAL_RCC_SPI2_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE(); __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};
    g.Mode=GPIO_MODE_AF_PP; g.Pull=GPIO_NOPULL; g.Speed=GPIO_SPEED_FREQ_HIGH;
    g.Alternate=GPIO_AF5_SPI2;
    g.Pin=GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15; HAL_GPIO_Init(GPIOB,&g); /* CLK MISO MOSI */

    /* CS pins as output */
    g.Mode=GPIO_MODE_OUTPUT_PP; g.Alternate=0;
    g.Pin=DAC_CS_PIN; HAL_GPIO_Init(DAC_CS_PORT,&g);
    g.Pin=TCH_CS_PIN; HAL_GPIO_Init(TCH_CS_PORT,&g);
    HAL_GPIO_WritePin(DAC_CS_PORT, DAC_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(TCH_CS_PORT, TCH_CS_PIN, GPIO_PIN_SET);

    /* Touch IRQ as input */
    g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_PULLUP;
    g.Pin=TCH_IRQ_PIN; HAL_GPIO_Init(TCH_IRQ_PORT,&g);

    /* S/H control */
    g.Mode=GPIO_MODE_OUTPUT_PP; g.Pull=GPIO_NOPULL;
    g.Pin=SH_PIN; HAL_GPIO_Init(SH_PORT,&g);
    HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_RESET); /* hold by default */

    hspi2.Instance=SPI2;
    hspi2.Init.Mode=SPI_MODE_MASTER;
    hspi2.Init.Direction=SPI_DIRECTION_2LINES;
    hspi2.Init.DataSize=SPI_DATASIZE_8BIT;
    hspi2.Init.CLKPolarity=SPI_POLARITY_LOW;
    hspi2.Init.CLKPhase=SPI_PHASE_2EDGE; /* AD5791 = CPOL0 CPHA1 */
    hspi2.Init.NSS=SPI_NSS_SOFT;
    hspi2.Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_16; /* ~5MHz */
    hspi2.Init.FirstBit=SPI_FIRSTBIT_MSB;
    HAL_SPI_Init(&hspi2);
}

/* ── AD5791 18-bit DAC driver ─────────────────────────────── */
/* AD5791 register write: 24-bit SPI frame */
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
#define DAC_REG_CLR    0x03u
#define DAC_CTRL_RBUF  (1u<<1u)
#define DAC_CTRL_OPGND (1u<<2u)
#define DAC_CTRL_DACTRI (1u<<3u)
#define DAC_CTRL_BIN   (1u<<8u) /* offset binary mode */

static void dac_init(void) {
    /* Control: RBUF=1, offset binary, SDO enabled, output connected */
    dac_write_reg(DAC_REG_CTRL, DAC_CTRL_RBUF | DAC_CTRL_BIN);
    dac_write_reg(DAC_REG_DAC, 0u); /* zero output */
}

/* Set DAC output for amplitude in µV (0–10 000 000 µV = 0–10 V) */
static void dac_set_uv(uint32_t uv) {
    /* 18-bit full scale = 262143 codes for 10V span */
    /* Apply polarity: if negative, invert */
    if (G.cfg.polarity == 1u) {
        uv = (uv <= 10000000u) ? (10000000u - uv) : 0u;
    }
    /* Apply attenuation factor (we attenuate digitally before DAC for precision) */
    /* Note: hardware attenuator also engaged for big ratios; DAC is pre-divided here */
    uint32_t code = (uint32_t)((uint64_t)uv * 262143u / 10000000u);
    if (code > 262143u) code = 262143u;
    dac_write_reg(DAC_REG_DAC, code);
}

/* ── USART2 (RS-232) ─────────────────────────────────────── */
static UART_HandleTypeDef huart;
static volatile uint8_t uart_rx_buf[UART_RX_BUF];
static volatile uint16_t uart_rx_head = 0, uart_rx_tail = 0;

static void uart_init(void) {
    __HAL_RCC_USART2_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    g.Pin=GPIO_PIN_2|GPIO_PIN_3; g.Mode=GPIO_MODE_AF_PP;
    g.Alternate=GPIO_AF7_USART2; g.Speed=GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA,&g);

    huart.Instance=USART2;
    huart.Init.BaudRate=UART_BAUD; huart.Init.WordLength=UART_WORDLENGTH_8B;
    huart.Init.StopBits=UART_STOPBITS_1; huart.Init.Parity=UART_PARITY_NONE;
    huart.Init.Mode=UART_MODE_TX_RX; huart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
    HAL_UART_Init(&huart);

    /* Enable RX interrupt */
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

/* ── Keypad (4×4 matrix) ─────────────────────────────────── */
/* Rows: PC8, PC9, PC11, PC12 (output, drive low to scan)
   Cols: PF6–PF9  (input, pull-up)
   Rationale: original PD0–PD3 conflict with FMC_D2/D3 and LTDC_G7;
              original PD4–PD7 conflict with LTDC_B2 (PD6).          */
static const uint8_t KP_MAP[KP_ROWS][KP_COLS] = {
    {'1','2','3','A'},
    {'4','5','6','B'},
    {'7','8','9','C'},
    {'*','0','#','D'}
};
/* 'A'=UP 'B'=DOWN 'C'=ENTER 'D'=BACK  */

/* Per-row pin masks on GPIOC */
static const uint16_t KP_ROW_PINS[KP_ROWS] = {
    GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_11, GPIO_PIN_12
};
/* Combined mask of all keypad row pins — used to set all rows HIGH at once */
#define KP_ROW_ALL_PINS (GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_11 | GPIO_PIN_12)
/* Per-col pin masks on GPIOF */
static const uint16_t KP_COL_PINS[KP_COLS] = {
    GPIO_PIN_6, GPIO_PIN_7, GPIO_PIN_8, GPIO_PIN_9
};

static void keypad_init(void) {
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    /* Rows as push-pull outputs, initially high */
    g.Pin  = KP_ROW_ALL_PINS;
    g.Mode = GPIO_MODE_OUTPUT_PP; g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &g);
    HAL_GPIO_WritePin(GPIOC, KP_ROW_ALL_PINS, GPIO_PIN_SET);
    /* Cols as pull-up inputs */
    g.Pin  = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9;
    g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOF, &g);
}

static char keypad_scan(void) {
    for (int r = 0; r < KP_ROWS; r++) {
        /* Drive all rows high then pull row r low */
        HAL_GPIO_WritePin(GPIOC, KP_ROW_ALL_PINS, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOC, KP_ROW_PINS[r], GPIO_PIN_RESET);
        HAL_Delay(1);
        for (int c = 0; c < KP_COLS; c++) {
            if (!HAL_GPIO_ReadPin(GPIOF, KP_COL_PINS[c]))
                return KP_MAP[r][c];
        }
    }
    /* Restore all rows high */
    HAL_GPIO_WritePin(GPIOC, KP_ROW_ALL_PINS, GPIO_PIN_SET);
    return 0;
}

static char keypad_get_key(void) {
    static char last = 0;
    static uint32_t press_time = 0;
    char k = keypad_scan();
    if (k && k != last) {
        last = k; press_time = HAL_GetTick();
        return k;
    }
    if (!k) last = 0;
    return 0;
}

/* ── Spinner encoder (PB3=A, PB4=B, PE2=SW) ─────────────── */
/* PB3/PB4 replace PE0/PE1 which are FMC_NBL0/NBL1 on STM32F429. */
static volatile int8_t  spinner_delta = 0;
static volatile bool    spinner_sw    = false;

static void spinner_init(void) {
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    GPIO_InitTypeDef g = {0};
    /* PB3=A, PB4=B — plain inputs with pull-up */
    g.Pin=GPIO_PIN_3|GPIO_PIN_4;
    g.Mode=GPIO_MODE_INPUT; g.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOB,&g);

    /* Use EXTI on PB3 (EXTI3) for A-phase detection */
    g.Mode=GPIO_MODE_IT_RISING_FALLING; g.Pin=GPIO_PIN_3;
    HAL_GPIO_Init(GPIOB,&g);
    HAL_NVIC_SetPriority(EXTI3_IRQn,2,0);
    HAL_NVIC_EnableIRQ(EXTI3_IRQn);

    /* PE2 switch — EXTI2 unchanged */
    g.Mode=GPIO_MODE_IT_FALLING; g.Pin=GPIO_PIN_2; g.Pull=GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE,&g);
    HAL_NVIC_SetPriority(EXTI2_IRQn,2,0);
    HAL_NVIC_EnableIRQ(EXTI2_IRQn);
}

void EXTI3_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_3);
    int a = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3);
    int b = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4);
    spinner_delta += (a != b) ? 1 : -1;
}
void EXTI2_IRQHandler(void) {
    __HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_2);
    static uint32_t last_sw = 0;
    uint32_t now = HAL_GetTick();
    if (now - last_sw > 200u) { spinner_sw = true; last_sw = now; }
}

/* ── XPT2046 Touch (SPI, CPOL=0 CPHA=0) ─────────────────── */
typedef struct { int16_t x; int16_t y; bool valid; } TouchPt_t;

static uint16_t xpt2046_read(uint8_t cmd) {
    uint8_t tx[3] = {cmd, 0, 0}, rx[3] = {0};
    HAL_GPIO_WritePin(TCH_CS_PORT, TCH_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 3, 10);
    HAL_GPIO_WritePin(TCH_CS_PORT, TCH_CS_PIN, GPIO_PIN_SET);
    return ((uint16_t)rx[1] << 5u) | (rx[2] >> 3u);
}

static TouchPt_t touch_read(void) {
    TouchPt_t pt = {0,0,false};
    if (HAL_GPIO_ReadPin(TCH_IRQ_PORT, TCH_IRQ_PIN)) return pt; /* not pressed */
    uint16_t rx = xpt2046_read(0xD0u); /* X cmd */
    uint16_t ry = xpt2046_read(0x90u); /* Y cmd */
    /* Map 12-bit ADC to screen coords (calibrate constants to your panel) */
    pt.x = (int16_t)((int32_t)(rx - 200) * LCD_W  / 3700);
    pt.y = (int16_t)((int32_t)(ry - 200) * LCD_H  / 3700);
    pt.x = (pt.x < 0) ? 0 : (pt.x >= (int16_t)LCD_W)  ? (int16_t)(LCD_W-1)  : pt.x;
    pt.y = (pt.y < 0) ? 0 : (pt.y >= (int16_t)LCD_H) ? (int16_t)(LCD_H-1) : pt.y;
    pt.valid = true;
    return pt;
}

/* ── TIM2 — pulse repetition rate ───────────────────────── */
/* TIM2 is in PWM mode; ARR controls period, CCR1 controls width  */
static TIM_HandleTypeDef htim2, htim3, htim4;

static void timers_init(void) {
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    /* PA0 = TIM2 CH1 (pulse output) */
    GPIO_InitTypeDef g={0};
    g.Pin=GPIO_PIN_0; g.Mode=GPIO_MODE_AF_PP;
    g.Alternate=GPIO_AF1_TIM2; g.Speed=GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA,&g);

    /* TIM2: APB1 timer clock = 90 MHz (HCLK=180 MHz, APB1 div=4, ×2).
       Prescaler=89 → 1 MHz tick.
       ARR = 1 000 000 / rate_hz − 1                           */
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 89;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 1000000u / 10000u - 1u; /* default 10 kHz */
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_PWM_Init(&htim2);

    TIM_OC_InitTypeDef oc={0};
    oc.OCMode=TIM_OCMODE_PWM1; oc.Pulse=10u; /* 10us default width */
    oc.OCPolarity=TIM_OCPOLARITY_HIGH; oc.OCFastMode=TIM_OCFAST_DISABLE;
    HAL_TIM_PWM_ConfigChannel(&htim2,&oc,TIM_CHANNEL_1);

    /* TIM4: ramp tick — fires every (ramp_time_s*1000/steps) ms
       APB1 timer clock = 90 MHz; Prescaler=8999 → 10 kHz base */
    htim4.Instance=TIM4;
    htim4.Init.Prescaler=8999; /* 90MHz/9000 = 10kHz */
    htim4.Init.CounterMode=TIM_COUNTERMODE_UP;
    htim4.Init.Period=9999u; /* 1 s tick by default */
    htim4.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;
    HAL_TIM_Base_Init(&htim4);
    HAL_NVIC_SetPriority(TIM4_IRQn,0,0);
    HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

/* TIM4 ISR — advances ramp one step */
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
    /* Interpolate amplitude */
    uint32_t span = G.cfg.ramp_stop_uv > G.cfg.ramp_start_uv
                  ? G.cfg.ramp_stop_uv - G.cfg.ramp_start_uv : 0u;
    G.ramp_ampl_uv = G.cfg.ramp_start_uv +
                     (uint32_t)((uint64_t)span * ramp_step / RAMP_STEPS);
    /* S/H sequence: sample → update DAC → hold */
    HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_SET);  /* sample */
    dac_set_uv(G.ramp_ampl_uv);
    /* short settle — in real HW you would wait on busy or timer */
    for (volatile int d=0;d<200;d++);
    HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_RESET); /* hold */
}

/* Apply live config to TIM2 and DAC */
static void output_apply(void) {
    /* Stop timer temporarily */
    HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);

    if (!G.pulse_on) {
        dac_set_uv(0);
        HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_RESET);
        return;
    }

    /* Rate */
    uint32_t arr = 1000000u / G.cfg.rate_hz;
    if (arr == 0u) arr = 1u;
    __HAL_TIM_SET_AUTORELOAD(&htim2, arr - 1u);

    /* Width: width_ns / 1000 = µs; 1 tick = 1µs */
    uint32_t ccr = G.cfg.width_ns / 1000u;
    if (ccr >= arr) ccr = arr - 1u;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, ccr);

    /* Precision mode: set DAC directly */
    if (G.cfg.op_mode == OP_PRECISION) {
        HAL_GPIO_WritePin(SH_PORT, SH_PIN, GPIO_PIN_SET); /* always sample */
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
    output_apply(); /* start TIM2 */

    /* Configure TIM4 tick interval:
       total ticks = RAMP_STEPS per cycle
       period per tick (10 kHz base) = (ramp_time_s * 10000) / RAMP_STEPS */
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
 *  SECTION 8 — NON-VOLATILE CONFIG (Backup SRAM)
 * ============================================================ */
static void nv_init(void) {
    /* Enable backup SRAM clock and access */
    __HAL_RCC_BKPSRAM_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    if (NV->magic != 0xDEADBEEFu) {
        /* First boot — initialise defaults */
        memset(NV, 0, sizeof(*NV));
        NV->magic = 0xDEADBEEFu;
        /* Slot 0: factory default */
        PulseConfig_t *s = &NV->slots[0];
        s->valid = 0xA5u;
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
    if (slot == 0u) return; /* protect factory default */
    memset(&NV->slots[slot], 0, sizeof(PulseConfig_t));
    if (G.active_slot == slot) G.active_slot = 0u;
}

static void nv_recall_defaults(void) {
    nv_load_slot(0u);
    G.active_slot = 0u;
}

/* ============================================================
 *  SECTION 9 — RS-232 COMMAND PARSER
 * ============================================================ */
static char cmd_buf[CMD_MAX_LEN + 1];
static uint8_t cmd_len = 0;

static void cmd_respond(const char *msg) {
    uart_puts(msg); uart_puts("\r\n");
}

static void cmd_ok(void)  { cmd_respond("OK"); }
static void cmd_err(void) { cmd_respond("ERROR"); }

/* Dispatch a complete command string (already lower-cased) */
static void cmd_dispatch(char *cmd) {
    /* Trim trailing whitespace */
    int len = strlen(cmd);
    while (len > 0 && (cmd[len-1]=='\r'||cmd[len-1]=='\n'||cmd[len-1]==' '))
        cmd[--len]='\0';

    /* ── set trigger mode ── */
    if (strncmp(cmd,"set trigger mode ",17)==0) {
        const char *m = cmd+17;
        if (!strcmp(m,"internal"))   G.cfg.trig_src=TRIG_INTERNAL;
        else if(!strcmp(m,"external"))    G.cfg.trig_src=TRIG_EXTERNAL;
        else if(!strcmp(m,"gated"))       G.cfg.trig_src=TRIG_GATED;
        else if(!strcmp(m,"one pulse"))   G.cfg.trig_src=TRIG_ONE_PULSE;
        else { cmd_err(); return; }
        cmd_ok();
    }
    /* ── set threshold ── */
    else if (strncmp(cmd,"set threshold ",14)==0) {
        float v = strtof(cmd+14, NULL);
        G.cfg.trig_threshold_mv = (uint16_t)(v * 1000.0f);
        if (G.cfg.trig_threshold_mv < 100u)  G.cfg.trig_threshold_mv = 100u;
        if (G.cfg.trig_threshold_mv > 3500u) G.cfg.trig_threshold_mv = 3500u;
        cmd_ok();
    }
    /* ── set amplitude ── */
    else if (strncmp(cmd,"set amplitude ",14)==0) {
        float v = strtof(cmd+14, NULL);
        if (v < 0.0f) v = 0.0f;
        if (v > 10.0f) v = 10.0f;
        G.cfg.ampl_uv = (uint32_t)(v * 1e6f);
        if (G.pulse_on && G.cfg.op_mode==OP_PRECISION) dac_set_uv(G.cfg.ampl_uv);
        cmd_ok();
    }
    /* ── set rep rate ── */
    else if (strncmp(cmd,"set rep rate ",13)==0) {
        uint32_t r = (uint32_t)atoi(cmd+13);
        if (r < 1u) r = 1u; if (r > 100000u) r = 100000u;
        G.cfg.rate_hz = r;
        if (G.pulse_on) output_apply();
        cmd_ok();
    }
    /* ── set width ── (value in ns per RS-232 spec) */
    else if (strncmp(cmd,"set width ",10)==0) {
        uint32_t w = (uint32_t)atoi(cmd+10);
        if (w < 1000u)   w = 1000u;
        if (w > 1000000u) w = 1000000u;
        G.cfg.width_ns = w;
        if (G.pulse_on) output_apply();
        cmd_ok();
    }
    /* ── set rise time ── */
    else if (strncmp(cmd,"set rise time ",14)==0) {
        uint8_t idx = (uint8_t)atoi(cmd+14);
        if (idx >= N_RISE) idx = N_RISE-1u;
        G.cfg.rise_idx = idx;
        cmd_ok();
    }
    /* ── set fall time ── */
    else if (strncmp(cmd,"set fall time ",14)==0) {
        uint8_t idx = (uint8_t)atoi(cmd+14);
        if (idx >= N_FALL) idx = N_FALL-1u;
        G.cfg.fall_idx = idx;
        cmd_ok();
    }
    /* ── set attenuation ── */
    else if (strncmp(cmd,"set attenuation ",16)==0) {
        uint8_t idx = (uint8_t)atoi(cmd+16);
        if (idx >= N_ATTEN) idx = N_ATTEN-1u;
        G.cfg.atten_idx = idx;
        cmd_ok();
    }
    /* ── set power on/off ── */
    else if (strncmp(cmd,"set power on ",13)==0) {
        G.pulse_on = (cmd[13]=='1');
        output_apply();
        cmd_ok();
    }
    /* ── set polarity ── */
    else if (strncmp(cmd,"set polarity positive ",22)==0) {
        G.cfg.polarity = (cmd[22]=='0') ? 1u : 0u;
        cmd_ok();
    }
    /* ── set tail pulse ── */
    else if (strncmp(cmd,"set tail pulse ",15)==0) {
        G.cfg.pulse_top = (cmd[15]=='1') ? 1u : 0u;
        cmd_ok();
    }
    /* ── clamp baseline ── */
    else if (strncmp(cmd,"clamp baseline ",15)==0) {
        G.cfg.clamp = (cmd[15]=='1') ? 1u : 0u;
        cmd_ok();
    }
    /* ── ramp parameters ── */
    else if (strncmp(cmd,"set ramp startv ",16)==0) {
        float v = strtof(cmd+16,NULL);
        G.cfg.ramp_start_uv = (uint32_t)(v*1e6f);
        cmd_ok();
    }
    else if (strncmp(cmd,"set ramp stopv ",15)==0) {
        float v = strtof(cmd+15,NULL);
        G.cfg.ramp_stop_uv = (uint32_t)(v*1e6f);
        cmd_ok();
    }
    else if (strncmp(cmd,"set ramp startev ",17)==0) {
        float kev = strtof(cmd+17,NULL);
        G.cfg.ramp_start_uv = (uint32_t)(kev / G.cfg.kev_full_scale * 10e6f);
        cmd_ok();
    }
    else if (strncmp(cmd,"set ramp stopev ",16)==0) {
        float kev = strtof(cmd+16,NULL);
        G.cfg.ramp_stop_uv = (uint32_t)(kev / G.cfg.kev_full_scale * 10e6f);
        cmd_ok();
    }
    else if (strncmp(cmd,"set ramp time ",14)==0) {
        uint32_t t = (uint32_t)atoi(cmd+14);
        if (t < 30u) t=30u; if (t > 900u) t=900u;
        G.cfg.ramp_time_s = t;
        cmd_ok();
    }
    else if (strncmp(cmd,"set ramp cycles ",16)==0) {
        uint16_t c = (uint16_t)atoi(cmd+16);
        if (c < 1u) c=1u; if (c > 9999u) c=9999u;
        G.cfg.ramp_cycles = c;
        cmd_ok();
    }
    /* ── execute / stop ramp ── */
    else if (!strcmp(cmd,"execute ramp") || !strcmp(cmd,"exrcute ramp")) {
        if (!G.ramp_running) ramp_start();
        cmd_ok();
    }
    else if (!strcmp(cmd,"stop ramp")) {
        if (G.ramp_running) ramp_stop();
        cmd_ok();
    }
    /* ── trigger one pulse ── */
    else if (!strcmp(cmd,"trigger one pulse")) {
        G.pulse_on = true; output_apply();
        HAL_Delay(1);
        G.pulse_on = false; output_apply();
        cmd_ok();
    }
    /* ── display / scale ── */
    else if (strncmp(cmd,"set display kev ",16)==0) {
        G.cfg.display_kev = (cmd[16]=='1') ? 1u : 0u;
        cmd_ok();
    }
    else if (strncmp(cmd,"set equivalent kev ",19)==0) {
        G.cfg.kev_full_scale = (uint32_t)atoi(cmd+19);
        cmd_ok();
    }
    /* ── save / recall ── */
    else if (strncmp(cmd,"save config ",12)==0) {
        uint8_t slot = (uint8_t)atoi(cmd+12);
        nv_save_slot(slot);
        cmd_ok();
    }
    else if (strncmp(cmd,"recall config ",14)==0) {
        uint8_t slot = (uint8_t)atoi(cmd+14);
        nv_load_slot(slot);
        output_apply();
        cmd_ok();
    }
    else if (!strcmp(cmd,"recall factory defaults")) {
        nv_recall_defaults();
        output_apply();
        cmd_ok();
    }
    /* ── operating mode ── */
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
    /* ── help ── */
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
        cmd_respond("help");
    }
    else {
        cmd_err();
    }
}

/* Process one byte from UART into command buffer */
static void uart_process_byte(uint8_t byte) {
    /* Ctrl-L escape from remote mode regardless */
    if (byte == REMOTE_ESCAPE_CHAR) {
        G.remote_mode = false;
        cmd_respond("OK - LOCAL mode restored via Ctrl-L");
        render_screen();
        return;
    }
    if (byte == '\r' || byte == '\n') {
        if (cmd_len > 0u) {
            cmd_buf[cmd_len] = '\0';
            /* Lower-case for case-insensitive match */
            for (int i=0;i<(int)cmd_len;i++)
                cmd_buf[i]=(char)tolower((unsigned char)cmd_buf[i]);
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
 *  SECTION 10 — INPUT PROCESSOR  (keys, touch, spinner)
 * ============================================================ */

/* Map screen row touched to menu cursor index */
static int touch_to_item(int y, int first_item_y, int item_h) {
    if (y < first_item_y) return -1;
    return (y - first_item_y) / item_h;
}

/* Returns true if any input was consumed and we need a screen redraw */
static bool input_process(void) {
    /* ── UART (always processed, even in remote mode) ── */
    uint8_t byte;
    bool dirty = false;
    while (uart_getchar(&byte)) {
        uart_process_byte(byte);
        dirty = true;
    }

    /* ── If remote mode, discard ALL local input ── */
    if (G.remote_mode) {
        /* drain spinner and key events silently */
        spinner_delta = 0;
        spinner_sw    = false;
        keypad_get_key();
        return dirty;
    }

    /* ── Spinner ── */
    int8_t delta = 0;
    __disable_irq();
    delta = spinner_delta; spinner_delta = 0;
    __enable_irq();
    bool sw = spinner_sw; spinner_sw = false;

    if (delta != 0) {
        /* In main/sub menus: move cursor */
        if (M.current == MENU_EDIT_VALUE) {
            /* spinner fine-tunes value: todo for HW integration */
        } else {
            M.cursor += (delta > 0) ? 1 : -1;
            if (M.cursor < 0) M.cursor = 0;
        }
        dirty = true;
    }
    if (sw) {
        /* Push = confirm selection (same as ENTER) */
        goto handle_enter;
    }

    /* ── Keypad ── */
    {
        char k = keypad_get_key();
        if (k) {
            dirty = true;
            if (k >= '0' && k <= '9') {
                /* Numeric: append to edit buffer if editing */
                if (M.current == MENU_EDIT_VALUE && M.edit_pos < 20) {
                    M.edit_buf[M.edit_pos++] = k;
                    M.edit_buf[M.edit_pos]   = '\0';
                }
            } else if (k == '*') {
                /* '*' = decimal point */
                if (M.current == MENU_EDIT_VALUE && M.edit_pos < 20) {
                    M.edit_buf[M.edit_pos++] = '.';
                    M.edit_buf[M.edit_pos]   = '\0';
                }
            } else if (k == 'A') { /* UP */
                M.cursor = (M.cursor > 0) ? M.cursor - 1 : M.cursor;
            } else if (k == 'B') { /* DOWN */
                M.cursor++;
            } else if (k == 'D') { /* BACK */
                if (M.current == MENU_EDIT_VALUE) {
                    if (M.edit_pos > 0) { M.edit_buf[--M.edit_pos]='\0'; }
                    else { M.current = M.return_to; M.cursor = M.edit_field; }
                } else {
                    M.current = MENU_MAIN; M.cursor = 0;
                }
            } else if (k == 'C') { /* ENTER */
                handle_enter:;
                /* handle based on current screen */
                switch (M.current) {

                case MENU_MAIN:
                    M.cursor = (M.cursor > 5) ? 5 : M.cursor;
                    M.current = (MenuID_t)(MENU_TRIGGER + M.cursor);
                    M.cursor = 0;
                    break;

                case MENU_TRIGGER:
                    switch (M.cursor) {
                    case 0: /* cycle source */
                        G.cfg.trig_src = (G.cfg.trig_src + 1u) % 4u;
                        break;
                    case 1: /* threshold: open editor */
                        M.return_to   = MENU_TRIGGER;
                        M.edit_field  = 1;
                        M.current     = MENU_EDIT_VALUE;
                        M.edit_pos    = 0;
                        snprintf(M.edit_buf,sizeof(M.edit_buf),"%u",
                                 G.cfg.trig_threshold_mv);
                        break;
                    case 2: /* apply */
                        cmd_respond("TRIG APPLIED");
                        break;
                    case 3: /* return */
                        M.current=MENU_MAIN; M.cursor=0;
                        break;
                    }
                    break;

                case MENU_EDIT_VALUE: {
                    /* Confirm edit */
                    float v = strtof(M.edit_buf, NULL);
                    if (M.return_to == MENU_TRIGGER) {
                        G.cfg.trig_threshold_mv = (uint16_t)(v);
                        if (G.cfg.trig_threshold_mv<100u) G.cfg.trig_threshold_mv=100u;
                        if (G.cfg.trig_threshold_mv>3500u) G.cfg.trig_threshold_mv=3500u;
                    } else if (M.return_to == MENU_PULSE) {
                        switch (M.edit_field) {
                        case 1: G.cfg.ampl_uv  = (uint32_t)(v*1e6f); break;
                        case 2: G.cfg.rate_hz  = (uint32_t)v; break;
                        case 3: G.cfg.width_ns = (uint32_t)(v*1000.0f); break;
                        }
                        if (G.pulse_on) output_apply();
                    } else if (M.return_to == MENU_RAMP) {
                        switch (M.edit_field) {
                        case 0: G.cfg.ramp_start_uv=(uint32_t)(v*1e6f); break;
                        case 1: G.cfg.ramp_stop_uv =(uint32_t)(v*1e6f); break;
                        case 2: G.cfg.ramp_time_s  =(uint32_t)v; break;
                        case 3: G.cfg.ramp_cycles  =(uint16_t)v; break;
                        }
                    } else if (M.return_to == MENU_SCALE) {
                        if (M.edit_field == 1) G.cfg.kev_full_scale=(uint32_t)v;
                    }
                    M.current = M.return_to;
                    M.cursor  = M.edit_field;
                    break;
                }

                case MENU_PULSE:
                    switch (M.cursor) {
                    case 0: G.cfg.op_mode=(G.cfg.op_mode==OP_PRECISION)?OP_SLIDING:OP_PRECISION; break;
                    case 1: /* amplitude edit */
                        M.return_to=MENU_PULSE; M.edit_field=1;
                        M.current=MENU_EDIT_VALUE; M.edit_pos=0;
                        snprintf(M.edit_buf,sizeof(M.edit_buf),"%.6f",
                                 (double)G.cfg.ampl_uv/1e6); break;
                    case 2: /* rate edit */
                        M.return_to=MENU_PULSE; M.edit_field=2;
                        M.current=MENU_EDIT_VALUE; M.edit_pos=0;
                        snprintf(M.edit_buf,sizeof(M.edit_buf),"%lu",G.cfg.rate_hz); break;
                    case 3: /* width edit */
                        M.return_to=MENU_PULSE; M.edit_field=3;
                        M.current=MENU_EDIT_VALUE; M.edit_pos=0;
                        snprintf(M.edit_buf,sizeof(M.edit_buf),"%lu",G.cfg.width_ns/1000u); break;
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
                    switch(M.cursor) {
                    case 0: break; /* info line */
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
                    if (M.cursor==0) M.confirm_yes=true;
                    else             M.confirm_yes=false;
                    M.current = M.return_to;
                    break;

                default: break;
                } /* switch M.current */
            } /* ENTER */
        } /* key != 0 */
    }

    /* ── Touch ── */
    {
        static bool was_pressed = false;
        TouchPt_t pt = touch_read();
        if (pt.valid && !was_pressed) {
            was_pressed = true;
            dirty = true;
            int first_y, item_h;
            /* Map touch Y to cursor; same layout as draw functions */
            switch (M.current) {
            case MENU_MAIN:
                first_y = TITBH + 30; item_h = LHGT + 6;
                M.cursor = touch_to_item(pt.y, first_y, item_h);
                if (M.cursor >= 0 && M.cursor <= 5) {
                    M.current = (MenuID_t)(MENU_TRIGGER + M.cursor);
                    M.cursor = 0;
                }
                break;
            case MENU_SAVRECALL:
                /* Slot tiles: 5 rows x 2 cols */
                {
                    int ch = FONT_H + 12;
                    int cw = (LCD_W - 2*MRG - 8) / 2;
                    int row = (pt.y - (TITBH+10)) / (ch+4);
                    int col = (pt.x - MRG) / (cw+8);
                    int slot = row*2 + col;
                    if (slot >= 0 && slot < (int)NUM_CONFIG_SLOTS) {
                        M.savrecall_slot = slot;
                        M.cursor = slot;
                    } else {
                        first_y = TITBH + 10 + 5*(ch+4) + 8;
                        M.cursor = 10 + touch_to_item(pt.y, first_y, LHGT+2);
                    }
                }
                break;
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
 *  SECTION 11 — TEMPERATURE COMPENSATION  (every 5 min)
 * ============================================================ */
static void temp_comp_update(void) {
    /* Read internal temp sensor via ADC1 CH18 */
    /* In production: compute correction and apply to trim DAC */
    /* Simplified: just clear the flag */
    G.temp_comp_due = false;
}

/* TIM5 would be used as 5-min watchdog; simplified here */
static uint32_t last_temp_tick = 0u;
static void temp_comp_tick(void) {
    if (HAL_GetTick() - last_temp_tick >= 300000u) { /* 5 minutes */
        last_temp_tick = HAL_GetTick();
        G.temp_comp_due = true;
    }
}

/* ============================================================
 *  SECTION 12 — MAIN ENTRY POINT
 * ============================================================ */
void pulser_run(void) {
    /* --- Hardware init --- */
    sdram_fmc_init(); /* must be first: framebuffer lives in SDRAM */
    ltdc_init();
    spi2_init();
    dac_init();
    uart_init();
    keypad_init();
    spinner_init();
    timers_init();
    nv_init();

    /* --- Load last config --- */
    nv_load_slot(0u);
    G.pulse_on     = false;
    G.ramp_running = false;
    G.remote_mode  = false;   /* always boot in LOCAL */
    G.active_slot  = 0u;

    /* --- Initial render --- */
    memset(&M, 0, sizeof(M));
    M.current = MENU_MAIN;
    M.cursor  = 0;
    M.savrecall_slot = 0;
    render_screen();

    /* --- Splash: hold for 2s then go to main menu --- */
    fb_fill(C_BG);
    draw_title_bar("Sliding-Precision Pulse Generator — Booting");
    fb_str_c(0, LCD_H/2 - FONT_H,   LCD_W, "Initialising hardware...", C_FG, C_BG);
    fb_str_c(0, LCD_H/2,            LCD_W, "DAC: AD5791 18-bit  |  S/H: active", C_DIM, C_BG);
    fb_str_c(0, LCD_H/2 + FONT_H,   LCD_W, "UART: 9600 baud  |  10 config slots", C_DIM, C_BG);
    fb_str_c(0, LCD_H/2 + FONT_H*2, LCD_W, "Always boots in LOCAL mode", C_WARN, C_BG);
    HAL_Delay(2000u);

    render_screen();

    /* --- Main loop --- */
    uint32_t last_render = 0u;
    for (;;) {
        /* Process inputs */
        bool dirty = input_process();

        /* Temperature compensation */
        temp_comp_tick();
        if (G.temp_comp_due) temp_comp_update();

        /* Redraw at most every 50ms (20fps), or immediately on input */
        uint32_t now = HAL_GetTick();
        if (dirty || (now - last_render >= 50u)) {
            render_screen();
            last_render = now;
        }

        /* Small yield */
        HAL_Delay(5u);
    }
}
