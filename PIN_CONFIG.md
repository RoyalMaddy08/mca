# STM32F429ZIT6 — Pin Configuration Reference
## Sliding-Precision Pulse Generator Firmware

**MCU:** STM32F429ZIT6 (LQFP144, 2 MB Flash, 256 KB SRAM)  
**System Clock:** 180 MHz  
**Source file:** `menu.c` (`pulser_main_f429.c`)

---

## Table of Contents

1. [GPIO Port Tables (PA – PG)](#1-gpio-port-tables)
2. [Peripheral Quick Reference](#2-peripheral-quick-reference)
3. [Pin Conflict and Sharing Analysis](#3-pin-conflict-and-sharing-analysis)
4. [Critical Warnings](#4-critical-warnings)

---

## 1. GPIO Port Tables

### Port A

| Pin  | Signal            | Peripheral   | Mode / AF         | Notes |
|------|-------------------|--------------|-------------------|-------|
| PA0  | PULSE_OUT         | TIM2_CH1     | AF_PP / AF1       | Pulse output (rep-rate PWM) |
| PA2  | USART2_TX         | USART2       | AF_PP / AF7       | RS-232 transmit |
| PA3  | USART2_RX         | USART2       | AF_PP / AF7       | RS-232 receive ⚠️ see §3 |
| PA3  | LTDC_B5           | LTDC         | AF_PP / AF14      | Blue bit 5 ⚠️ conflicts USART2_RX |
| PA4  | LTDC_VSYNC        | LTDC         | AF_PP / AF14      | Vertical sync ⚠️ see §3 |
| PA4  | TCH_CS            | XPT2046      | OUTPUT_PP         | Touch chip-select ⚠️ conflicts LTDC_VSYNC |
| PA6  | LTDC_G2           | LTDC         | AF_PP / AF14      | Green bit 2 |
| PA11 | LTDC_R4           | LTDC         | AF_PP / AF14      | Red bit 4 |
| PA12 | LTDC_R5           | LTDC         | AF_PP / AF14      | Red bit 5 |

### Port B

| Pin  | Signal            | Peripheral   | Mode / AF         | Notes |
|------|-------------------|--------------|-------------------|-------|
| PB0  | LTDC_R3           | LTDC         | AF_PP / **AF9**   | Red bit 3 — code incorrectly uses AF14 ⚠️ see §4 |
| PB0  | TCH_IRQ           | XPT2046      | INPUT / PULLUP    | Touch interrupt ⚠️ conflicts LTDC_R3 |
| PB1  | LTDC_R6           | LTDC         | AF_PP / **AF9**   | Red bit 6 — code incorrectly uses AF14 ⚠️ see §4 |
| PB8  | LTDC_B6           | LTDC         | AF_PP / AF14      | Blue bit 6 |
| PB9  | LTDC_B7           | LTDC         | AF_PP / AF14      | Blue bit 7 |
| PB10 | LTDC_G4           | LTDC         | AF_PP / AF14      | Green bit 4 |
| PB11 | LTDC_G5           | LTDC         | AF_PP / AF14      | Green bit 5 |
| PB12 | DAC_CS            | AD5791       | OUTPUT_PP         | DAC chip-select (active-low) |
| PB13 | SPI2_SCK          | SPI2         | AF_PP / AF5       | SPI2 clock (DAC + touch) |
| PB14 | SPI2_MISO         | SPI2         | AF_PP / AF5       | SPI2 MISO |
| PB15 | SPI2_MOSI         | SPI2         | AF_PP / AF5       | SPI2 MOSI |

### Port C

| Pin  | Signal            | Peripheral   | Mode / AF         | Notes |
|------|-------------------|--------------|-------------------|-------|
| PC1  | S/H_HOLD          | Sample-Hold  | OUTPUT_PP         | High = sample, Low = hold. **Moved from PC0** ⚠️ see §4 |
| PC6  | LTDC_HSYNC        | LTDC         | AF_PP / AF14      | Horizontal sync |
| PC7  | LTDC_G6           | LTDC         | AF_PP / AF14      | Green bit 6 |
| PC10 | LTDC_R2           | LTDC         | AF_PP / AF14      | Red bit 2 |

### Port D

| Pin  | Signal            | Peripheral   | Mode / AF         | Notes |
|------|-------------------|--------------|-------------------|-------|
| PD0  | FMC_D2            | FMC SDRAM    | AF_PP / AF12      | Data bit 2 ⚠️ shared with keypad row |
| PD0  | KP_ROW0           | Keypad       | OUTPUT_PP         | Keypad row 0 ⚠️ shared with FMC_D2 |
| PD1  | FMC_D3            | FMC SDRAM    | AF_PP / AF12      | Data bit 3 ⚠️ shared with keypad row |
| PD1  | KP_ROW1           | Keypad       | OUTPUT_PP         | Keypad row 1 ⚠️ shared with FMC_D3 |
| PD2  | KP_ROW2           | Keypad       | OUTPUT_PP         | Keypad row 2 |
| PD3  | LTDC_G7           | LTDC         | AF_PP / AF14      | Green bit 7 ⚠️ conflicts keypad row |
| PD3  | KP_ROW3           | Keypad       | OUTPUT_PP         | Keypad row 3 ⚠️ conflicts LTDC_G7 |
| PD4  | KP_COL0           | Keypad       | INPUT / PULLUP    | Keypad column 0 |
| PD5  | KP_COL1           | Keypad       | INPUT / PULLUP    | Keypad column 1 |
| PD6  | LTDC_B2           | LTDC         | AF_PP / AF14      | Blue bit 2 ⚠️ conflicts keypad col |
| PD6  | KP_COL2           | Keypad       | INPUT / PULLUP    | Keypad column 2 ⚠️ conflicts LTDC_B2 |
| PD7  | KP_COL3           | Keypad       | INPUT / PULLUP    | Keypad column 3 |
| PD8  | FMC_D13           | FMC SDRAM    | AF_PP / AF12      | Data bit 13 |
| PD9  | FMC_D14           | FMC SDRAM    | AF_PP / AF12      | Data bit 14 |
| PD10 | FMC_D15           | FMC SDRAM    | AF_PP / AF12      | Data bit 15 |
| PD14 | FMC_D0            | FMC SDRAM    | AF_PP / AF12      | Data bit 0 |
| PD15 | FMC_D1            | FMC SDRAM    | AF_PP / AF12      | Data bit 1 |

### Port E

| Pin  | Signal            | Peripheral   | Mode / AF         | Notes |
|------|-------------------|--------------|-------------------|-------|
| PE0  | FMC_NBL0          | FMC SDRAM    | AF_PP / AF12      | Lower byte enable ⚠️ shared with spinner |
| PE0  | SPINNER_A         | Encoder      | INPUT / EXTI      | Quadrature phase A (EXTI0) ⚠️ shared with FMC_NBL0 |
| PE1  | FMC_NBL1          | FMC SDRAM    | AF_PP / AF12      | Upper byte enable ⚠️ shared with spinner |
| PE1  | SPINNER_B         | Encoder      | INPUT / PULLUP    | Quadrature phase B ⚠️ shared with FMC_NBL1 |
| PE2  | SPINNER_SW        | Encoder      | INPUT / EXTI      | Spinner push-button (EXTI2) |
| PE7  | FMC_D4            | FMC SDRAM    | AF_PP / AF12      | Data bit 4 |
| PE8  | FMC_D5            | FMC SDRAM    | AF_PP / AF12      | Data bit 5 |
| PE9  | FMC_D6            | FMC SDRAM    | AF_PP / AF12      | Data bit 6 |
| PE10 | FMC_D7            | FMC SDRAM    | AF_PP / AF12      | Data bit 7 |
| PE11 | FMC_D8            | FMC SDRAM    | AF_PP / AF12      | Data bit 8 |
| PE12 | FMC_D9            | FMC SDRAM    | AF_PP / AF12      | Data bit 9 |
| PE13 | FMC_D10           | FMC SDRAM    | AF_PP / AF12      | Data bit 10 |
| PE14 | FMC_D11           | FMC SDRAM    | AF_PP / AF12      | Data bit 11 |
| PE15 | FMC_D12           | FMC SDRAM    | AF_PP / AF12      | Data bit 12 |

### Port F

| Pin  | Signal            | Peripheral   | Mode / AF         | Notes |
|------|-------------------|--------------|-------------------|-------|
| PF0  | FMC_A0            | FMC SDRAM    | AF_PP / AF12      | Address bit 0 |
| PF1  | FMC_A1            | FMC SDRAM    | AF_PP / AF12      | Address bit 1 |
| PF2  | FMC_A2            | FMC SDRAM    | AF_PP / AF12      | Address bit 2 |
| PF3  | FMC_A3            | FMC SDRAM    | AF_PP / AF12      | Address bit 3 |
| PF4  | FMC_A4            | FMC SDRAM    | AF_PP / AF12      | Address bit 4 |
| PF5  | FMC_A5            | FMC SDRAM    | AF_PP / AF12      | Address bit 5 |
| PF10 | LTDC_DE           | LTDC         | AF_PP / AF14      | Data enable |
| PF11 | FMC_SDNRAS        | FMC SDRAM    | AF_PP / AF12      | Row address strobe (active-low) |
| PF12 | FMC_A6            | FMC SDRAM    | AF_PP / AF12      | Address bit 6 |
| PF13 | FMC_A7            | FMC SDRAM    | AF_PP / AF12      | Address bit 7 |
| PF14 | FMC_A8            | FMC SDRAM    | AF_PP / AF12      | Address bit 8 |
| PF15 | FMC_A9            | FMC SDRAM    | AF_PP / AF12      | Address bit 9 |

### Port G

| Pin  | Signal            | Peripheral   | Mode / AF         | Notes |
|------|-------------------|--------------|-------------------|-------|
| PG0  | FMC_A10           | FMC SDRAM    | AF_PP / AF12      | Address bit 10 |
| PG1  | FMC_A11           | FMC SDRAM    | AF_PP / AF12      | Address bit 11 |
| PG4  | FMC_BA0           | FMC SDRAM    | AF_PP / AF12      | Bank address bit 0 |
| PG5  | FMC_BA1           | FMC SDRAM    | AF_PP / AF12      | Bank address bit 1 |
| PG6  | LTDC_R7           | LTDC         | AF_PP / AF14      | Red bit 7 |
| PG7  | LTDC_CLK          | LTDC         | AF_PP / AF14      | Pixel clock |
| PG8  | FMC_SDCLK         | FMC SDRAM    | AF_PP / AF12      | SDRAM clock (HCLK/2 = 90 MHz) |
| PG10 | LTDC_G3           | LTDC         | AF_PP / **AF9**   | Green bit 3 — code incorrectly uses AF14 ⚠️ see §4 |
| PG11 | LTDC_B3           | LTDC         | AF_PP / AF14      | Blue bit 3 |
| PG12 | LTDC_B4           | LTDC         | AF_PP / **AF9**   | Blue bit 4 — code incorrectly uses AF14 ⚠️ see §4 |
| PG13 | FMC_SDNE0         | FMC SDRAM    | AF_PP / AF12      | Chip enable bank 0 (active-low) |
| PG14 | FMC_SDNWE         | FMC SDRAM    | AF_PP / AF12      | Write enable (active-low) |
| PG15 | FMC_SDNCAS        | FMC SDRAM    | AF_PP / AF12      | Column address strobe (active-low) |

---

## 2. Peripheral Quick Reference

### LTDC — RGB565 LCD Display (800 × 480)
Pixel clock ≈ 33.3 MHz (PLLSAIN=200, PLLSAIR=3, PLLSAIDIVR=2)

| Signal   | Pin   | AF   |
|----------|-------|------|
| R2       | PC10  | AF14 |
| R3       | PB0   | **AF9** |
| R4       | PA11  | AF14 |
| R5       | PA12  | AF14 |
| R6       | PB1   | **AF9** |
| R7       | PG6   | AF14 |
| G2       | PA6   | AF14 |
| G3       | PG10  | **AF9** |
| G4       | PB10  | AF14 |
| G5       | PB11  | AF14 |
| G6       | PC7   | AF14 |
| G7       | PD3   | AF14 |
| B2       | PD6   | AF14 |
| B3       | PG11  | AF14 |
| B4       | PG12  | **AF9** |
| B5       | PA3   | AF14 |
| B6       | PB8   | AF14 |
| B7       | PB9   | AF14 |
| CLK      | PG7   | AF14 |
| HSYNC    | PC6   | AF14 |
| VSYNC    | PA4   | AF14 |
| DE       | PF10  | AF14 |

> **Note:** Pins PB0, PB1, PG10, PG12 require **AF9** on the STM32F429.
> The firmware currently configures all LTDC pins with AF14 — see §4.

### FMC SDRAM — IS42S16400J (16-bit, Bank 5 = 0xC000_0000)
All FMC signals use **AF12**.

| Signal        | Pin(s)                                                    |
|---------------|-----------------------------------------------------------|
| A0–A5         | PF0–PF5                                                   |
| A6–A9         | PF12–PF15                                                 |
| A10–A11       | PG0–PG1                                                   |
| BA0–BA1       | PG4–PG5                                                   |
| D0–D1         | PD14–PD15                                                 |
| D2–D3         | PD0–PD1                                                   |
| D4–D12        | PE7–PE15                                                  |
| D13–D15       | PD8–PD10                                                  |
| NBL0–NBL1     | PE0–PE1                                                   |
| SDCLK         | PG8                                                       |
| SDNCAS        | PG15                                                      |
| SDNRAS        | PF11                                                      |
| SDNE0         | PG13                                                      |
| SDNWE         | PG14                                                      |

### SPI2 — AD5791 DAC + XPT2046 Touch Controller

| Signal        | Pin   | Mode / AF         |
|---------------|-------|-------------------|
| SPI2_SCK      | PB13  | AF_PP / AF5       |
| SPI2_MISO     | PB14  | AF_PP / AF5       |
| SPI2_MOSI     | PB15  | AF_PP / AF5       |
| AD5791 CS     | PB12  | OUTPUT_PP         |
| XPT2046 CS    | PA4   | OUTPUT_PP         |
| XPT2046 IRQ   | PB0   | INPUT / PULLUP    |
| S/H HOLD      | PC1   | OUTPUT_PP         |

### USART2 — RS-232 (9600 baud)

| Signal        | Pin   | Mode / AF         |
|---------------|-------|-------------------|
| USART2_TX     | PA2   | AF_PP / AF7       |
| USART2_RX     | PA3   | AF_PP / AF7       |

### Keypad — 4 × 4 Matrix

| Signal        | Pin(s)      | Mode              |
|---------------|-------------|-------------------|
| ROW0–ROW3     | PD0–PD3     | OUTPUT_PP (low = active row) |
| COL0–COL3     | PD4–PD7     | INPUT / PULLUP    |

Keymap (row, col):

|       | COL0 | COL1 | COL2 | COL3 |
|-------|------|------|------|------|
| ROW0  | `1`  | `2`  | `3`  | `A`  |
| ROW1  | `4`  | `5`  | `6`  | `B`  |
| ROW2  | `7`  | `8`  | `9`  | `C`  |
| ROW3  | `*`  | `0`  | `#`  | `D`  |

### Spinner Encoder — Rotary Input

| Signal        | Pin   | Mode / IRQ        |
|---------------|-------|-------------------|
| SPINNER_A     | PE0   | IT_RISING_FALLING / EXTI0 (priority 2) |
| SPINNER_B     | PE1   | INPUT / PULLUP    |
| SPINNER_SW    | PE2   | IT_FALLING / EXTI2 (priority 2, 200 ms debounce) |

### Timer PWM — Pulse Generation

| Signal        | Pin   | Peripheral    | Notes |
|---------------|-------|---------------|-------|
| PULSE_OUT     | PA0   | TIM2_CH1 AF1  | Rep-rate PWM; prescaler=89 → 1 MHz tick |
| (ramp tick)   | —     | TIM4          | 90 MHz / 9000 prescaler = 10 kHz tick |

---

## 3. Pin Conflict and Sharing Analysis

### 3a. Intended Sharing (FMC idle during scan)

These pins are intentionally multiplexed between FMC and a lower-speed
peripheral. Safe operation requires the firmware to reconfigure the pin
as GPIO **only when no FMC transaction is in progress**, then restore
the FMC alternate function before the next memory access.

| Pins        | FMC function  | Shared with     | Risk |
|-------------|---------------|-----------------|------|
| PD0–PD1     | FMC D2–D3     | Keypad ROW0–1   | Medium — PD GPIO overrides FMC AF during scan |
| PE0–PE1     | FMC NBL0–NBL1 | Spinner A/B     | Medium — Spinner uses EXTI while FMC may be active |

> **Production recommendation:** Remap the keypad to PH0–PH7 and the
> spinner to PI0–PI2 (available on the F429 LQFP144 package) to
> eliminate FMC contention entirely.

### 3b. True Pin Conflicts (same pin, incompatible peripherals)

The following pins are assigned to two incompatible peripherals by the
current firmware. The last `HAL_GPIO_Init()` call wins, meaning one
peripheral will malfunction.

| Pin  | Conflict                           | Impact |
|------|------------------------------------|--------|
| PA3  | LTDC_B5 (AF14) vs USART2_RX (AF7)  | Blue column pixel data OR serial receive — not both |
| PA4  | LTDC_VSYNC (AF14) vs XPT2046 CS    | LCD sync OR touch chip-select — not both |
| PB0  | LTDC_R3 (AF9) vs XPT2046 IRQ       | Red pixel data OR touch interrupt — not both |
| PD3  | LTDC_G7 (AF14) vs Keypad ROW3      | Green pixel data OR keypad row drive — not both |
| PD6  | LTDC_B2 (AF14) vs Keypad COL2      | Blue pixel data OR keypad column read — not both |

> These conflicts appear to be carry-over issues from the original F407
> design where some peripherals may have been mutually exclusive at
> runtime. They should be resolved before production use.

---

## 4. Critical Warnings

### ⚠️ W1 — Wrong Alternate Function for Four LTDC Pins

On the STM32F429, the following LTDC signals are on **AF9**, not AF14:

| Pin  | Signal  | Correct AF | Current Code |
|------|---------|------------|--------------|
| PB0  | LTDC_R3 | AF9        | AF14 (wrong) |
| PB1  | LTDC_R6 | AF9        | AF14 (wrong) |
| PG10 | LTDC_G3 | AF9        | AF14 (wrong) |
| PG12 | LTDC_B4 | AF9        | AF14 (wrong) |

All four pins are initialised with `GPIO_AF14_LTDC` in `ltdc_init()` at
the single `gpio.Alternate = GPIO_AF14_LTDC` assignment (line ~870).
They must instead use `GPIO_AF9_LTDC`.

**Fix:** Split the LTDC GPIO initialisation into two passes:

```c
/* Pass 1 — AF14 for most LTDC pins */
gpio.Alternate = GPIO_AF14_LTDC;
/* ... configure all pins except PB0, PB1, PG10, PG12 ... */

/* Pass 2 — AF9 for pins that require it on F429 */
gpio.Alternate = GPIO_AF9_LTDC;
gpio.Pin = GPIO_PIN_0; HAL_GPIO_Init(GPIOB, &gpio);  /* R3 */
gpio.Pin = GPIO_PIN_1; HAL_GPIO_Init(GPIOB, &gpio);  /* R6 */
gpio.Pin = GPIO_PIN_10; HAL_GPIO_Init(GPIOG, &gpio); /* G3 */
gpio.Pin = GPIO_PIN_12; HAL_GPIO_Init(GPIOG, &gpio); /* B4 */
```

**Symptom if not fixed:** Red bits R3/R6, green bit G3, and blue bit B4
will always read as 0, producing incorrect colour rendering.

---

### ⚠️ W2 — S/H Hold Pin Moved PC0 → PC1

The Sample-and-Hold control signal was originally on **PC0**. On some
STM32F429 board layouts PC0 routes to `FMC_SDNWE` (SDRAM write-enable).
To avoid contention the pin has been reassigned:

```c
/* menu.c */
#define SH_PORT  GPIOC
#define SH_PIN   GPIO_PIN_1   /* previously GPIO_PIN_0 */
```

> **Action required:** Verify your hardware schematic. If your board
> connects the S/H signal to PC0, add a wire or PCB rework to PC1,
> **or** change `SH_PIN` back to `GPIO_PIN_0` and ensure PC0 is not
> connected to the FMC SDNWE line.

---

### ⚠️ W3 — Keypad and Spinner Share Active FMC Pins

The keypad (PD0–PD7) and spinner (PE0–PE1) reuse pins that are
simultaneously configured as FMC data/byte-enable lines. The firmware
reconfigures them as plain GPIO for each scan, which temporarily
disables FMC access. Consequences:

- **Cache coherency:** A keypad scan or spinner read while the CPU is
  executing from SDRAM will stall the AHB bus until the scan completes.
- **EXTI conflict:** The spinner's EXTI0 interrupt fires on PE0 even
  when PE0 is configured as FMC_NBL0, potentially corrupting an
  in-flight SDRAM transaction.

**Mitigation (current code):** Keypad is only scanned in the main loop,
not inside interrupts. Spinner EXTI is low-priority (level 2). This is
safe for prototyping; move to dedicated GPIO for production.

---

### ⚠️ W4 — True Pin Conflicts Must Be Resolved

See §3b above. Five pins are assigned to two incompatible peripherals.
The recommended reassignments are:

| Conflicting use | Suggested fix |
|-----------------|---------------|
| USART2_RX on PA3 conflicts with LTDC_B5 | Move USART2 RX to PA10 (AF7) |
| XPT2046 CS on PA4 conflicts with LTDC_VSYNC | Move touch CS to an unused pin (e.g. PC3) |
| XPT2046 IRQ on PB0 conflicts with LTDC_R3 | Move touch IRQ to an unused pin (e.g. PC2) |
| Keypad ROW3 on PD3 conflicts with LTDC_G7 | Move ROW3 to PH3 or another free pin |
| Keypad COL2 on PD6 conflicts with LTDC_B2 | Move COL2 to PH6 or another free pin |

---

*Document generated from `menu.c` (pulser_main_f429.c) as shipped in
this repository. Verify against your specific PCB schematic before
programming.*
