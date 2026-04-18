/**
  ******************************************************************************
  * @file    spg_20bit.h
  * @brief   20-bit Sliding Pulse Generator header for AD5791 DAC
  *          with STM32H723ZG
  ******************************************************************************
  */

#ifndef SPG_20BIT_H
#define SPG_20BIT_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#ifdef __cplusplus
extern "C" {
#endif

// Declare external SPI handle (defined in main.c)
extern SPI_HandleTypeDef hspi1;

//=============================================================================
// AD5791 20-bit DAC Definitions
//=============================================================================
#define AD5791_RESOLUTION            20
#define AD5791_MAX_STEPS             (1UL << 20)     // 1,048,576
#define AD5791_MAX_CODE              0xFFFFFUL       // 20-bit max
#define AD5791_MIN_CODE              0x00000UL

// Reference Voltage (ADR445 - 10V precision reference)
#define VREF_VOLTAGE                 10.0f
#define LSB_VOLTAGE                  (VREF_VOLTAGE / AD5791_MAX_STEPS)  // 9.5367431640625 μV

// Sliding technique for 1μV resolution (Gatti's method)
#define SLIDING_STEPS_FOR_1UV        10
#define SLIDING_OFFSET_STEPS         9

//=============================================================================
// SPI Configuration (STM32H723ZG)
//=============================================================================
// SPI1 for AD5791 DAC
#define SPG_SPI                      hspi1
#define SPG_CS_PIN                   GPIO_PIN_0
#define SPG_CS_PORT                  GPIOB
#define SPG_LDAC_PIN                 GPIO_PIN_1
#define SPG_LDAC_PORT                GPIOB
#define SPG_RESET_PIN                GPIO_PIN_2
#define SPG_RESET_PORT               GPIOB
#define SPG_CLEAR_PIN                GPIO_PIN_6
#define SPG_CLEAR_PORT               GPIOB

//=============================================================================
// AD5791 Register Addresses
//=============================================================================
#define AD5791_REG_CONTROL           0x00
#define AD5791_REG_DAC               0x01
#define AD5791_REG_SOFTWARE          0x02
#define AD5791_REG_CLEARCODE         0x03
#define AD5791_REG_SOFTLOAD          0x04

//=============================================================================
// AD5791 Control Register Bits
//=============================================================================
#define AD5791_CTRL_SDO_EN           (1UL << 13)      // Enable SDO readback
#define AD5791_CTRL_SDO_DIS          (0UL << 13)
#define AD5791_CTRL_BIN2S            (0UL << 16)      // Straight binary
#define AD5791_CTRL_2SCOMP           (1UL << 16)      // Two's complement
#define AD5791_CTRL_RANGE_10V        (0UL << 2)       // 0V to 10V
#define AD5791_CTRL_RANGE_10_8V      (1UL << 2)       // -10V to +8V (not used)
#define AD5791_CTRL_RANGE_EXT        (2UL << 2)       // External range

//=============================================================================
// Sweep Configuration Enums
//=============================================================================
typedef enum {
    SWEEP_MODE_UP_ONLY = 0,      // 0V → 10V only
    SWEEP_MODE_DOWN_ONLY,        // 10V → 0V only
    SWEEP_MODE_TRIANGLE,         // 0V → 10V → 0V
    SWEEP_MODE_SAWTOOTH          // 0V → 10V, fast reset
} SweepMode_t;

typedef enum {
    SWEEP_DURATION_1_SEC = 1,
    SWEEP_DURATION_5_SEC = 5,
    SWEEP_DURATION_10_SEC = 10,
    SWEEP_DURATION_50_SEC = 50,
    SWEEP_DURATION_100_SEC = 100,
    SWEEP_DURATION_500_SEC = 500,
    SWEEP_DURATION_CUSTOM = 0
} SweepDuration_t;

//=============================================================================
// Configuration Structures
//=============================================================================
typedef struct {
    uint32_t start_code;
    uint32_t end_code;
    uint32_t step_size;
    uint32_t dwell_time_us;
    bool auto_increment;
} PrecisionStepConfig_t;

typedef struct {
    bool enabled;
    uint8_t total_slides;
    uint8_t current_slide;
    uint32_t base_offset_code;
    float offset_voltage_uv;
    bool sweep_complete;
} SlidingScaleConfig_t;

typedef struct {
    // DAC State
    uint32_t current_code;
    float current_voltage;

    // Sweep State
    SweepMode_t mode;
    SweepDuration_t duration;
    uint32_t step_delay_us;
    uint32_t total_steps_per_sweep;
    uint32_t current_step_index;
    bool direction_up;
    bool is_sweeping;

    // Sliding Scale
    SlidingScaleConfig_t sliding;

    // Precision Step Mode
    PrecisionStepConfig_t precision;

    // Statistics
    uint32_t total_steps_completed;
    uint32_t sweeps_completed;
    float sweep_start_time;
    float sweep_end_time;
} SPG_20bit_Config_t;

//=============================================================================
// External Global Variable
//=============================================================================
extern SPG_20bit_Config_t spg;

//=============================================================================
// Initialization Functions
//=============================================================================
void SPG_20bit_Init(void);
void SPG_20bit_Reset(void);
void SPG_20bit_CalibrateOffset(void);

//=============================================================================
// Direct DAC Control Functions
//=============================================================================
void SPG_20bit_SetCode(uint32_t code);
void SPG_20bit_SetVoltage(float voltage);
uint32_t SPG_20bit_GetCode(void);
float SPG_20bit_GetVoltage(void);
uint32_t SPG_20bit_Readback(void);

//=============================================================================
// Conversion Utility Functions
//=============================================================================
uint32_t SPG_20bit_VoltageToCode(float voltage);
float SPG_20bit_CodeToVoltage(uint32_t code);
uint32_t SPG_20bit_uVToCode(float microvolts);

//=============================================================================
// Sweep Control Functions
//=============================================================================
void SPG_20bit_StartSweep(SweepDuration_t duration, SweepMode_t mode);
void SPG_20bit_StartCustomSweep(float start_v, float end_v, float step_uv, uint32_t dwell_us);
void SPG_20bit_StopSweep(void);
void SPG_20bit_PauseSweep(void);
void SPG_20bit_ResumeSweep(void);

//=============================================================================
// Sliding Scale Functions (Gatti's Method for 1μV resolution)
//=============================================================================
void SPG_20bit_EnableSlidingScale(bool enable);
void SPG_20bit_SetSlidingResolution(float target_uv);
void SPG_20bit_StartSlidingSweep(SweepDuration_t duration);
void SPG_20bit_SlidingStep(void);

//=============================================================================
// Precision DNL Testing Functions
//=============================================================================
void SPG_20bit_PrecisionStepMode(uint32_t start_code, uint32_t end_code,
                                  uint32_t step_size, uint32_t dwell_us);
void SPG_20bit_SingleStep(void);

//=============================================================================
// Timer Integration Functions
//=============================================================================
void SPG_20bit_SetTimer(TIM_HandleTypeDef* htim);
void SPG_20bit_SetTimerHandle(TIM_HandleTypeDef* htim);
void SPG_20bit_TimerISR(void);

//=============================================================================
// Status and Debug Functions
//=============================================================================
float SPG_20bit_GetProgress(void);
uint32_t SPG_20bit_GetRemainingSteps(void);
void SPG_20bit_PrintProgress(void);
void SPG_20bit_PrintStatus(void);

#ifdef __cplusplus
}
#endif

#endif /* SPG_20BIT_H */
