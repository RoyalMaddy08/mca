/**
  ******************************************************************************
  * @file    spg_20bit.c
  * @brief   20-bit Sliding Pulse Generator implementation for AD5791 DAC
  *          with STM32H723ZG
  ******************************************************************************
  */

#include "spg_20bit.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

//=============================================================================
// Global Variables
//=============================================================================
SPG_20bit_Config_t spg = {0};
static TIM_HandleTypeDef* sweep_timer = NULL;
static volatile bool step_trigger = false;
static uint8_t spi_tx[4];
static uint8_t spi_rx[4];

//=============================================================================
// Private Helper Functions
//=============================================================================

// Send 32-bit command to AD5791
static void AD5791_SendCommand(uint32_t command) {
    // 32-bit command format: [23:20]=addr, [19:0]=data
    spi_tx[0] = (command >> 24) & 0xFF;
    spi_tx[1] = (command >> 16) & 0xFF;
    spi_tx[2] = (command >> 8) & 0xFF;
    spi_tx[3] = command & 0xFF;

    // Select DAC
    HAL_GPIO_WritePin(SPG_CS_PORT, SPG_CS_PIN, GPIO_PIN_RESET);

    // Send data
    HAL_SPI_Transmit(&SPG_SPI, spi_tx, 4, HAL_MAX_DELAY);

    // Deselect DAC
    HAL_GPIO_WritePin(SPG_CS_PORT, SPG_CS_PIN, GPIO_PIN_SET);

    // Pulse LDAC to update output
    HAL_GPIO_WritePin(SPG_LDAC_PORT, SPG_LDAC_PIN, GPIO_PIN_RESET);
    __NOP(); __NOP();  // Small delay
    HAL_GPIO_WritePin(SPG_LDAC_PORT, SPG_LDAC_PIN, GPIO_PIN_SET);
}

// Read back from AD5791
static uint32_t AD5791_Readback(void) {
    uint32_t command = (AD5791_REG_DAC << 24) | (1UL << 23);  // Read command

    spi_tx[0] = (command >> 24) & 0xFF;
    spi_tx[1] = (command >> 16) & 0xFF;
    spi_tx[2] = (command >> 8) & 0xFF;
    spi_tx[3] = command & 0xFF;

    HAL_GPIO_WritePin(SPG_CS_PORT, SPG_CS_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&SPG_SPI, spi_tx, spi_rx, 4, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPG_CS_PORT, SPG_CS_PIN, GPIO_PIN_SET);

    // Extract 20-bit data
    return ((spi_rx[1] & 0x0F) << 16) | (spi_rx[2] << 8) | spi_rx[3];
}

//=============================================================================
// Initialization
//=============================================================================

void SPG_20bit_Init(void) {
    // Configure GPIO pins
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // CS Pin
    GPIO_InitStruct.Pin = SPG_CS_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(SPG_CS_PORT, &GPIO_InitStruct);

    // LDAC Pin
    GPIO_InitStruct.Pin = SPG_LDAC_PIN;
    HAL_GPIO_Init(SPG_LDAC_PORT, &GPIO_InitStruct);

    // RESET Pin
    GPIO_InitStruct.Pin = SPG_RESET_PIN;
    HAL_GPIO_Init(SPG_RESET_PORT, &GPIO_InitStruct);

    // CLEAR Pin
    GPIO_InitStruct.Pin = SPG_CLEAR_PIN;
    HAL_GPIO_Init(SPG_CLEAR_PORT, &GPIO_InitStruct);

    // Set initial states
    HAL_GPIO_WritePin(SPG_CS_PORT, SPG_CS_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPG_LDAC_PORT, SPG_LDAC_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPG_RESET_PORT, SPG_RESET_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(SPG_CLEAR_PORT, SPG_CLEAR_PIN, GPIO_PIN_SET);

    // Hardware reset
    HAL_GPIO_WritePin(SPG_RESET_PORT, SPG_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SPG_RESET_PORT, SPG_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    // Configure AD5791 Control Register
    uint32_t control_cmd = (AD5791_REG_CONTROL << 24);
    control_cmd |= AD5791_CTRL_SDO_EN;      // Enable readback
    control_cmd |= AD5791_CTRL_BIN2S;       // Straight binary
    control_cmd |= AD5791_CTRL_RANGE_10V;   // 0-10V range

    AD5791_SendCommand(control_cmd);
    HAL_Delay(5);

    // Set output to 0V
    SPG_20bit_SetCode(0);

    // Initialize configuration structure
    spg.current_code = 0;
    spg.current_voltage = 0.0f;
    spg.mode = SWEEP_MODE_TRIANGLE;
    spg.direction_up = true;
    spg.is_sweeping = false;
    spg.total_steps_per_sweep = AD5791_MAX_STEPS;
    spg.current_step_index = 0;
    spg.step_delay_us = 100;  // Default 100μs per step
    spg.total_steps_completed = 0;
    spg.sweeps_completed = 0;

    // Initialize sliding scale
    spg.sliding.enabled = false;
    spg.sliding.total_slides = 10;
    spg.sliding.current_slide = 0;
    spg.sliding.base_offset_code = 0;
    spg.sliding.offset_voltage_uv = 0.0f;
    spg.sliding.sweep_complete = false;

    // Initialize precision step mode
    spg.precision.start_code = 0;
    spg.precision.end_code = AD5791_MAX_CODE;
    spg.precision.step_size = 1;
    spg.precision.dwell_time_us = 100;
    spg.precision.auto_increment = false;

    printf("SPG 20-bit initialized\n");
    printf("DAC LSB: %.3f μV\n", LSB_VOLTAGE * 1e6f);
}

void SPG_20bit_Reset(void) {
    HAL_GPIO_WritePin(SPG_RESET_PORT, SPG_RESET_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SPG_RESET_PORT, SPG_RESET_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
    SPG_20bit_SetCode(0);
    printf("SPG Reset complete\n");
}

void SPG_20bit_CalibrateOffset(void) {
    // Read back zero to check offset error
    SPG_20bit_SetCode(0);
    HAL_Delay(10);
    uint32_t readback = AD5791_Readback();

    if (readback != 0) {
        printf("Offset detected: %lu codes (%.3f μV)\n",
               readback, readback * LSB_VOLTAGE * 1e6f);
    } else {
        printf("No offset detected\n");
    }
}

//=============================================================================
// Direct DAC Control
//=============================================================================

void SPG_20bit_SetCode(uint32_t code) {
    if (code > AD5791_MAX_CODE) code = AD5791_MAX_CODE;

    uint32_t command = (AD5791_REG_DAC << 24) | (code & 0xFFFFF);
    AD5791_SendCommand(command);

    spg.current_code = code;
    spg.current_voltage = (float)code * LSB_VOLTAGE;
}

void SPG_20bit_SetVoltage(float voltage) {
    uint32_t code = SPG_20bit_VoltageToCode(voltage);
    SPG_20bit_SetCode(code);
}

uint32_t SPG_20bit_GetCode(void) {
    return spg.current_code;
}

float SPG_20bit_GetVoltage(void) {
    return spg.current_voltage;
}

uint32_t SPG_20bit_Readback(void) {
    return AD5791_Readback();
}

//=============================================================================
// Conversion Utilities
//=============================================================================

uint32_t SPG_20bit_VoltageToCode(float voltage) {
    if (voltage <= 0.0f) return 0;
    if (voltage >= VREF_VOLTAGE) return AD5791_MAX_CODE;

    // Add sliding offset if enabled
    float effective_voltage = voltage;
    if (spg.sliding.enabled) {
        effective_voltage += (spg.sliding.offset_voltage_uv / 1e6f);
        if (effective_voltage > VREF_VOLTAGE) effective_voltage = VREF_VOLTAGE;
    }

    return (uint32_t)((effective_voltage / VREF_VOLTAGE) * AD5791_MAX_STEPS + 0.5f);
}

float SPG_20bit_CodeToVoltage(uint32_t code) {
    if (code > AD5791_MAX_CODE) code = AD5791_MAX_CODE;
    return (float)code * LSB_VOLTAGE;
}

uint32_t SPG_20bit_uVToCode(float microvolts) {
    return (uint32_t)(microvolts / (LSB_VOLTAGE * 1e6f) + 0.5f);
}

//=============================================================================
// Sweep Configuration and Control
//=============================================================================

void SPG_20bit_StartSweep(SweepDuration_t duration, SweepMode_t mode) {
    if (spg.is_sweeping) {
        SPG_20bit_StopSweep();
        HAL_Delay(10);
    }

    spg.mode = mode;
    spg.duration = duration;
    spg.is_sweeping = true;
    spg.current_step_index = 0;
    spg.total_steps_completed = 0;

    // Calculate total steps based on mode
    switch(mode) {
        case SWEEP_MODE_UP_ONLY:
        case SWEEP_MODE_DOWN_ONLY:
            spg.total_steps_per_sweep = AD5791_MAX_STEPS;
            break;
        case SWEEP_MODE_TRIANGLE:
            spg.total_steps_per_sweep = AD5791_MAX_STEPS * 2;
            break;
        case SWEEP_MODE_SAWTOOTH:
            spg.total_steps_per_sweep = AD5791_MAX_STEPS;
            break;
        default:
            spg.total_steps_per_sweep = AD5791_MAX_STEPS;
            break;
    }

    // Calculate step delay based on duration
    float sweep_time_sec = (float)duration;
    spg.step_delay_us = (uint32_t)((sweep_time_sec / spg.total_steps_per_sweep) * 1e6f);
    if (spg.step_delay_us < 2) spg.step_delay_us = 2;  // Minimum 2μs

    // Set starting voltage
    switch(mode) {
        case SWEEP_MODE_UP_ONLY:
        case SWEEP_MODE_TRIANGLE:
        case SWEEP_MODE_SAWTOOTH:
            spg.direction_up = true;
            SPG_20bit_SetCode(0);
            break;
        case SWEEP_MODE_DOWN_ONLY:
            spg.direction_up = false;
            SPG_20bit_SetCode(AD5791_MAX_CODE);
            break;
    }

    const char* mode_str;
    switch(mode) {
        case SWEEP_MODE_UP_ONLY: mode_str = "UP"; break;
        case SWEEP_MODE_DOWN_ONLY: mode_str = "DOWN"; break;
        case SWEEP_MODE_TRIANGLE: mode_str = "TRIANGLE"; break;
        case SWEEP_MODE_SAWTOOTH: mode_str = "SAWTOOTH"; break;
        default: mode_str = "UNKNOWN"; break;
    }

    printf("Sweep started: %s, Duration: %d sec, Step delay: %lu μs, Total steps: %lu\n",
           mode_str, duration, spg.step_delay_us, spg.total_steps_per_sweep);
}

void SPG_20bit_StartCustomSweep(float start_v, float end_v, float step_uv, uint32_t dwell_us) {
    if (spg.is_sweeping) SPG_20bit_StopSweep();

    uint32_t start_code = SPG_20bit_VoltageToCode(start_v);
    uint32_t end_code = SPG_20bit_VoltageToCode(end_v);
    uint32_t step_code = SPG_20bit_uVToCode(step_uv);

    if (step_code == 0) step_code = 1;

    spg.precision.start_code = start_code;
    spg.precision.end_code = end_code;
    spg.precision.step_size = step_code;
    spg.precision.dwell_time_us = dwell_us;
    spg.precision.auto_increment = true;

    spg.step_delay_us = dwell_us;
    spg.is_sweeping = true;
    spg.direction_up = (end_code > start_code);
    spg.current_step_index = 0;

    // Calculate total steps
    uint32_t range = (end_code > start_code) ? (end_code - start_code) : (start_code - end_code);
    spg.total_steps_per_sweep = range / step_code;

    SPG_20bit_SetCode(start_code);

    printf("Custom sweep: %.3fV to %.3fV, step: %.2f μV, dwell: %lu μs, steps: %lu\n",
           start_v, end_v, step_uv, dwell_us, spg.total_steps_per_sweep);
}

void SPG_20bit_StopSweep(void) {
    spg.is_sweeping = false;
    spg.precision.auto_increment = false;

    if (sweep_timer) {
        HAL_TIM_Base_Stop_IT(sweep_timer);
    }

    printf("Sweep stopped at code: 0x%05lX (%.6f V)\n",
           spg.current_code, spg.current_voltage);
}

void SPG_20bit_PauseSweep(void) {
    if (sweep_timer) {
        HAL_TIM_Base_Stop_IT(sweep_timer);
        printf("Sweep paused\n");
    }
}

void SPG_20bit_ResumeSweep(void) {
    if (spg.is_sweeping && sweep_timer) {
        HAL_TIM_Base_Start_IT(sweep_timer);
        printf("Sweep resumed\n");
    }
}

//=============================================================================
// Sliding Scale (Gatti's Method) for 1μV Resolution
//=============================================================================

void SPG_20bit_EnableSlidingScale(bool enable) {
    spg.sliding.enabled = enable;
    if (!enable) {
        spg.sliding.current_slide = 0;
        spg.sliding.base_offset_code = 0;
        spg.sliding.offset_voltage_uv = 0.0f;
        spg.sliding.sweep_complete = false;
    }
    printf("Sliding scale %s\n", enable ? "enabled" : "disabled");
}

void SPG_20bit_SetSlidingResolution(float target_uv) {
    if (target_uv <= 0) target_uv = 1.0f;

    // Number of slides needed = LSB / target_resolution
    float lsb_uv = LSB_VOLTAGE * 1e6f;
    spg.sliding.total_slides = (uint8_t)(lsb_uv / target_uv + 0.5f);
    if (spg.sliding.total_slides < 1) spg.sliding.total_slides = 1;
    if (spg.sliding.total_slides > 20) spg.sliding.total_slides = 20;

    printf("Sliding scale: %d slides needed for %.1f μV resolution (DAC LSB = %.3f μV)\n",
           spg.sliding.total_slides, target_uv, lsb_uv);
}

void SPG_20bit_StartSlidingSweep(SweepDuration_t duration) {
    if (!spg.sliding.enabled) {
        SPG_20bit_EnableSlidingScale(true);
    }

    spg.sliding.current_slide = 0;
    spg.sliding.sweep_complete = false;
    spg.sliding.offset_voltage_uv = 0.0f;
    spg.sliding.base_offset_code = 0;

    // Start first sweep
    SPG_20bit_StartSweep(duration, SWEEP_MODE_TRIANGLE);

    printf("Sliding sweep started - Slide %d/%d (offset: %.1f μV)\n",
           spg.sliding.current_slide + 1, spg.sliding.total_slides,
           spg.sliding.offset_voltage_uv);
}

void SPG_20bit_SlidingStep(void) {
    // Call this when a sweep completes to advance to next slide
    if (!spg.sliding.enabled || spg.sliding.sweep_complete) return;

    spg.sliding.current_slide++;
    spg.sliding.offset_voltage_uv = (float)spg.sliding.current_slide *
                                     (LSB_VOLTAGE * 1e6f / spg.sliding.total_slides);
    spg.sliding.base_offset_code = SPG_20bit_uVToCode(spg.sliding.offset_voltage_uv);

    if (spg.sliding.current_slide >= spg.sliding.total_slides) {
        spg.sliding.sweep_complete = true;
        spg.is_sweeping = false;
        printf("Sliding sweep complete! All %d slides done.\n", spg.sliding.total_slides);
    } else {
        // Start next sweep with new offset
        SPG_20bit_StartSweep(spg.duration, SWEEP_MODE_TRIANGLE);
        printf("Sliding sweep - Slide %d/%d (offset: %.3f μV)\n",
               spg.sliding.current_slide + 1, spg.sliding.total_slides,
               spg.sliding.offset_voltage_uv);
    }
}

//=============================================================================
// Precision DNL Testing
//=============================================================================

void SPG_20bit_PrecisionStepMode(uint32_t start_code, uint32_t end_code,
                                  uint32_t step_size, uint32_t dwell_us) {
    SPG_20bit_StopSweep();

    spg.precision.start_code = start_code;
    spg.precision.end_code = end_code;
    spg.precision.step_size = (step_size == 0) ? 1 : step_size;
    spg.precision.dwell_time_us = dwell_us;
    spg.precision.auto_increment = false;

    spg.step_delay_us = dwell_us;
    SPG_20bit_SetCode(start_code);

    printf("Precision mode: Code 0x%05lX to 0x%05lX, step: %lu, dwell: %lu μs\n",
           start_code, end_code, step_size, dwell_us);
}

void SPG_20bit_SingleStep(void) {
    if (!spg.precision.auto_increment) {
        uint32_t next_code;

        if (spg.direction_up) {
            next_code = spg.current_code + spg.precision.step_size;
            if (next_code > spg.precision.end_code) {
                spg.direction_up = false;
                next_code = spg.current_code - spg.precision.step_size;
            }
        } else {
            if (spg.current_code > spg.precision.step_size) {
                next_code = spg.current_code - spg.precision.step_size;
            } else {
                next_code = spg.precision.start_code;
                spg.direction_up = true;
            }
        }

        SPG_20bit_SetCode(next_code);
        printf("Step to code: 0x%05lX (%.6f V)\n", next_code, spg.current_voltage);
    }
}

void SPG_20bit_SetTimerHandle(TIM_HandleTypeDef* htim) {
    sweep_timer = htim;
}

//=============================================================================
// Timer Interrupt Handler
//=============================================================================

void SPG_20bit_TimerISR(void) {
    if (!spg.is_sweeping) return;

    if (spg.precision.auto_increment) {
        // Custom precision step mode
        uint32_t next_code;
        int32_t step_dir = (spg.direction_up) ? (int32_t)spg.precision.step_size : -(int32_t)spg.precision.step_size;
        next_code = spg.current_code + step_dir;

        // Check boundaries
        if (spg.direction_up && next_code > spg.precision.end_code) {
            next_code = spg.precision.end_code;
            spg.is_sweeping = false;
        } else if (!spg.direction_up && next_code < spg.precision.start_code) {
            next_code = spg.precision.start_code;
            spg.is_sweeping = false;
        }

        SPG_20bit_SetCode(next_code);
        spg.current_step_index++;

    } else {
        // Standard sweep mode
        uint32_t next_code;

        switch(spg.mode) {
            case SWEEP_MODE_UP_ONLY:
                if (spg.current_code < AD5791_MAX_CODE) {
                    next_code = spg.current_code + 1;
                    SPG_20bit_SetCode(next_code);
                } else {
                    spg.is_sweeping = false;
                }
                break;

            case SWEEP_MODE_DOWN_ONLY:
                if (spg.current_code > 0) {
                    next_code = spg.current_code - 1;
                    SPG_20bit_SetCode(next_code);
                } else {
                    spg.is_sweeping = false;
                }
                break;

            case SWEEP_MODE_TRIANGLE:
                if (spg.direction_up) {
                    if (spg.current_code < AD5791_MAX_CODE) {
                        next_code = spg.current_code + 1;
                        SPG_20bit_SetCode(next_code);
                    } else {
                        spg.direction_up = false;
                        next_code = spg.current_code - 1;
                        SPG_20bit_SetCode(next_code);
                    }
                } else {
                    if (spg.current_code > 0) {
                        next_code = spg.current_code - 1;
                        SPG_20bit_SetCode(next_code);
                    } else {
                        spg.direction_up = true;
                        spg.sweeps_completed++;

                        // Check if sliding mode needs next slide
                        if (spg.sliding.enabled && !spg.sliding.sweep_complete) {
                            spg.is_sweeping = false;
                            SPG_20bit_SlidingStep();
                        } else {
                            spg.is_sweeping = false;
                        }
                    }
                }
                break;

            case SWEEP_MODE_SAWTOOTH:
                if (spg.current_code < AD5791_MAX_CODE) {
                    next_code = spg.current_code + 1;
                    SPG_20bit_SetCode(next_code);
                } else {
                    // Fast reset to 0
                    SPG_20bit_SetCode(0);
                    spg.sweeps_completed++;
                }
                break;

            default:
                break;
        }
    }

    spg.total_steps_completed++;
}

//=============================================================================
// Status and Debug
//=============================================================================

float SPG_20bit_GetProgress(void) {
    if (spg.total_steps_per_sweep == 0) return 0.0f;
    return (float)spg.current_step_index / spg.total_steps_per_sweep * 100.0f;
}

uint32_t SPG_20bit_GetRemainingSteps(void) {
    if (spg.total_steps_per_sweep <= spg.current_step_index) return 0;
    return spg.total_steps_per_sweep - spg.current_step_index;
}

void SPG_20bit_PrintProgress(void) {
    float progress = SPG_20bit_GetProgress();
    uint32_t remaining = SPG_20bit_GetRemainingSteps();

    printf("\n--- Sweep Progress ---\n");
    printf("Progress: %.2f%%\n", progress);
    printf("Steps: %lu / %lu\n", spg.current_step_index, spg.total_steps_per_sweep);
    printf("Remaining steps: %lu\n", remaining);

    if (spg.is_sweeping && spg.step_delay_us > 0 && remaining > 0) {
        uint32_t remaining_sec = (remaining * spg.step_delay_us) / 1000000;
        uint32_t remaining_min = remaining_sec / 60;
        remaining_sec = remaining_sec % 60;
        printf("Est. time remaining: %lu min %lu sec\n", remaining_min, remaining_sec);
    }
    printf("----------------------\n");
}

void SPG_20bit_PrintStatus(void) {
    printf("\n========== SPG 20-bit Status ==========\n");
    printf("DAC: AD5791, 20-bit, LSB = %.3f μV\n", LSB_VOLTAGE * 1e6f);
    printf("Current: Code = 0x%05lX (%lu), Voltage = %.6f V\n",
           spg.current_code, spg.current_code, spg.current_voltage);
    printf("Sweeping: %s\n", spg.is_sweeping ? "YES" : "NO");

    printf("Mode: ");
    switch(spg.mode) {
        case SWEEP_MODE_UP_ONLY: printf("UP ONLY\n"); break;
        case SWEEP_MODE_DOWN_ONLY: printf("DOWN ONLY\n"); break;
        case SWEEP_MODE_TRIANGLE: printf("TRIANGLE\n"); break;
        case SWEEP_MODE_SAWTOOTH: printf("SAWTOOTH\n"); break;
        default: printf("UNKNOWN\n"); break;
    }

    printf("Step delay: %lu μs\n", spg.step_delay_us);
    printf("Steps completed (total): %lu\n", spg.total_steps_completed);
    printf("Sweeps completed: %lu\n", spg.sweeps_completed);
    printf("Current step index: %lu / %lu\n", spg.current_step_index, spg.total_steps_per_sweep);

    printf("\n--- Sliding Scale ---\n");
    printf("Enabled: %s\n", spg.sliding.enabled ? "YES" : "NO");
    if(spg.sliding.enabled) {
        printf("Slide: %d/%d\n", spg.sliding.current_slide + 1, spg.sliding.total_slides);
        printf("Offset: %.3f μV (%lu codes)\n",
               spg.sliding.offset_voltage_uv, spg.sliding.base_offset_code);
        printf("Sweep complete: %s\n", spg.sliding.sweep_complete ? "YES" : "NO");
    }

    printf("\n--- Readback Verification ---\n");
    uint32_t readback = AD5791_Readback();
    printf("Readback code: 0x%05lX (%lu)\n", readback, readback);
    int32_t diff = (int32_t)spg.current_code - (int32_t)readback;
    printf("Difference: %ld codes (%.3f μV)\n", diff, diff * LSB_VOLTAGE * 1e6f);
    printf("======================================\n");
}

//=============================================================================
// Set Timer Reference (called from main.c)
//=============================================================================

void SPG_20bit_SetTimer(TIM_HandleTypeDef* htim) {
    sweep_timer = htim;
}
