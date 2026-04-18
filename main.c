// This is for spg

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body for 20-bit Sliding Pulse Generator
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spg_20bit.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

// UART Command Buffer Size
#define UART_RX_BUFFER_SIZE     64
#define UART_TX_BUFFER_SIZE     128

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

// UART buffers
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
//static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static volatile uint8_t uart_rx_index = 0;
static volatile bool uart_data_ready = false;

// System tick counter
static volatile uint32_t system_ticks = 0;

// Button debounce timers
static uint32_t last_start_button_time = 0;
static uint32_t last_stop_button_time = 0;
static uint32_t last_mode_switch_time = 0;

// Last button states
static uint8_t last_start_state = 1;
static uint8_t last_stop_state = 1;
static uint8_t last_mode_state = 1;

// Sweep mode selection
typedef enum {
    UI_MODE_IDLE = 0,
    UI_MODE_SWEEPING,
    UI_MODE_SLIDING,
    UI_MODE_PRECISION
} UI_Mode_t;

static UI_Mode_t ui_mode = UI_MODE_IDLE;
static bool sliding_mode_enabled = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
static void ProcessUARTCommand(void);
static void CheckButtons(void);
static void PrintMenu(void);
static void LED_Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Retarget printf to UART
int __io_putchar(int ch) {
    HAL_UART_Transmit(&huart3, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

#ifdef __GNUC__
int _write(int file, char *ptr, int len) {
    (void)file;
    HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
#endif

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MPU Configuration--------------------------------------------------------*/
  MPU_Config();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */

  // Print startup banner
  printf("\r\n\r\n");
  printf("========================================\r\n");
  printf("   20-bit Sliding Pulse Generator\r\n");
  printf("   STM32H723ZG + AD5791 DAC\r\n");
  printf("   Resolution: %.3f μV per step\r\n", LSB_VOLTAGE * 1e6f);
  printf("========================================\r\n\r\n");

  // Initialize SPG
  SPG_20bit_Init();
  SPG_20bit_CalibrateOffset();

  // Set timer for sweep interrupts
  SPG_20bit_SetTimer(&htim2);

  // Start TIM2 interrupt for sweep steps
  HAL_TIM_Base_Start_IT(&htim2);

  // Start TIM3 PWM for pulse generation
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  // Start UART receive interrupt
  HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);

  // Set initial LED states
  HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_SWEEP_ACTIVE_GPIO_Port, LED_SWEEP_ACTIVE_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_RESET);

  // Print help menu
  PrintMenu();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    // Process UART commands
    if (uart_data_ready) {
        uart_data_ready = false;
        ProcessUARTCommand();
        // Restart UART receive
        HAL_UART_Receive_IT(&huart3, uart_rx_buffer, 1);
    }

    // Check button inputs
    CheckButtons();

    // Update LEDs based on state
    LED_Update();

    // Small delay to prevent watchdog issues
    HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 5;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T3_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_8CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 199;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 199;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, START_BUTTON_Pin|STOP_BUTTON_Pin|MODE_SWITCH_Pin|LED_STATUS_Pin
                          |LED_SWEEP_ACTIVE_Pin|LED_ERROR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, AD5791_CS_Pin|AD5791_LDAC_Pin|AD5791_RESET_Pin|AD5791_CLEAR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : START_BUTTON_Pin STOP_BUTTON_Pin MODE_SWITCH_Pin LED_STATUS_Pin
                           LED_SWEEP_ACTIVE_Pin LED_ERROR_Pin */
  GPIO_InitStruct.Pin = START_BUTTON_Pin|STOP_BUTTON_Pin|MODE_SWITCH_Pin|LED_STATUS_Pin
                          |LED_SWEEP_ACTIVE_Pin|LED_ERROR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EXT_TRIGGER_Pin */
  GPIO_InitStruct.Pin = EXT_TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EXT_TRIGGER_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : AD5791_CS_Pin AD5791_LDAC_Pin AD5791_RESET_Pin AD5791_CLEAR_Pin */
  GPIO_InitStruct.Pin = AD5791_CS_Pin|AD5791_LDAC_Pin|AD5791_RESET_Pin|AD5791_CLEAR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC2, SYSCFG_SWITCH_PC2_CLOSE);

  /*AnalogSwitch Config */
  HAL_SYSCFG_AnalogSwitchConfig(SYSCFG_SWITCH_PC3, SYSCFG_SWITCH_PC3_CLOSE);

  /**/
  HAL_I2CEx_EnableFastModePlus(SYSCFG_PMCR_I2C_PB6_FMP);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXT_TRIGGER_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXT_TRIGGER_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

//=============================================================================
// UART Command Processing
//=============================================================================

static void PrintMenu(void) {
    printf("\r\n");
    printf("========== SPG 20-bit Commands ==========\r\n");
    printf("  h         - Show this menu\r\n");
    printf("  s         - Start 50s triangle sweep\r\n");
    printf("  t         - Start 50s triangle sweep (same as s)\r\n");
    printf("  u         - Up sweep only (0V -> 10V)\r\n");
    printf("  d         - Down sweep only (10V -> 0V)\r\n");
    printf("  l         - Start sliding sweep (1μV resolution)\r\n");
    printf("  p         - Stop/Pause sweep\r\n");
    printf("  r         - Resume sweep\r\n");
    printf("  v X.XX    - Set voltage (e.g., v 5.0)\r\n");
    printf("  c X       - Set code (0-1048575)\r\n");
    printf("  status    - Show system status\r\n");
    printf("  progress  - Show sweep progress\r\n");
    printf("  readback  - Verify DAC output\r\n");
    printf("  mode0     - Disable sliding mode\r\n");
    printf("  mode1     - Enable sliding mode (1μV resolution)\r\n");
    printf("=========================================\r\n");
}

static void ProcessUARTCommand(void) {
    char cmd[UART_RX_BUFFER_SIZE];
//    char param[UART_RX_BUFFER_SIZE];

    // Copy command to string
    strncpy(cmd, (char*)uart_rx_buffer, uart_rx_index);
    cmd[uart_rx_index] = '\0';

    // Remove newline characters
    char* newline = strchr(cmd, '\r');
    if (newline) *newline = '\0';
    newline = strchr(cmd, '\n');
    if (newline) *newline = '\0';

    // Echo command
    printf("\r\n> %s\r\n", cmd);

    // Parse command
    if (strcmp(cmd, "h") == 0 || strcmp(cmd, "help") == 0) {
        PrintMenu();
    }
    else if (strcmp(cmd, "s") == 0 || strcmp(cmd, "t") == 0) {
        SPG_20bit_EnableSlidingScale(false);
        SPG_20bit_StartSweep(SWEEP_DURATION_50_SEC, SWEEP_MODE_TRIANGLE);
        ui_mode = UI_MODE_SWEEPING;
        printf("Triangle sweep started (50 seconds)\r\n");
    }
    else if (strcmp(cmd, "u") == 0) {
        SPG_20bit_EnableSlidingScale(false);
        SPG_20bit_StartSweep(SWEEP_DURATION_50_SEC, SWEEP_MODE_UP_ONLY);
        ui_mode = UI_MODE_SWEEPING;
        printf("Up sweep started (0V -> 10V, 50 seconds)\r\n");
    }
    else if (strcmp(cmd, "d") == 0) {
        SPG_20bit_EnableSlidingScale(false);
        SPG_20bit_StartSweep(SWEEP_DURATION_50_SEC, SWEEP_MODE_DOWN_ONLY);
        ui_mode = UI_MODE_SWEEPING;
        printf("Down sweep started (10V -> 0V, 50 seconds)\r\n");
    }
    else if (strcmp(cmd, "l") == 0) {
        SPG_20bit_EnableSlidingScale(true);
        SPG_20bit_StartSlidingSweep(SWEEP_DURATION_50_SEC);
        ui_mode = UI_MODE_SLIDING;
        sliding_mode_enabled = true;
        printf("Sliding sweep started (1μV resolution)\r\n");
    }
    else if (strcmp(cmd, "p") == 0) {
        SPG_20bit_StopSweep();
        ui_mode = UI_MODE_IDLE;
        printf("Sweep stopped\r\n");
    }
    else if (strcmp(cmd, "r") == 0) {
        SPG_20bit_ResumeSweep();
        if (sliding_mode_enabled) ui_mode = UI_MODE_SLIDING;
        else ui_mode = UI_MODE_SWEEPING;
        printf("Sweep resumed\r\n");
    }
    else if (strncmp(cmd, "v ", 2) == 0) {
        float voltage;
        if (sscanf(cmd + 2, "%f", &voltage) == 1) {
            SPG_20bit_SetVoltage(voltage);
            printf("Voltage set to %.6f V (Code: 0x%05lX)\r\n",
                   SPG_20bit_GetVoltage(), SPG_20bit_GetCode());
        } else {
            printf("Invalid voltage. Use: v 5.0\r\n");
        }
    }
    else if (strncmp(cmd, "c ", 2) == 0) {
        uint32_t code;
        if (sscanf(cmd + 2, "%lu", &code) == 1) {
            if (code <= AD5791_MAX_CODE) {
                SPG_20bit_SetCode(code);
                printf("Code set to 0x%05lX (%.6f V)\r\n", code, SPG_20bit_GetVoltage());
            } else {
                printf("Code out of range. Max: %lu\r\n", AD5791_MAX_CODE);
            }
        } else {
            printf("Invalid code. Use: c 524288\r\n");
        }
    }
    else if (strcmp(cmd, "status") == 0) {
        SPG_20bit_PrintStatus();
    }
    else if (strcmp(cmd, "progress") == 0) {
        SPG_20bit_PrintProgress();
    }
    else if (strcmp(cmd, "readback") == 0) {
        uint32_t readback = SPG_20bit_Readback();
        printf("Readback: 0x%05lX (%lu)\r\n", readback, readback);
        printf("Expected: 0x%05lX (%lu)\r\n", SPG_20bit_GetCode(), SPG_20bit_GetCode());
        int32_t diff = (int32_t)SPG_20bit_GetCode() - (int32_t)readback;
        printf("Difference: %ld codes (%.3f μV)\r\n", diff, diff * LSB_VOLTAGE * 1e6f);
    }
    else if (strcmp(cmd, "mode0") == 0) {
        SPG_20bit_EnableSlidingScale(false);
        sliding_mode_enabled = false;
        printf("Sliding mode disabled (standard 9.54μV resolution)\r\n");
    }
    else if (strcmp(cmd, "mode1") == 0) {
        SPG_20bit_EnableSlidingScale(true);
        sliding_mode_enabled = true;
        printf("Sliding mode enabled (1μV resolution using Gatti's method)\r\n");
    }
    else if (strlen(cmd) > 0) {
        printf("Unknown command. Type 'h' for help.\r\n");
    }
}

//=============================================================================
// Button Input Processing
//=============================================================================

static void CheckButtons(void) {
    uint32_t current_time = HAL_GetTick();

    // Start button (PC0) - Active low
    uint8_t start_state = HAL_GPIO_ReadPin(START_BUTTON_GPIO_Port, START_BUTTON_Pin);
    if (start_state == 0 && last_start_state == 1) {
        if ((current_time - last_start_button_time) > 50) { // Debounce
            last_start_button_time = current_time;
            printf("\r\n[Button] START pressed\r\n");
            SPG_20bit_EnableSlidingScale(sliding_mode_enabled);
            if (sliding_mode_enabled) {
                SPG_20bit_StartSlidingSweep(SWEEP_DURATION_50_SEC);
                ui_mode = UI_MODE_SLIDING;
            } else {
                SPG_20bit_StartSweep(SWEEP_DURATION_50_SEC, SWEEP_MODE_TRIANGLE);
                ui_mode = UI_MODE_SWEEPING;
            }
        }
    }
    last_start_state = start_state;

    // Stop button (PC1) - Active low
    uint8_t stop_state = HAL_GPIO_ReadPin(STOP_BUTTON_GPIO_Port, STOP_BUTTON_Pin);
    if (stop_state == 0 && last_stop_state == 1) {
        if ((current_time - last_stop_button_time) > 50) { // Debounce
            last_stop_button_time = current_time;
            printf("\r\n[Button] STOP pressed\r\n");
            SPG_20bit_StopSweep();
            ui_mode = UI_MODE_IDLE;
        }
    }
    last_stop_state = stop_state;

    // Mode switch (PC2) - For sliding mode toggle
    uint8_t mode_state = HAL_GPIO_ReadPin(MODE_SWITCH_GPIO_Port, MODE_SWITCH_Pin);
    if (mode_state != last_mode_state) {
        if ((current_time - last_mode_switch_time) > 50) { // Debounce
            last_mode_switch_time = current_time;
            if (mode_state == 0) {  // Switch pressed/toggled
                sliding_mode_enabled = !sliding_mode_enabled;
                SPG_20bit_EnableSlidingScale(sliding_mode_enabled);
                printf("\r\n[Switch] Sliding mode: %s\r\n",
                       sliding_mode_enabled ? "ENABLED (1μV)" : "DISABLED (9.54μV)");
            }
        }
    }
    last_mode_state = mode_state;
}

//=============================================================================
// LED Status Update
//=============================================================================

static void LED_Update(void) {
    static uint32_t last_blink = 0;
    static bool blink_state = false;
    uint32_t current_time = HAL_GetTick();

    // Status LED - Heartbeat
    if ((current_time - last_blink) > 1000) {
        last_blink = current_time;
        blink_state = !blink_state;
        HAL_GPIO_WritePin(LED_STATUS_GPIO_Port, LED_STATUS_Pin,
                          blink_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    // Sweep active LED
    if (SPG_20bit_GetProgress() > 0 && SPG_20bit_GetProgress() < 100) {
        HAL_GPIO_WritePin(LED_SWEEP_ACTIVE_GPIO_Port, LED_SWEEP_ACTIVE_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(LED_SWEEP_ACTIVE_GPIO_Port, LED_SWEEP_ACTIVE_Pin, GPIO_PIN_RESET);
    }
}

//=============================================================================
// UART Receive Callback
//=============================================================================

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart3) {
        if (uart_rx_buffer[uart_rx_index] == '\r' || uart_rx_buffer[uart_rx_index] == '\n') {
            uart_data_ready = true;
        } else {
            uart_rx_index++;
            if (uart_rx_index >= UART_RX_BUFFER_SIZE - 1) {
                uart_rx_index = 0;  // Buffer overflow, reset
            }
            HAL_UART_Receive_IT(&huart3, &uart_rx_buffer[uart_rx_index], 1);
        }
    }
}

//=============================================================================
// Timer Callback for Sweep Steps
//=============================================================================

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim2) {
        SPG_20bit_TimerISR();
    }
}

//=============================================================================
// External Interrupt Callback
//=============================================================================

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == EXT_TRIGGER_Pin) {
        // External trigger - start sweep
        printf("\r\n[EXT TRIGGER] Starting sweep\r\n");
        SPG_20bit_EnableSlidingScale(sliding_mode_enabled);
        if (sliding_mode_enabled) {
            SPG_20bit_StartSlidingSweep(SWEEP_DURATION_50_SEC);
        } else {
            SPG_20bit_StartSweep(SWEEP_DURATION_50_SEC, SWEEP_MODE_TRIANGLE);
        }
    }
}

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0x87;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  HAL_GPIO_WritePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin, GPIO_PIN_SET);
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  printf("Assert failed: %s at line %lu\r\n", file, line);
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
