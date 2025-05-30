/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fft.h"
#include "bgt60ltr11_spi.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
// Radar 1 data structures
uint16_t IFI_radar1 = 0;
uint16_t IFQ_radar1 = 0;
float32_t buffer1_radar1[2 * FFT_BUFFER_SIZE];
float32_t buffer2_radar1[2 * FFT_BUFFER_SIZE];
float32_t *active_buffer_radar1 = buffer1_radar1;
float32_t *processing_buffer_radar1 = buffer2_radar1;
uint16_t acquired_sample_count_radar1 = 0;
uint8_t data_ready_f_radar1 = 0;
float32_t peak_index_radar1 = 0.0f;
float32_t max_value_radar1 = 0.0f;
float32_t target_velocity_radar1 = 0.0f;

// Radar 2 data structures
uint16_t IFI_radar2 = 0;
uint16_t IFQ_radar2 = 0;
float32_t buffer1_radar2[2 * FFT_BUFFER_SIZE];
float32_t buffer2_radar2[2 * FFT_BUFFER_SIZE];
float32_t *active_buffer_radar2 = buffer1_radar2;
float32_t *processing_buffer_radar2 = buffer2_radar2;
uint16_t acquired_sample_count_radar2 = 0;
uint8_t data_ready_f_radar2 = 0;
float32_t peak_index_radar2 = 0.0f;
float32_t max_value_radar2 = 0.0f;
float32_t target_velocity_radar2 = 0.0f;

// Shared variables
uint8_t radar1_initialized = 0;
uint8_t radar2_initialized = 0;
uint32_t error_cnt = 0;

#define VELOCITY_BUFFER_SIZE 3
float32_t velocity_buffer_radar1[VELOCITY_BUFFER_SIZE];
uint8_t velocity_buffer_index_radar1 = 0;
uint8_t velocity_buffer_filled_radar1 = 0;
float32_t velocity_average_radar1 = 0.0f;

float32_t velocity_buffer_radar2[VELOCITY_BUFFER_SIZE];
uint8_t velocity_buffer_index_radar2 = 0;
uint8_t velocity_buffer_filled_radar2 = 0;
float32_t velocity_average_radar2 = 0.0f;

#define RADAR1_CS_PORT GPIOC
#define RADAR1_CS_PIN  GPIO_PIN_13
#define RADAR2_CS_PORT GPIOA
#define RADAR2_CS_PIN  GPIO_PIN_9
#define Speaker_En_GPIO_Port GPIOB
#define Speaker_En_Pin GPIO_PIN_2
#define VOL_UP_PIN   Vol_1_Pin
#define VOL_DOWN_PIN Vol_2_Pin

#define MIN_FREQ 50
#define MAX_FREQ 2000
#define PWM_FREQ 20000
#define SINE_STEPS 32  // Number of steps in our sine lookup table

// Sine lookup table (32 values for one complete sine wave cycle)
// Values are scaled to fit PWM range (0-1600)
const uint16_t sine_table[SINE_STEPS] = {
    800,
    991,
    1175,
    1345,
    1491,
    1608,
    1688,
    1727,
    1727,
    1688,
    1608,
    1491,
    1345,
    1175,
    991,
    800,
    608,
    417,
    233,
    63,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    63,
    233,
    417,
    608,
    800
};

uint8_t sine_index = 0;        // Current position in sine table
int step_counter = 0;          // Counter for timing sine wave steps
int steps_per_cycle = 0;       // How many timer ticks between sine table updates
int update_counter = 0;        // Counter for frequency updates

volatile uint8_t volume_percent = 50;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	//TEST
  /* USER CODE END 1 */

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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */


  HAL_GPIO_WritePin(Speaker_En_GPIO_Port, Speaker_En_Pin, GPIO_PIN_SET);   // Enable the speaker

  // Initialize CS pins
  HAL_GPIO_WritePin(RADAR1_CS_PORT, RADAR1_CS_PIN, GPIO_PIN_SET);   // Radar 1 CS high
  HAL_GPIO_WritePin(RADAR2_CS_PORT, RADAR2_CS_PIN, GPIO_PIN_SET);  // Radar 2 CS high

  // Initialize Radar 1
  bgt60ltr11_HW_reset(RADAR_1);
  HAL_Delay(100);  // Wait for radar to stabilize
  if (bgt60ltr11_pulsed_mode_init_extended_range(RADAR_1) != HAL_OK) {
      Error_Handler();
  }

  // Initialize Radar 2
  bgt60ltr11_HW_reset(RADAR_2);
  HAL_Delay(100);  // Wait for radar to stabilize
  if (bgt60ltr11_pulsed_mode_init_extended_range(RADAR_2) != HAL_OK) {
      Error_Handler();
  }

  HAL_Delay(1000);
  HAL_TIM_Base_Start_IT(&htim3);

  // Radars successfully initialized
  radar1_initialized = 1;
  radar2_initialized = 1;

  for (uint8_t i = 0; i < VELOCITY_BUFFER_SIZE; i++) {
	  velocity_buffer_radar1[i] = 0.0f;
	  velocity_buffer_radar2[i] = 0.0f;
  }

  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, sine_table[0]);
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, sine_table[0]);

  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);  // Speaker 1 (PB10)
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);  // Speaker 2 (PB11)
  HAL_TIM_Base_Start_IT(&htim2);  // For square wave generation

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Process Radar 1 data when ready
	  if (radar1_initialized && data_ready_f_radar1) {
		  fft256_spectrum(processing_buffer_radar1);

		  //visualize_129_frequencies(processing_buffer_radar1, 129, 1);

		  find_peak_frequency(processing_buffer_radar1, FFT_BUFFER_SIZE, 1000, &peak_index_radar1, &max_value_radar1, &target_velocity_radar1);
		  printf("Radar 1 - velocity: %.5f m/s\t", target_velocity_radar1);
		  //printf("peak1: %0.5f", peak_index_radar1);
		  data_ready_f_radar1 = 0;
	  } else {
		  //printf("%-35s", "");
	  }

	  // Process Radar 2 data when ready
	  if (radar2_initialized && data_ready_f_radar2) {
		  fft256_spectrum(processing_buffer_radar2);
		  find_peak_frequency(processing_buffer_radar2, FFT_BUFFER_SIZE, 1000, &peak_index_radar2, &max_value_radar2, &target_velocity_radar2);
		  printf("Radar 2 - velocity: %.5f m/s", target_velocity_radar2);
		  //printf("peak2: %0.5f", peak_index_radar2);
		  data_ready_f_radar2 = 0;
	  } else {
		  printf("\r\n");
	  }
	  printf("\r\n");

	  HAL_Delay(10);

	  if (HAL_GPIO_ReadPin(GPIOB, VOL_UP_PIN) == GPIO_PIN_SET) {
	    if (volume_percent < 100){
	    	volume_percent += 10;
	    }
	    HAL_Delay(5);
	  }
	  if (HAL_GPIO_ReadPin(GPIOB, VOL_DOWN_PIN) == GPIO_PIN_SET) {
	    if (volume_percent > 0) {
	    	volume_percent -= 10;
	    }
	    HAL_Delay(5);
	  }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600-1;
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
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 31;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEL_1_GPIO_Port, SEL_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Speaker_En_GPIO_Port, Speaker_En_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SEL_2_GPIO_Port, SEL_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SEL_1_Pin */
  GPIO_InitStruct.Pin = SEL_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEL_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TDet_1_Pin PDet_1_Pin */
  GPIO_InitStruct.Pin = TDet_1_Pin|PDet_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pin : Speaker_En_Pin */
  GPIO_InitStruct.Pin = Speaker_En_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Speaker_En_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Vol_2_Pin Vol_1_Pin */
  GPIO_InitStruct.Pin = Vol_2_Pin|Vol_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SEL_2_Pin */
  GPIO_InitStruct.Pin = SEL_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SEL_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : TDet_2_Pin PDet_2_Pin */
  GPIO_InitStruct.Pin = TDet_2_Pin|PDet_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &htim3) {
        // Process both radars in the same interrupt

        // Process Radar 1
        if (acquired_sample_count_radar1 < FFT_BUFFER_SIZE) {
            if (bgt60ltr11_get_RAW_data(RADAR_1, &IFI_radar1, &IFQ_radar1) == HAL_OK) {
                if (IFI_radar1 <= 0x3FC && IFQ_radar1 <= 0x3FC) {
                    active_buffer_radar1[2 * acquired_sample_count_radar1] = (float32_t)(IFI_radar1 >> 2) / 255.0f;
                    active_buffer_radar1[2 * acquired_sample_count_radar1 + 1] = (float32_t)(IFQ_radar1 >> 2) / 255.0f;
                    acquired_sample_count_radar1++;
                }
            } else {
                error_cnt++;
            }
        } else {
            // Buffer full, swap buffers
            float32_t *temp = active_buffer_radar1;
            active_buffer_radar1 = processing_buffer_radar1;
            processing_buffer_radar1 = temp;
            data_ready_f_radar1 = 1;
            acquired_sample_count_radar1 = 0;
        }

        // Process Radar 2
        if (acquired_sample_count_radar2 < FFT_BUFFER_SIZE) {
            if (bgt60ltr11_get_RAW_data(RADAR_2, &IFI_radar2, &IFQ_radar2) == HAL_OK) {
                if (IFI_radar2 <= 0x3FC && IFQ_radar2 <= 0x3FC) {
                    active_buffer_radar2[2 * acquired_sample_count_radar2] = (float32_t)(IFI_radar2 >> 2) / 255.0f;
                    active_buffer_radar2[2 * acquired_sample_count_radar2 + 1] = (float32_t)(IFQ_radar2 >> 2) / 255.0f;
                    acquired_sample_count_radar2++;
                }
            } else {
                error_cnt++;
            }
        } else {
            // Buffer full, swap buffers
            float32_t *temp = active_buffer_radar2;
            active_buffer_radar2 = processing_buffer_radar2;
            processing_buffer_radar2 = temp;
            data_ready_f_radar2 = 1;
            acquired_sample_count_radar2 = 0;
        }
    } else if(htim == &htim2){

        bool det1 = (GPIOH->IDR & PDet_1_Pin) != 0;
        bool det2 = (GPIOA->IDR & PDet_2_Pin) != 0;

        // if neither radar sees motion, mute both channels immediately
        // This works!
        if (!det1 && !det2) {
            TIM2->CCR3 = 0;
            TIM2->CCR4 = 0;
            return;
        }


    	// every interrupt (clk/1600) it increases step
    	step_counter++;

    	if (step_counter >= steps_per_cycle) {
    		// if step is steps_per_cycle, Moves to the next position in the sine wave table
			step_counter = 0;

			sine_index = (sine_index + 1) % SINE_STEPS;

		}

    	uint16_t raw   = sine_table[sine_index];
    	uint16_t amp   = (raw * volume_percent) / 100;
    	TIM2->CCR3 = det1 ? amp : 0;
    	TIM2->CCR4 = det2 ? amp : 0;

		update_counter++;
		// Every 2000 interrupts:
		// Reads the latest peak frequency from radar processing
		// Recalculates steps_per_cycle to set the audio tone frequency

		if (update_counter >= 2000) {
			update_counter = 0;

			int freq = MIN_FREQ;  // Default to minimum frequency
			if (target_velocity_radar1 != 0.0f || target_velocity_radar2 != 0.0f) {

				int freq1 = (target_velocity_radar1 != 0.0f) ? (int)peak_index_radar1 : 0;
				int freq2 = (target_velocity_radar2 != 0.0f) ? (int)peak_index_radar2 : 0;

				freq = (freq1 > freq2) ? freq1 : freq2;
				if (freq < MIN_FREQ) freq = MIN_FREQ;
				if (freq > MAX_FREQ) freq = MAX_FREQ;
			}

			steps_per_cycle = PWM_FREQ / (freq * SINE_STEPS);
			if (steps_per_cycle < 1) steps_per_cycle = 1;
		}
    }
}



void sendDataToMonitor(float32_t vel) {
	printf("DATA,%.5f\n", vel);
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
