/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32l0xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32l0xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

#define MIN_FREQ 50
#define MAX_FREQ 2000
#define PWM_FREQ 20000
#define SINE_STEPS 256  // Number of steps in our sine lookup table


static const uint16_t sine_table[SINE_STEPS] = {
		32768, 33572, 34375, 35178, 35979, 36779, 37575, 38369,
		39160, 39947, 40729, 41507, 42279, 43046, 43807, 44560,
		45307, 46046, 46777, 47500, 48214, 48919, 49613, 50298,
		50972, 51635, 52287, 52927, 53555, 54170, 54773, 55362,
		55938, 56499, 57047, 57579, 58097, 58600, 59087, 59558,
		60013, 60451, 60873, 61278, 61666, 62036, 62389, 62724,
		63041, 63339, 63620, 63881, 64124, 64348, 64553, 64739,
		64905, 65053, 65180, 65289, 65377, 65446, 65496, 65525,
		65535, 65525, 65496, 65446, 65377, 65289, 65180, 65053,
		64905, 64739, 64553, 64348, 64124, 63881, 63620, 63339,
		63041, 62724, 62389, 62036, 61666, 61278, 60873, 60451,
		60013, 59558, 59087, 58600, 58097, 57579, 57047, 56499,
		55938, 55362, 54773, 54170, 53555, 52927, 52287, 51635,
		50972, 50298, 49613, 48919, 48214, 47500, 46777, 46046,
		45307, 44560, 43807, 43046, 42279, 41507, 40729, 39947,
		39160, 38369, 37575, 36779, 35979, 35178, 34375, 33572,
		32768, 31963, 31160, 30357, 29556, 28756, 27960, 27166,
		26375, 25588, 24806, 24028, 23256, 22489, 21728, 20975,
		20228, 19489, 18758, 18035, 17321, 16616, 15922, 15237,
		14563, 13900, 13248, 12608, 11980, 11365, 10762, 10173,
		9597, 9036, 8488, 7956, 7438, 6935, 6448, 5977,
		5522, 5084, 4662, 4257, 3869, 3499, 3146, 2811,
		2494, 2196, 1915, 1654, 1411, 1187, 982, 796,
		630, 482, 355, 246, 158, 89, 39, 10,
		0, 10, 39, 89, 158, 246, 355, 482,
		630, 796, 982, 1187, 1411, 1654, 1915, 2196,
		2494, 2811, 3146, 3499, 3869, 4257, 4662, 5084,
		5522, 5977, 6448, 6935, 7438, 7956, 8488, 9036,
		9597, 10173, 10762, 11365, 11980, 12608, 13248, 13900,
		14563, 15237, 15922, 16616, 17321, 18035, 18758, 19489,
		20228, 20975, 21728, 22489, 23256, 24028, 24806, 25588,
		26375, 27166, 27960, 28756, 29556, 30357, 31160, 31963};

static uint32_t sine_index = 0;        // Current position in sine table
uint32_t sine_inc = (((uint64_t)1000)<<32)/PWM_FREQ;
int sine_amp = 1600;
//int step_counter = 0;          // Counter for timing sine wave steps
//int steps_per_cycle = 0;       // How many timer ticks between sine table updates
static int update_counter = 0;        // Counter for frequency updates

float peak_index_radar2 = 0.0f;
float target_velocity_radar2 = 0.0f;

float peak_index_radar1 = 0.0f;
float target_velocity_radar1 = 0.0f;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0+ Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable Interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32L0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32l0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
	// Updates PWM outputs to both speakers
	TIM2->CCR3 = (((uint32_t)sine_table[sine_index>>24]*sine_amp)>>16);
	TIM2->CCR4 = (((uint32_t)sine_table[sine_index>>24]*sine_amp)>>16);
	//sine_amp = {0..1600}
	sine_index += sine_inc;

	return;
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
