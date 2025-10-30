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
#include <stdio.h>
#include "nau7802.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define PI (3.14159265359f)   // computed from dt/(tau+dt)

char uartBuf[64];

// LOAD CELL
int32_t zeroOffset = 0;
double force_coeff = 8.76884498e-06;
float force_bias = 3.92787482 + 0.5315445;							// calculated calibration factors to map raw adc to force (0.5315445 from numerical calibration)
static float v_load_coeff = 0.5 * 1.225 * 0.12 * 0.15 * 1.28;		// [rho = 1.225, A = 0.12*0.15. Cd = 1.28]
static volatile float force = 0;
static volatile float v_load = 0;

static uint8_t samplingRate = 200; 					// Sampling rate
static volatile uint8_t outputBool = 0;				// Boolean to send the code over uart

// ENCODER PV
static volatile int32_t prevCount = 0;         		// last count (wrap-safe with int32_t)
static volatile int32_t latestCount = 0;
static volatile int32_t latestDelta = 0;
static volatile uint8_t sampleReady = 0;
static float encoder_coeff = 0.069776;				// Maps delta counts to linear speed
static float encoder_bias =  0.916947;
static volatile float v_enc = 0.0;

// Filter Params
static float fc_cross = 1.0;
static float dt = 1.0f / 200.0f;
static volatile float tau;
static volatile float alpha;
static float LP_load = 0;
static float LP_enc = 0;
static float v_est = 0;
static bool calibrationMode = false;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */
static inline void printUART(const char* s)
{
  HAL_UART_Transmit(&huart2, (uint8_t*)s, (uint16_t)strlen(s), 50);
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
  HAL_NVIC_SetPriority(TIM4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(TIM4_IRQn);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  MX_I2C1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */


  /* ----------- LOAD CELL VALUES INIT --------- */
  if (!NAU7802_begin(&hi2c1))
  	{
  		sprintf(uartBuf, "NAU7802 init failed!\r\n");
  		HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);
  		while (1);
  	}

  sprintf(uartBuf, "NAU7802 initialized!\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);

  // --- ZERO OFFSET CALIBRATION ---
  sprintf(uartBuf, "Place sensor with no wind at rest...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);
  HAL_Delay(3000); // wait 3 seconds for user

  zeroOffset = NAU7802_getAverage(&hi2c1, 64, 1000); // average 320 readings (1 second)
  NAU7802_setZeroOffset(zeroOffset);

  sprintf(uartBuf, "Zero offset: %ld\r\n", zeroOffset);
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);

  printf(uartBuf, "Place known weight (50g)...\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);
  HAL_Delay(5000); // wait 5 seconds for user

  // --- SCALE FACTOR CALIBRATION ---
//  sprintf(uartBuf, "Place known weight (1000g)...\r\n");
//  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);
//  HAL_Delay(5000); // wait 5 seconds for user
//
//  int32_t readingWithWeight = NAU7802_getAverage(&hi2c1, 64 , 1000);
//  calFactor = (float)(readingWithWeight - zeroOffset) / 1.0f; // units per gram
//  NAU7802_setCalibrationFactor(calFactor);

//  sprintf(uartBuf, "Calibration done. Factor: %.3f units/g\r\n", calFactor);
//  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);

  sprintf(uartBuf, "OUTPUT: Load Cell [g], Angular Velocity [deg/s]\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 100);


  /* ----------- ENCODER VALUES INIT --------- */
//   int counterValue = 0;
//   int pastCounterValue = 0;
//   float angleValue=0;

   // Zero and start the hardware encoder
   __HAL_TIM_SET_COUNTER(&htim2, 0);
   HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);


   /*----------- OUTPUT TIMER INIT ----------- */
   HAL_TIM_Base_Start_IT(&htim3);						// Init clock for outputting data

   // Compute filtering params
   tau = 1.0f /(2*PI*fc_cross);
   alpha = dt/(tau + dt);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Output when outputBool set to TRUE
	  if (outputBool == 1)
	  {
		  // reset the outputBool
		  outputBool = 0;

		  // Output the latest force and values from the  value
		  if (calibrationMode) {
			  sprintf(uartBuf, "%.4f, %ld \n", force, latestDelta);
			  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 50);

		  }
		  else {
			  sprintf(uartBuf, "%.4f\n", v_est);
			  HAL_UART_Transmit(&huart2, (uint8_t*)uartBuf, strlen(uartBuf), 50);
		  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 8400;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 50;
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
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* Prevent unused argument(s) compilation warning */
  if (htim->Instance==TIM3) {
	  outputBool = 1;


	  /* -------- LOAD CELL V ESTIMATE -------- */
	  float raw_adc = NAU7802_getReading(&hi2c1);
	  // convert to force based on calibration factors
	  force = force_coeff * raw_adc + force_bias;

	  // v^2 from the force equation
	  v_load = (force / v_load_coeff);

	  if (v_load < 0.0f) {
		  v_load = 0.0;
	  }
	  else {
		  v_load = sqrt(v_load);
	  }

	  /* -------- ENCODER V ESTIMATE -------- */
	  int32_t c = (int32_t)__HAL_TIM_GET_COUNTER(&htim2);
	  int32_t d = c - prevCount;   							// wrap-safe with signed math

	  // convert to linear velocity
	  v_enc = encoder_coeff * d + encoder_bias;

	  // update values
	  prevCount = c;
	  latestCount = c;
	  latestDelta = d;

	  /* --------  V ESTIMATE -------- */
	  // LP_force[i] = LP_force[i-1] + alpha * (v_est_load[i] - LP_force[i-1])
	  LP_load += alpha * (v_load - LP_load);
	  LP_enc   += alpha * (v_enc   - LP_enc);
	  v_est = (LP_enc + LP_load)/2;

  }
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
