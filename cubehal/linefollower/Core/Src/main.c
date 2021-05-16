/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum States
{
	CALIBRATING,
	DRIVING,
	OBSTACLE,
	INTERRUPTIONINI,
	INTERRUPTION,
}States;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t sensorValues[DMA_MEMDEPTH];
uint32_t weights[8] = { 0, 15, 30, 45, 60, 75, 90, 105 };
uint32_t minVals[8];
uint32_t maxVals[8];
uint32_t threshVals[8];
WEIGHTEDAVG_t filter;
uint32_t pos;

VL53_t s2;

volatile uint32_t ticksLeft = 0;
volatile uint32_t ticksRight = 0;

CTRL_t dTicks;
CTRL_t ctrl;

uint8_t wireRx[20];
uint8_t wireTx[20];
QUEUE_t wireRxQueue, wireTxQueue;
WIRE_t wire;

bool success;

uint16_t dists[2];

bool tp1 = false;
bool tp2 = false;

bool isLine = false;
bool isOilspill = false;
int stop = -101;
enum States state;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

void calibrateFiler();
bool validate();
void followTrack();
void turn(bool left, int speed, int ticks);
void goStraight(int speed, int ticks);
void obstacleSequence();
void interruptionSequence();
void checkforOilspill();

void ISR_ENCODER0()
{
	ticksLeft++;
}

void ISR_ENCODER1()
{
	ticksRight++;
}

void ISR_DMA(void)
{

}

void ISR_CTRL()
{
//	tp2 = !tp2;
//	HAL_GPIO_WritePin(TP2_GPIO_Port, TP2_Pin, tp2);
	followTrack();
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  state = CALIBRATING;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  Powertrain_Init();

  WEIGHTEDAVG_Init(&filter,
		  8,
		  sensorValues,
		  minVals,
		  maxVals,
		  1600,
		  weights);
  WEIGHTEDAVG_Invert(&filter, true);

  HAL_GPIO_WritePin(XS0_GPIO_Port, XS0_Pin, 1);
  HAL_GPIO_WritePin(XS1_GPIO_Port, XS1_Pin, 1);
  HAL_GPIO_WritePin(XS2_GPIO_Port, XS2_Pin, 1);
  HAL_GPIO_WritePin(XS3_GPIO_Port, XS3_Pin, 1);
  HAL_GPIO_WritePin(XS4_GPIO_Port, XS4_Pin, 1);

  HAL_Delay(100);

  QUEUE_Init(&wireRxQueue, (char*)wireRx, 20);
  QUEUE_Init(&wireTxQueue, (char*)wireTx, 20);
  HAL_ADC_Start_DMA(&hadc1, sensorValues, DMA_MEMDEPTH);
  WIRE_Init(&wire,
		  SDA_GPIO_Port, SDA_Pin,
		  SCL_GPIO_Port, SCL_Pin,
		  0,
		  &wireRxQueue, &wireTxQueue);
  WIRE_Begin(&wire);

  CTRL_Init(KP_LINE,
		  KI_LINE,
		  KD_LINE,
		  T_LINE,
		  100.0, 100.0, &ctrl);

  CTRL_Init(KP_DIFF,
		  KI_DIFF,
		  KD_DIFF,
		  T_DIFF,
		  100.0, 100.0, &dTicks);

  calibrateFiler();

  VL53_SetTimeout(&s2, 500);
  success = VL53_Init(&s2, &wire, 0x29, XS2_GPIO_Port, XS2_Pin);
  VL53_SetMeasurementTimingBudget(&s2, 20000);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  for(int i = 0; i < 2; i++)
	  {
		  if(!VL53_TimeoutOccured(&s2))
		  {
			  dists[i] = VL53_ReadRangeSingle_mm(&s2);
		  }
		  else
		  {
			  dists[i] = OBSTACLE_MINDIST_MM + 5;
		  }
	  }

	  if(dists[0] <= OBSTACLE_MINDIST_MM && dists[1] <= OBSTACLE_MINDIST_MM)
	  {
		  state = OBSTACLE;
		  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 1);
		  obstacleSequence();
		  state = DRIVING;
		  HAL_Delay(100);
	  }
	  else if(INTERRUPTIONINI != state)
	  {
		  HAL_GPIO_WritePin(LED0_GPIO_Port, LED0_Pin, 0);
		  state = DRIVING;
	  }

	  if(INTERRUPTIONINI == state)
	  {
		  state = INTERRUPTION;
//		  HAL_Delay(200);
		  if(!isLine)
		  {
			  interruptionSequence();
//			  Powertrain_SetDutycycles(stop, stop);
//			  HAL_Delay(100);
//			  obstacleSequence();
		  }
		  state = DRIVING;
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

  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_AnalogWDGConfTypeDef AnalogWDGConfig = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 8;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analog WatchDog 1
  */
  AnalogWDGConfig.WatchdogMode = ADC_ANALOGWATCHDOG_SINGLE_REG;
  AnalogWDGConfig.HighThreshold = 0;
  AnalogWDGConfig.LowThreshold = 0;
  AnalogWDGConfig.Channel = ADC_CHANNEL_0;
  AnalogWDGConfig.ITMode = DISABLE;
  if (HAL_ADC_AnalogWDGConfig(&hadc1, &AnalogWDGConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7250;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xffff;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  HAL_TIM_Base_Start_IT(&htim2);
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0x1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 3500;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */
  HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END TIM4_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED0_Pin|LED1_Pin|XS4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, XS0_Pin|XS1_Pin|XS2_Pin|XS3_Pin
                          |TP1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TP2_Pin|SCL_Pin|nSLEEP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED0_Pin LED1_Pin XS4_Pin */
  GPIO_InitStruct.Pin = LED0_Pin|LED1_Pin|XS4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BTN0_Pin BTN1_Pin SDA_Pin nFAULT_Pin */
  GPIO_InitStruct.Pin = BTN0_Pin|BTN1_Pin|SDA_Pin|nFAULT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC0_Pin ENC1_Pin */
  GPIO_InitStruct.Pin = ENC0_Pin|ENC1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : S8_Pin */
  GPIO_InitStruct.Pin = S8_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(S8_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : XS0_Pin XS1_Pin XS2_Pin XS3_Pin
                           TP1_Pin */
  GPIO_InitStruct.Pin = XS0_Pin|XS1_Pin|XS2_Pin|XS3_Pin
                          |TP1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : TP2_Pin nSLEEP_Pin */
  GPIO_InitStruct.Pin = TP2_Pin|nSLEEP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SCL_Pin */
  GPIO_InitStruct.Pin = SCL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SCL_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void Powertrain_Init()
{
	Powertrain_EnableMotors(1);
	Powertrain_SetDutycycles(0, 0);
}


void Powertrain_EnableMotors(int enableState)
{
	HAL_GPIO_WritePin(nSLEEP_GPIO_Port, nSLEEP_Pin, enableState);
}

void Powertrain_SetDutycycles(int left, int right)
{
	static int lastLeft = 0;
	static int lastRight = 0;

	if(left > 100)
	{
		left = 100;
	}
	if(left < -100 && left != stop)
	{
		left = -100;
	}

	if(right > 100)
	{
		right = 100;
	}
	if(right < -100 && right != stop)
	{
		right = -100;
	}

	left *= 100;
	right *= 100;

	if(left != lastLeft)
	{
		if(left < 0)
		{
			if(left < -100*100)
			{
				htim3.Instance->CCR1 = 100*100;
				htim3.Instance->CCR3 = 100*100;
			}
			else
			{
				left = -left;
				htim3.Instance->CCR1 = left;
				htim3.Instance->CCR3 = 0;
			}
		}
		else
		{
			htim3.Instance->CCR1 = 0;
			htim3.Instance->CCR3 = left;
		}
		lastLeft = left;
	}

	if(right != lastRight)
	{
		if(right < 0)
		{
			if(right < -100*100)
			{
				htim3.Instance->CCR2 = 100*100;
				htim3.Instance->CCR4 = 100*100;
			}
			else
			{
				right = -right;
				htim3.Instance->CCR2 = right;
				htim3.Instance->CCR4 = 0;
			}
		}
		else
		{
			htim3.Instance->CCR2 = 0;
			htim3.Instance->CCR4 = right;
		}
		lastRight = right;
	}
}

void Powertrain_Break(bool left, bool right)
{
	Powertrain_SetDutycycles(0, 0);
	if(left)
	{
		htim3.Instance->CCR1 = 100*100;
		htim3.Instance->CCR3 = 100*100;
	}
	if(right)
	{
		htim3.Instance->CCR2 = 100*100;
		htim3.Instance->CCR4 = 100*100;
	}
}

void calibrateFiler()
{
	state = CALIBRATING;
	while(HAL_GPIO_ReadPin(BTN1_GPIO_Port, BTN1_Pin))
	{
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		WEIGHTEDAVG_Calibrate(&filter);
	}
	for(int i = 0; i < 8; i++)
	{
		threshVals[i] = (minVals[i] + maxVals[i]) / 2;
	}
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
	state = DRIVING;
}

bool validate()
{
	uint32_t min = 0xffffffff;
	int minPos = 0;

	for(int i = 0; i < 8; i++)
	{
		if(sensorValues[i] < min)
		{
			minPos = i;
			min = sensorValues[i];
		}
	}

	if((min <= threshVals[minPos] && minPos < 5) || isOilspill)
	{
		return true;
	}
	else
	{
		return false;
	}
}

void followTrack()
{
	HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, 1);

	if(CALIBRATING == state)
	{
		return;
	}
	  for(int i = 0; i < DMA_MEMDEPTH; i += 8)
	  {
		  WEIGHTEDAVG_SetData(&filter, &sensorValues[i]);
		  WEIGHTEDAVG_AddSamples(&filter);
	  }
	  pos = WEIGHTEDAVG_Process(&filter);
	  checkforOilspill();
	  if(validate())
	  {
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 1);
		  isLine = true;
		  float delta = CTRL_Calculate((float)pos, (float)LINEPOS, &ctrl, false);
		  int left = V0 + (int)delta;
		  if(left < 0)
		  {
			  left = stop;
		  }
		  if(left > V0)
		  {
			  left = V0;
		  }
		  int right = V0 - (int)delta;
		  if(right < 0)
		  {
			  right = stop;
		  }
		  if(right > V0)
		  {
			  right = V0;
		  }

		  if(DRIVING == state)
		  {
			  Powertrain_SetDutycycles(left, right);
		  }
	  }
	  else
	  {
		  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, 0);
		  isLine = false;
		  if(INTERRUPTION != state && OBSTACLE != state)
		  {
			  state = INTERRUPTIONINI;
		  }
		  if(DRIVING == state || INTERRUPTIONINI == state)
		  {
			  Powertrain_SetDutycycles(stop, stop);
		  }
	  }
	HAL_GPIO_WritePin(TP1_GPIO_Port, TP1_Pin, 0);
}

void turn(bool left, int speed, int ticks)
{
	ticksLeft = 0;
	ticksRight = 0;
	if(left)
	{
		Powertrain_SetDutycycles(-speed, stop);
	}
	else
	{
		Powertrain_SetDutycycles(stop, -speed);
	}
	while((left && ticksLeft < ticks) || (!left && ticksRight < ticks)){}
	Powertrain_SetDutycycles(stop, stop);
}

void turnCenter(bool left, int speed, int ticks)
{
	ticksLeft = 0;
	ticksRight = 0;
	if(left)
	{
		Powertrain_SetDutycycles(-speed, speed);
	}
	else
	{
		Powertrain_SetDutycycles(speed, -speed);
	}
	while((left && ticksLeft < ticks) || (!left && ticksRight < ticks)){}
	Powertrain_SetDutycycles(stop, stop);
}

void goStraight(int speed, int ticks)
{
	ticksLeft = 0;
	ticksRight = 0;
	CTRL_Calculate(0.0, 0.0, &dTicks, true);
	while((ticksLeft < ticks) && (ticksRight < ticks))
	{
		int delta = ticksLeft - ticksRight;
		int dv = (float)CTRL_Calculate((float)delta, 0.0, &dTicks, false);
		int left = speed + dv;
		int right = speed - dv;
		if(left < 0)
		{
			left = stop;
		}
		if(right < 0)
		{
			right = stop;
		}
		Powertrain_SetDutycycles(left, right);
	}
	Powertrain_SetDutycycles(stop, stop);
}

void obstacleSequence()
{
	  Powertrain_SetDutycycles(stop, stop);
	  HAL_Delay(100);
	  turn(true, 40, 55);
	  HAL_Delay(10);
	  Powertrain_SetDutycycles(50, 50);
	  HAL_Delay(150);
	  Powertrain_SetDutycycles(40, 22);
	  HAL_Delay(500);
	  while(!isLine){}
	  goStraight(40, 20);
	  Powertrain_SetDutycycles(stop, stop);
	  HAL_Delay(150);
	  while(!isLine)
	  {
		  turn(true, 43, 2);
	  }
	  Powertrain_SetDutycycles(stop, stop);
	  HAL_Delay(150);
	  goStraight(40, 10);
}

void interruptionSequence()
{
	goStraight(V0, 30);
	Powertrain_SetDutycycles(stop, stop);
	HAL_Delay(10);
	bool first = true;
	bool found = false;
	while(!isLine)
	{
		int i = 20;
		if(first)
		{
			i = 10;
			first = false;
		}

		for(int k = 0; k < i; k++)
		{
			if(isLine || found)
			{
				found = true;
				break;
			}
			turn(true, V0, 2);
		}
		i = 20;
		Powertrain_SetDutycycles(stop, stop);
		HAL_Delay(50);
		for(int k = 0; k < i; k++)
		{
			if(isLine || found)
			{
				found = true;
				break;
			}
			turn(false, V0, 2);
		}
	}
}

void checkforOilspill()
{
	int count = 0;
	for(int i = 0; i < 8; i++)
	{
		if(sensorValues[i] < threshVals[i])
		{
			count ++;
		}
	}
	if(count > 5)
	{
		isOilspill = true;
		WEIGHTEDAVG_Invert(&filter, false);
	}
	else
	{
		isOilspill = false;
		WEIGHTEDAVG_Invert(&filter, true);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
