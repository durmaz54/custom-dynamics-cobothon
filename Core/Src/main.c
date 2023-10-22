/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "encoder_init.h"
#include "encoder.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX	1000
#define MED	499
#define MIN	0



/*GPIOC u,v,w hall sensor pin*/
#define 	HALL_PIN_U	GPIO_PIN_5
#define 	HALL_PIN_V	GPIO_PIN_6
#define 	HALL_PIN_W	GPIO_PIN_8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim15;

/* USER CODE BEGIN PV */
int8_t positionx = 0;
int8_t u_state=0;
int8_t v_state;
int8_t w_state;

typedef enum{
	pos1 = 5,
	pos2 = 4,
	pos3 = 3,
	pos4 = 2,
	pos5 = 1,
	pos6 = 0
}Poss;

uint16_t reading = 0x0000;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM15_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

int8_t readHallPosition(){
	/*	0b0000 0uvw	*/

	Poss positionx = 0;
	 u_state = HAL_GPIO_ReadPin(GPIOC, HALL_PIN_U);
	 v_state = HAL_GPIO_ReadPin(GPIOC, HALL_PIN_V);
	 w_state = HAL_GPIO_ReadPin(GPIOC, HALL_PIN_W);

	if((u_state == 1) && (v_state == 0) && (w_state == 1)){
		positionx = pos1;
	}
	else if((u_state == 1) && (v_state == 0) && (w_state == 0)){
		positionx = pos2;
	}
	else if((u_state == 1) && (v_state == 1) && (w_state == 0)){
		positionx = pos3;
	}
	else if((u_state == 0) && (v_state == 1) && (w_state == 0)){
		positionx = pos4;
	}else if((u_state == 0) && (v_state == 1) && (w_state == 1)){
		positionx = pos5;
	}else if((u_state == 0) && (v_state == 0) && (w_state == 1)){
		positionx = pos6;
	}


	return positionx;
}

void setMotorPosition(int8_t pos){

if(pos == pos1)
{
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MIN); //2-4
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MAX);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MED);

}
else if(pos == pos2){//
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MIN); // 2-6
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MED);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MAX);


}
else if(pos == pos3){

	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MED); //1-6
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MIN);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MAX);


}
else if(pos == pos4){
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MAX);//1-5
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MIN);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MED);



}
else if(pos == pos5){

	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MAX);//3-5
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MED);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MIN);



}
else if(pos == pos6){// 3-4

	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MED);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MAX);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MIN);


}
else{
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MED);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MED);
	__HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MED);
	HAL_Delay(1);

}


}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

uint16_t adrr;

#define BIT_MODITY(src, i, val) ((src) ^= (-(val) ^ (src)) & (1UL << (i)))
#define BIT_READ(src, i) (((src) >> (i)&1U))
#define BIT_TOGGLE(src, i) ((src) ^= 1UL << (i))

static uint8_t is_evenParity(uint16_t data)
{
  uint8_t shift = 1;
  while (shift < (sizeof(data) * 8))
  {
    data ^= (data >> shift);
    shift <<= 1;
  }
  return !(data & 0x1);
}

uint16_t calcAdress(uint16_t data)
{
  uint16_t frame = data & 0x3FFF;

  /* Data frame bit 14 always low(0). */
  BIT_MODITY(frame, 14, 0);

  /* Parity bit(even) calculated on the lower 15 bits. */
  if (!is_evenParity(frame)){
	BIT_TOGGLE(frame, 15);
  }

  return frame;
}

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

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM15_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */


  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_SET);
  HAL_Delay(500);


  //as5047p_init_t asp;
  //encoder_init(&asp);
  //as5047p_config(&asp, 0b00100101, 0x0007);
  //as5047p_setZero(&asp, 1052);

  as5047p_init_t a;
  encoder_init(&a);
  as5047p_config(&a, 0x0025, 0x0007);
  as5047p_setZero(&a, 1052);
  as5047p_readData(&a, AS5047P_SETTINGS1);
  as5047p_readData(&a, AS5047P_SETTINGS2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 positionx = readHallPosition();
	 setMotorPosition(positionx);
/*
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MAX);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MIN);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MED);
	  HAL_Delay(1);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MAX);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MED);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MIN);
	  HAL_Delay(1);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MED);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MAX);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MIN);
	  HAL_Delay(1);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MIN);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MAX);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MED);
	  HAL_Delay(1);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MIN);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MED);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MAX);
	  HAL_Delay(1);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1,MED);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2,MIN);
	  __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_3,MAX);
	  HAL_Delay(1);
*/

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 16;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 49;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 699;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LPUART1_RX_Pin */
  GPIO_InitStruct.Pin = LPUART1_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF12_LPUART1;
  HAL_GPIO_Init(LPUART1_RX_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC5 PC6 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
