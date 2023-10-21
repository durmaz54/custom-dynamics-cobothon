/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

// INU -> PA8
#define IN_U_PIN        GPIO_PIN_8
#define IN_U_PORT       GPIOA

// ENU -> PB13
#define EN_U_PIN        GPIO_PIN_13
#define EN_U_PORT       GPIOB

// INV -> PA9
#define IN_V_PIN        GPIO_PIN_9
#define IN_V_PORT       GPIOA

// ENV -> PB14
#define EN_V_PIN        GPIO_PIN_14
#define EN_V_PORT       GPIOB

// INW -> PA10
#define IN_W_PIN        GPIO_PIN_10
#define IN_W_PORT       GPIOA

// ENW -> PB15
#define EN_W_PIN        GPIO_PIN_15
#define EN_W_PORT       GPIOB


#define IN_U_PIN_ON				HAL_GPIO_WritePin(IN_U_PORT, IN_U_PIN, GPIO_PIN_SET)
#define IN_V_PIN_ON				HAL_GPIO_WritePin(IN_V_PORT, IN_V_PIN, GPIO_PIN_SET)
#define IN_W_PIN_ON				HAL_GPIO_WritePin(IN_W_PORT, IN_W_PIN, GPIO_PIN_SET)

#define IN_U_PIN_OFF			HAL_GPIO_WritePin(IN_U_PORT, IN_U_PIN, GPIO_PIN_RESET)
#define IN_V_PIN_OFF			HAL_GPIO_WritePin(IN_V_PORT, IN_V_PIN, GPIO_PIN_RESET)
#define IN_W_PIN_OFF			HAL_GPIO_WritePin(IN_W_PORT, IN_W_PIN, GPIO_PIN_RESET)

#define IN_U_PIN_TOGGLE			HAL_GPIO_TogglePin(IN_U_PORT, IN_U_PIN)
#define IN_V_PIN_TOGGLE			HAL_GPIO_TogglePin(IN_V_PORT, IN_V_PIN)
#define IN_W_PIN_TOGGLE			HAL_GPIO_TogglePin(IN_W_PORT, IN_W_PIN)


#define EN_U_PIN_ON				HAL_GPIO_WritePin(EN_U_PORT, EN_U_PIN, GPIO_PIN_SET)
#define EN_V_PIN_ON				HAL_GPIO_WritePin(EN_V_PORT, EN_V_PIN, GPIO_PIN_SET)
#define EN_W_PIN_ON				HAL_GPIO_WritePin(EN_W_PORT, EN_W_PIN, GPIO_PIN_SET)

#define EN_U_PIN_OFF			HAL_GPIO_WritePin(EN_U_PORT, EN_U_PIN, GPIO_PIN_RESET)
#define EN_V_PIN_OFF			HAL_GPIO_WritePin(EN_V_PORT, EN_V_PIN, GPIO_PIN_RESET)
#define EN_W_PIN_OFF			HAL_GPIO_WritePin(EN_W_PORT, EN_W_PIN, GPIO_PIN_RESET)

#define EN_U_PIN_TOGGLE			HAL_GPIO_TogglePin(EN_U_PORT, EN_U_PIN)
#define EN_V_PIN_TOGGLE			HAL_GPIO_TogglePin(EN_V_PORT, EN_V_PIN)
#define EN_W_PIN_TOGGLE			HAL_GPIO_TogglePin(EN_W_PORT, EN_W_PIN)


/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EN_U_PIN_Pin GPIO_PIN_13
#define EN_U_PIN_GPIO_Port GPIOB
#define EN_V_PIN_Pin GPIO_PIN_14
#define EN_V_PIN_GPIO_Port GPIOB
#define EN_W_PIN_Pin GPIO_PIN_15
#define EN_W_PIN_GPIO_Port GPIOB
#define IN_U_PIN_Pin GPIO_PIN_8
#define IN_U_PIN_GPIO_Port GPIOA
#define IN_V_PIN_Pin GPIO_PIN_9
#define IN_V_PIN_GPIO_Port GPIOA
#define IN_W_PIN_Pin GPIO_PIN_10
#define IN_W_PIN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
