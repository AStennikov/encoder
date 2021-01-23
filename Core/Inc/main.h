/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
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

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IPROPI_ADC_Pin GPIO_PIN_0
#define IPROPI_ADC_GPIO_Port GPIOA
#define OUT4_Pin GPIO_PIN_5
#define OUT4_GPIO_Port GPIOA
#define OUT3_Pin GPIO_PIN_6
#define OUT3_GPIO_Port GPIOA
#define OUT2_Pin GPIO_PIN_7
#define OUT2_GPIO_Port GPIOA
#define OUT1_Pin GPIO_PIN_4
#define OUT1_GPIO_Port GPIOC
#define GROUP1_Pin GPIO_PIN_12
#define GROUP1_GPIO_Port GPIOB
#define GROUP2_Pin GPIO_PIN_13
#define GROUP2_GPIO_Port GPIOB
#define GROUP3_Pin GPIO_PIN_14
#define GROUP3_GPIO_Port GPIOB
#define GROUP4_Pin GPIO_PIN_15
#define GROUP4_GPIO_Port GPIOB
#define GROUP5_Pin GPIO_PIN_6
#define GROUP5_GPIO_Port GPIOC
#define LED_GREEN_Pin GPIO_PIN_15
#define LED_GREEN_GPIO_Port GPIOA
#define LED_RED_Pin GPIO_PIN_10
#define LED_RED_GPIO_Port GPIOC
#define NFAULT_Pin GPIO_PIN_11
#define NFAULT_GPIO_Port GPIOC
#define NSLEEP_Pin GPIO_PIN_3
#define NSLEEP_GPIO_Port GPIOB
#define PMODE_Pin GPIO_PIN_6
#define PMODE_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
