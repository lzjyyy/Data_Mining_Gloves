/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

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
#define APP_ADDRESS 0x08020000U  // bootloader程序跳转地址
void System_VectorTable_Init(uint32_t vector_table_addr); 
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MOT2_PH_Pin GPIO_PIN_2
#define MOT2_PH_GPIO_Port GPIOE
#define MOT3_PH_Pin GPIO_PIN_3
#define MOT3_PH_GPIO_Port GPIOE
#define MOT4_PH_Pin GPIO_PIN_4
#define MOT4_PH_GPIO_Port GPIOE
#define MOT5_PH_Pin GPIO_PIN_5
#define MOT5_PH_GPIO_Port GPIOE
#define MOT6_PH_Pin GPIO_PIN_6
#define MOT6_PH_GPIO_Port GPIOE
#define LED2_Pin GPIO_PIN_13
#define LED2_GPIO_Port GPIOC
#define LED3_Pin GPIO_PIN_14
#define LED3_GPIO_Port GPIOC
#define LED4_Pin GPIO_PIN_15
#define LED4_GPIO_Port GPIOC
#define KEY1_Pin GPIO_PIN_2
#define KEY1_GPIO_Port GPIOB
#define MOT1_NFAULT_Pin GPIO_PIN_7
#define MOT1_NFAULT_GPIO_Port GPIOE
#define MOT2_NFAULT_Pin GPIO_PIN_8
#define MOT2_NFAULT_GPIO_Port GPIOE
#define MOT3_NFAULT_Pin GPIO_PIN_10
#define MOT3_NFAULT_GPIO_Port GPIOE
#define MOT6_NFAULT_Pin GPIO_PIN_15
#define MOT6_NFAULT_GPIO_Port GPIOE
#define LED1_Pin GPIO_PIN_12
#define LED1_GPIO_Port GPIOB
#define MOT6_NSLEEP_Pin GPIO_PIN_11
#define MOT6_NSLEEP_GPIO_Port GPIOD
#define MOT1_NSLEEP_Pin GPIO_PIN_0
#define MOT1_NSLEEP_GPIO_Port GPIOD
#define MOT2_NSLEEP_Pin GPIO_PIN_1
#define MOT2_NSLEEP_GPIO_Port GPIOD
#define MOT3_NSLEEP_Pin GPIO_PIN_3
#define MOT3_NSLEEP_GPIO_Port GPIOD
#define MOT4_NSLEEP_Pin GPIO_PIN_4
#define MOT4_NSLEEP_GPIO_Port GPIOD
#define RS485_RE_Pin GPIO_PIN_5
#define RS485_RE_GPIO_Port GPIOD
#define MOT5_NSLEEP_Pin GPIO_PIN_6
#define MOT5_NSLEEP_GPIO_Port GPIOD
#define MOT1_PH_Pin GPIO_PIN_1
#define MOT1_PH_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
