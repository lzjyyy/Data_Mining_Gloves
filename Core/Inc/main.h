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
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
#define Pwr_3V3_Ctrl_Pin GPIO_PIN_2
#define Pwr_3V3_Ctrl_GPIO_Port GPIOE
#define Sys_Ref_Pin GPIO_PIN_3
#define Sys_Ref_GPIO_Port GPIOC
#define Sys_Vadc_Pin GPIO_PIN_0
#define Sys_Vadc_GPIO_Port GPIOB
#define Sys_Iadc_Pin GPIO_PIN_1
#define Sys_Iadc_GPIO_Port GPIOB
#define Pwr_24V_Ctrl_Pin GPIO_PIN_8
#define Pwr_24V_Ctrl_GPIO_Port GPIOE
#define Pwr_19V_Ctrl_Pin GPIO_PIN_9
#define Pwr_19V_Ctrl_GPIO_Port GPIOE
#define Pwr_12V_Ctrl_Pin GPIO_PIN_10
#define Pwr_12V_Ctrl_GPIO_Port GPIOE
#define Pwr_5V_Ctrl_Pin GPIO_PIN_11
#define Pwr_5V_Ctrl_GPIO_Port GPIOE
#define Led1_Pin GPIO_PIN_12
#define Led1_GPIO_Port GPIOE
#define Led0_Pin GPIO_PIN_13
#define Led0_GPIO_Port GPIOE
#define NET1_INT_Pin GPIO_PIN_15
#define NET1_INT_GPIO_Port GPIOE
#define NET1_RST_Pin GPIO_PIN_10
#define NET1_RST_GPIO_Port GPIOB
#define NET1_CS_Pin GPIO_PIN_12
#define NET1_CS_GPIO_Port GPIOB
#define RS485B_TX_Pin GPIO_PIN_8
#define RS485B_TX_GPIO_Port GPIOD
#define RS485B_RX_Pin GPIO_PIN_9
#define RS485B_RX_GPIO_Port GPIOD
#define Warning_Light_Pin GPIO_PIN_7
#define Warning_Light_GPIO_Port GPIOC
#define Distance_Limit_Pin GPIO_PIN_8
#define Distance_Limit_GPIO_Port GPIOC
#define Lift_Ctrl_1_Pin GPIO_PIN_9
#define Lift_Ctrl_1_GPIO_Port GPIOC
#define Lower_Limit_Pin GPIO_PIN_8
#define Lower_Limit_GPIO_Port GPIOA
#define RS485A_TX_Pin GPIO_PIN_9
#define RS485A_TX_GPIO_Port GPIOA
#define RS485A_RX_Pin GPIO_PIN_10
#define RS485A_RX_GPIO_Port GPIOA
#define Lift_Ctrl_0_Pin GPIO_PIN_11
#define Lift_Ctrl_0_GPIO_Port GPIOA
#define Upper_Limit_Pin GPIO_PIN_12
#define Upper_Limit_GPIO_Port GPIOA
#define RS485C_TX_Pin GPIO_PIN_10
#define RS485C_TX_GPIO_Port GPIOC
#define RS485C_RX_Pin GPIO_PIN_11
#define RS485C_RX_GPIO_Port GPIOC
#define CAN1_RX_Pin_Pin GPIO_PIN_0
#define CAN1_RX_Pin_GPIO_Port GPIOD
#define CAN1_TX_Pin_Pin GPIO_PIN_1
#define CAN1_TX_Pin_GPIO_Port GPIOD
#define RS485C_DIR_Pin GPIO_PIN_3
#define RS485C_DIR_GPIO_Port GPIOD
#define Dbg_Usart_Tx_Pin GPIO_PIN_5
#define Dbg_Usart_Tx_GPIO_Port GPIOD
#define Dbg_Usart_Rx_Pin GPIO_PIN_6
#define Dbg_Usart_Rx_GPIO_Port GPIOD
#define CAN2_RX_Pin_Pin GPIO_PIN_5
#define CAN2_RX_Pin_GPIO_Port GPIOB
#define CAN2_TX_Pin_Pin GPIO_PIN_6
#define CAN2_TX_Pin_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
  uint8_t get_w5500_init_status(void);
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
