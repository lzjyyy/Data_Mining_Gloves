/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.h
  * @brief   This file contains all the function prototypes for
  *          the gpio.c file
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
#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif

  /* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* USER CODE BEGIN Private defines */
#define LED_ON	GPIO_PIN_SET
#define LED_OFF	GPIO_PIN_RESET
/* USER CODE END Private defines */

  void MX_GPIO_Init(void);

  /* USER CODE BEGIN Prototypes */
  void Board_Led0_On(void);
  void Board_Led0_Off(void);
  void Board_Led1_On(void);
  void Board_Led1_Off(void);
  void Board_Pwr_24V_Enable(void);
  void Board_Pwr_24V_Disable(void);
  void Board_Pwr_19V_Enable(void);
  void Board_Pwr_19V_Disable(void);
  void Board_Pwr_12V_Enable(void);
  void Board_Pwr_12V_Disable(void);
  void Board_Pwr_5V_Enable(void);
  void Board_Pwr_5V_Disable(void);
  void Board_Pwr_3V3_Enable(void);
  void Board_Pwr_3V3_Disable(void);
  void Board_Pwr_Sequence_Enable(void);
  void Lift_Hold(void);
  void Lift_Up(void);
  void Lift_Down(void);
  bool Chk_UpperLimit_Reached(void);
  bool Chk_LowerLimit_Reached(void);
  bool Chk_Distance_Reached(void);
  void Warning_Light_On(void);
  void Warning_Light_Off(void);
  void Relay_0_On(void);
  void Relay_0_Off(void);
  void Relay_1_On(void);
  void Relay_1_Off(void);
  void Relay_2_On(void);
  void Relay_2_Off(void);
  void Relay_3_On(void);
  void Relay_3_Off(void);
  void Relay_4_On(void);
  void Relay_4_Off(void);
  void Relay_5_On(void);
  void Relay_5_Off(void);
  /* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */

