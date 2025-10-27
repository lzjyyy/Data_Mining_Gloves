/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    gpio.c
  * @brief   This file provides code for the configuration
  *          of all used GPIO pins.
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
#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as
        * Analog
        * Input
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, Pwr_3V3_Ctrl_Pin | Relay_Ctrl_0_Pin | Pwr_24V_Ctrl_Pin | Pwr_19V_Ctrl_Pin
    | Pwr_12V_Ctrl_Pin | Pwr_5V_Ctrl_Pin | Led1_Pin | Led0_Pin
    | NET1_INT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Relay_Ctrl_5_Pin | Relay_Ctrl_4_Pin | Relay_Ctrl_3_Pin | Lift_Ctrl_0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, Relay_Ctrl_2_Pin | Relay_Ctrl_1_Pin | Warning_Light_Pin | Lift_Ctrl_1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NET1_RST_GPIO_Port, NET1_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NET1_CS_GPIO_Port, NET1_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RS485C_DIR_GPIO_Port, RS485C_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PEPin PEPin PEPin PEPin
                           PEPin PEPin */
  GPIO_InitStruct.Pin = Pwr_3V3_Ctrl_Pin | Relay_Ctrl_0_Pin | Pwr_24V_Ctrl_Pin | Pwr_19V_Ctrl_Pin
    | Pwr_12V_Ctrl_Pin | Pwr_5V_Ctrl_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin PAPin */
  GPIO_InitStruct.Pin = Relay_Ctrl_5_Pin | Relay_Ctrl_4_Pin | Relay_Ctrl_3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = Relay_Ctrl_2_Pin | Relay_Ctrl_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PEPin PEPin */
  GPIO_InitStruct.Pin = Led1_Pin | Led0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = NET1_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(NET1_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PBPin PBPin */
  GPIO_InitStruct.Pin = NET1_RST_Pin | NET1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PCPin PCPin */
  GPIO_InitStruct.Pin = Warning_Light_Pin | Lift_Ctrl_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Distance_Limit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(Distance_Limit_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAPin PAPin */
  GPIO_InitStruct.Pin = Lower_Limit_Pin | Upper_Limit_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Lift_Ctrl_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Lift_Ctrl_0_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = RS485C_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RS485C_DIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 2 */
void Board_Led0_On(void)
{
  HAL_GPIO_WritePin(GPIOB, Led0_Pin, LED_ON);
}

void Board_Led0_Off(void)
{
  HAL_GPIO_WritePin(GPIOB, Led0_Pin, LED_OFF);
}

void Board_Led1_On(void)
{
  HAL_GPIO_WritePin(GPIOB, Led1_Pin, LED_ON);
}

void Board_Led1_Off(void)
{
  HAL_GPIO_WritePin(GPIOB, Led1_Pin, LED_OFF);
}

void Board_Pwr_24V_Enable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_24V_Ctrl_Pin, GPIO_PIN_SET);
}

void Board_Pwr_24V_Disable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_24V_Ctrl_Pin, GPIO_PIN_RESET);
}

void Board_Pwr_19V_Enable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_19V_Ctrl_Pin, GPIO_PIN_SET);
}

void Board_Pwr_19V_Disable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_19V_Ctrl_Pin, GPIO_PIN_RESET);
}

void Board_Pwr_12V_Enable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_12V_Ctrl_Pin, GPIO_PIN_SET);
}

void Board_Pwr_12V_Disable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_12V_Ctrl_Pin, GPIO_PIN_RESET);
}

void Board_Pwr_5V_Enable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_5V_Ctrl_Pin, GPIO_PIN_SET);
}

void Board_Pwr_5V_Disable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_5V_Ctrl_Pin, GPIO_PIN_RESET);
}

void Board_Pwr_3V3_Enable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_3V3_Ctrl_Pin, GPIO_PIN_SET);
}

void Board_Pwr_3V3_Disable(void)
{
  HAL_GPIO_WritePin(GPIOE, Pwr_3V3_Ctrl_Pin, GPIO_PIN_RESET);
}

void Board_Pwr_Sequence_Enable(void)
{
  Board_Pwr_24V_Enable();
  HAL_Delay(300);
  Board_Pwr_19V_Enable();
  HAL_Delay(300);
  Board_Pwr_12V_Enable();
  HAL_Delay(300);
  Board_Pwr_5V_Enable();
  HAL_Delay(300);
  Board_Pwr_3V3_Enable();
  HAL_Delay(300);
}

void Lift_Hold(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Lift_Ctrl_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Lift_Ctrl_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Lift_Ctrl_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Lift_Ctrl_0_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(Lift_Ctrl_0_GPIO_Port, Lift_Ctrl_0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(Lift_Ctrl_1_GPIO_Port, Lift_Ctrl_1_Pin, GPIO_PIN_RESET);
}

void Lift_Up(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Lift_Ctrl_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Lift_Ctrl_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Lift_Ctrl_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(Lift_Ctrl_0_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(Lift_Ctrl_1_GPIO_Port, Lift_Ctrl_1_Pin, GPIO_PIN_RESET);

}

void Lift_Down(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Lift_Ctrl_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  HAL_GPIO_Init(Lift_Ctrl_1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = Lift_Ctrl_0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(Lift_Ctrl_0_GPIO_Port, &GPIO_InitStruct);

  HAL_GPIO_WritePin(Lift_Ctrl_0_GPIO_Port, Lift_Ctrl_0_Pin, GPIO_PIN_RESET);
}

bool Chk_UpperLimit_Reached(void)
{
  if (HAL_GPIO_ReadPin(Upper_Limit_GPIO_Port, Upper_Limit_Pin) == GPIO_PIN_RESET)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Chk_LowerLimit_Reached(void)
{
  if (HAL_GPIO_ReadPin(Lower_Limit_GPIO_Port, Lower_Limit_Pin) == GPIO_PIN_RESET)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool Chk_Distance_Reached(void)
{
  if (HAL_GPIO_ReadPin(Distance_Limit_GPIO_Port, Distance_Limit_Pin) == GPIO_PIN_RESET)
  {
    return true;
  }
  else
  {
    return false;
  }

}

void Warning_Light_On(void)
{
  HAL_GPIO_WritePin(Warning_Light_GPIO_Port, Warning_Light_Pin, GPIO_PIN_SET);
}

void Warning_Light_Off(void)
{
  HAL_GPIO_WritePin(Warning_Light_GPIO_Port, Warning_Light_Pin, GPIO_PIN_RESET);
}

void Relay_0_On(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_0_GPIO_Port, Relay_Ctrl_0_Pin, GPIO_PIN_SET);
}

void Relay_0_Off(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_0_GPIO_Port, Relay_Ctrl_0_Pin, GPIO_PIN_RESET);
}

void Relay_1_On(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_1_GPIO_Port, Relay_Ctrl_1_Pin, GPIO_PIN_SET);
}

void Relay_1_Off(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_1_GPIO_Port, Relay_Ctrl_1_Pin, GPIO_PIN_RESET);
}

void Relay_2_On(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_2_GPIO_Port, Relay_Ctrl_2_Pin, GPIO_PIN_SET);
}

void Relay_2_Off(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_2_GPIO_Port, Relay_Ctrl_2_Pin, GPIO_PIN_RESET);
}

void Relay_3_On(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_3_GPIO_Port, Relay_Ctrl_3_Pin, GPIO_PIN_SET);
}

void Relay_3_Off(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_3_GPIO_Port, Relay_Ctrl_3_Pin, GPIO_PIN_RESET);
}

void Relay_4_On(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_4_GPIO_Port, Relay_Ctrl_4_Pin, GPIO_PIN_SET);
}

void Relay_4_Off(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_4_GPIO_Port, Relay_Ctrl_4_Pin, GPIO_PIN_RESET);
}

void Relay_5_On(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_5_GPIO_Port, Relay_Ctrl_5_Pin, GPIO_PIN_SET);
}

void Relay_5_Off(void)
{
  HAL_GPIO_WritePin(Relay_Ctrl_5_GPIO_Port, Relay_Ctrl_5_Pin, GPIO_PIN_RESET);
}
/* USER CODE END 2 */
