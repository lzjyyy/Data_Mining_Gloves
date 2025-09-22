/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    adc.c
  * @brief   This file provides code for the configuration
  *          of the ADC instances.
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
#include "stm32f4xx.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "adc.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

ADC_HandleTypeDef hadc1;

/* ADC1 init function */
void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* adcHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspInit 0 */

  /* USER CODE END ADC1_MspInit 0 */
    /* ADC1 clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PC3     ------> ADC1_IN13
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    GPIO_InitStruct.Pin = Sys_Ref_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Sys_Ref_GPIO_Port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = Sys_Vadc_Pin|Sys_Iadc_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN ADC1_MspInit 1 */

  /* USER CODE END ADC1_MspInit 1 */
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* adcHandle)
{

  if(adcHandle->Instance==ADC1)
  {
  /* USER CODE BEGIN ADC1_MspDeInit 0 */

  /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PC3     ------> ADC1_IN13
    PB0     ------> ADC1_IN8
    PB1     ------> ADC1_IN9
    */
    HAL_GPIO_DeInit(Sys_Ref_GPIO_Port, Sys_Ref_Pin);

    HAL_GPIO_DeInit(GPIOB, Sys_Vadc_Pin|Sys_Iadc_Pin);

  /* USER CODE BEGIN ADC1_MspDeInit 1 */

  /* USER CODE END ADC1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
static uint16_t adc_current_raw = 0;  
static uint16_t adc_voltage_raw = 0;  
static uint16_t adc_ref_raw = 0;
static float system_voltage = 0.0f;

#define VOLTAGE_TRANS_PARAM		(float)(2.5 * 104.7 / 4.7)

void ADC_ReadValues(void)
{
  HAL_ADC_Start(&hadc1);

  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adc_voltage_raw = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adc_current_raw = HAL_ADC_GetValue(&hadc1);
	
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
  adc_ref_raw = HAL_ADC_GetValue(&hadc1);

  HAL_ADC_Stop(&hadc1);
}

/**
  * @brief get origin ADC voltage value
  */
uint16_t Get_Adc_Voltage_Raw(void)
{
  return adc_voltage_raw;
}

/**
  * @brief get origin ADC current value
  */
uint16_t Get_Adc_Current_Raw(void)
{
  return adc_current_raw;
}

/**
  * @brief get origin ADC ref value
  */
uint16_t Get_Adc_Ref_Raw(void)
{
  return adc_ref_raw;
}

float Get_Sys_Voltage(void)
{
	system_voltage = ((float)adc_voltage_raw  * VOLTAGE_TRANS_PARAM)/ (float)adc_ref_raw;
	return system_voltage;
}

void ADC1_IN9_Init(void) 
{
	// 1. Enable GPIOB clock and set PB1 to analog
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
//	GPIOB->MODER |= GPIO_MODER_MODE1;
//	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD1;
	GPIOB->MODER |= (3 << (1 * 2));
	GPIOB->PUPDR &= ~(3 << (1 * 2));

	// 2. Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	// 3. Configure ADC1
	ADC->CCR = 0;
	
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;
	ADC1->SQR3 = 9; // Channel 9
//	ADC1->SMPR2 |= ADC_SMPR2_SMP9_2;
	ADC1->SMPR2 |= (7 << (3 * 9));                // one sampling time is 480 cycles

	// 4. Enable ADC1
	ADC1->CR2 |= ADC_CR2_ADON;
	for (volatile int i = 0; i < 1000; i++); // short delay
}

void ADC1_IN8_Init(void) 
{
	// 1. Enable GPIOB clock and set PB1 to analog
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
	GPIOB->MODER |= (3 << (0 * 2));
	GPIOB->PUPDR &= ~(3 << (0 * 2));

	// 2. Enable ADC1 clock
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	ADC->CCR = 0;
	
	// 3. Configure ADC1
	ADC1->CR1 = 0;
	ADC1->CR2 = 0;
	ADC1->SQR3 = 8; // Channel 8
//	ADC1->SMPR2 |= ADC_SMPR2_SMP9_2;
	ADC1->SMPR2 |= (7 << (3 * 8));                 // one sampling time is 480 cycles

	// 4. Enable ADC1
	ADC1->CR2 |= ADC_CR2_ADON;
	for (volatile int i = 0; i < 1000; i++); // short delay
}

uint16_t ADC1_Read_IN9(void) 
{
	ADC1->CR2 |= ADC_CR2_SWSTART;             // Start conversion
	while (!(ADC1->SR & ADC_SR_EOC));         // Wait until done
	return ADC1->DR;                           // Return value
}

uint16_t ADC1_Read_IN8(void) 
{
	ADC1->CR2 |= ADC_CR2_SWSTART;             // Start conversion
	while (!(ADC1->SR & ADC_SR_EOC));         // Wait until done
	return ADC1->DR;                           // Return value
}
/* USER CODE END 1 */
