/* USER CODE BEGIN Header */
///**
//  ******************************************************************************
//  * @file    can.c
//  * @brief   This file provides code for the configuration
//  *          of the CAN instances.
//  ******************************************************************************
//  * @attention
//  *
//  * Copyright (c) 2025 STMicroelectronics.
//  * All rights reserved.
//  *
//  * This software is licensed under terms that can be found in the LICENSE file
//  * in the root directory of this software component.
//  * If no LICENSE file comes with this software, it is provided AS-IS.
//  *
//  ******************************************************************************
//  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "canfestival.h"

extern CO_Data Kinco_Ctrl_Data;
extern CO_Data ZeroErr_Ctrl_Data;

/* USER CODE BEGIN 0 */
static CAN_RxHeaderTypeDef CAN1_Test_RxHeader;
static CAN_RxHeaderTypeDef CAN2_Test_RxHeader;
extern uint8_t CAN1_Test_RxData[8];
extern uint8_t CAN2_Test_RxData[8];

#define CAN_RX_BUFFER_SIZE 16

volatile CAN_TempRxMsg can1_temp_rx;
volatile uint8_t can1_rx_ready = 0;

volatile CAN_TempRxMsg can2_temp_rx;
volatile uint8_t can2_rx_ready = 0;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 6;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_7TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_6TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef Filter;
  Filter.FilterIdHigh = 0;
  Filter.FilterIdLow = 0;
  Filter.FilterMaskIdHigh = 0;
  Filter.FilterMaskIdLow = 0;
  Filter.SlaveStartFilterBank = 14;
  Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  Filter.FilterMode = CAN_FILTERMODE_IDMASK;
  Filter.FilterBank = 0;
  Filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  Filter.FilterActivation = CAN_FILTER_ENABLE;

  HAL_CAN_ConfigFilter(&hcan1, &Filter);

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 3;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_11TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  CAN_FilterTypeDef Filter;
  Filter.FilterIdHigh = 0;
  Filter.FilterIdLow = 0;
  Filter.FilterMaskIdHigh = 0;
  Filter.FilterMaskIdLow = 0;
  Filter.SlaveStartFilterBank = 14;
  Filter.FilterScale = CAN_FILTERSCALE_32BIT;
  Filter.FilterMode = CAN_FILTERMODE_IDMASK;
  Filter.FilterBank = 14;
  Filter.FilterFIFOAssignment = CAN_FilterFIFO0;
  Filter.FilterActivation = CAN_FILTER_ENABLE;

  HAL_CAN_ConfigFilter(&hcan2, &Filter);
  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED = 0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = { 0 };
  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspInit 0 */

    /* USER CODE END CAN1_MspInit 0 */
      /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if (HAL_RCC_CAN1_CLK_ENABLED == 1) {
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = CAN1_RX_Pin_Pin | CAN1_TX_Pin_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
    /* USER CODE BEGIN CAN1_MspInit 1 */

    /* USER CODE END CAN1_MspInit 1 */
  }
  else if (canHandle->Instance == CAN2)
  {
    /* USER CODE BEGIN CAN2_MspInit 0 */

    /* USER CODE END CAN2_MspInit 0 */
      /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if (HAL_RCC_CAN1_CLK_ENABLED == 1) {
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = CAN2_RX_Pin_Pin | CAN2_TX_Pin_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
    /* USER CODE BEGIN CAN2_MspInit 1 */

    /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if (canHandle->Instance == CAN1)
  {
    /* USER CODE BEGIN CAN1_MspDeInit 0 */

    /* USER CODE END CAN1_MspDeInit 0 */
      /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if (HAL_RCC_CAN1_CLK_ENABLED == 0) {
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PD0     ------> CAN1_RX
    PD1     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOD, CAN1_RX_Pin_Pin | CAN1_TX_Pin_Pin);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    /* USER CODE BEGIN CAN1_MspDeInit 1 */

    /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if (canHandle->Instance == CAN2)
  {
    /* USER CODE BEGIN CAN2_MspDeInit 0 */

    /* USER CODE END CAN2_MspDeInit 0 */
      /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if (HAL_RCC_CAN1_CLK_ENABLED == 0) {
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB5     ------> CAN2_RX
    PB6     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, CAN2_RX_Pin_Pin | CAN2_TX_Pin_Pin);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
    /* USER CODE BEGIN CAN2_MspDeInit 1 */

    /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  if (hcan->Instance == CAN1)
  {
    CAN_RxHeaderTypeDef RxHeader = { 0 };
    uint8_t RxData[8] = { 0 };
    Message rxm = { 0 };

    // 读取 FIFO0 中消息
    if (HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      // 接收错误处理
      return;
    }

    // 丢弃扩展帧
    if (RxHeader.IDE == CAN_ID_EXT)
      return;

    rxm.cob_id = RxHeader.StdId;
    rxm.rtr = (RxHeader.RTR == CAN_RTR_REMOTE) ? 1 : 0;
    rxm.len = RxHeader.DLC;
    memcpy(rxm.data, RxData, RxHeader.DLC);
    // 保存到临时变量，主循环打印
    can1_temp_rx.StdId = RxHeader.StdId;
    can1_temp_rx.DLC = RxHeader.DLC;
    memcpy(can1_temp_rx.Data, RxData, RxHeader.DLC);
    can1_rx_ready = 1;

    // 调用 CANopen 分发函数
    canDispatch(&Kinco_Ctrl_Data, &rxm);
  }
  else if (hcan->Instance == CAN2)
  {
    CAN_RxHeaderTypeDef RxHeader = { 0 };
    uint8_t RxData[8] = { 0 };
    Message rxm = { 0 };

    // 读取 FIFO0 中消息
    if (HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
    {
      // 接收错误处理
      return;
    }

    // 丢弃扩展帧
    if (RxHeader.IDE == CAN_ID_EXT)
      return;

    rxm.cob_id = RxHeader.StdId;
    rxm.rtr = (RxHeader.RTR == CAN_RTR_REMOTE) ? 1 : 0;
    rxm.len = RxHeader.DLC;
    memcpy(rxm.data, RxData, RxHeader.DLC);
    // 保存到临时变量，主循环打印
    can2_temp_rx.StdId = RxHeader.StdId;
    can2_temp_rx.DLC = RxHeader.DLC;
    memcpy(can2_temp_rx.Data, RxData, RxHeader.DLC);
    can2_rx_ready = 1;

    // 调用 CANopen 分发函数
    canDispatch(&ZeroErr_Ctrl_Data, &rxm);
  }
}

void print_can1_recv_msg(void)
{
  if (can1_rx_ready)
  {
    printf("CAN1 RX: ID=0x%03X DLC=%d Data=", can1_temp_rx.StdId, can1_temp_rx.DLC);
    for (int i = 0; i < can1_temp_rx.DLC; i++)
      printf("%02X ", can1_temp_rx.Data[i]);
    printf("\r\n");
    can1_rx_ready = 0;
  }
}

void print_can2_recv_msg(void)
{
  if (can2_rx_ready)
  {
    printf("CAN2 RX: ID=0x%03X DLC=%d Data=", can2_temp_rx.StdId, can2_temp_rx.DLC);
    for (int i = 0; i < can2_temp_rx.DLC; i++)
      printf("%02X ", can2_temp_rx.Data[i]);
    printf("\r\n");
    can2_rx_ready = 0;
  }
}
/* USER CODE END 1 */
