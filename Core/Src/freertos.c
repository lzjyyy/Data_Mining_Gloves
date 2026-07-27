/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Rs485RecvTask.h"
#include "UartDbgTask.h"
#include "LedTask.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for CurrLoopTask */
osThreadId_t CurrLoopTaskHandle;
const osThreadAttr_t CurrLoopTask_attributes = {
  .name = "CurrLoopTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh5,
};
/* Definitions for SpdLoopTask */
osThreadId_t SpdLoopTaskHandle;
const osThreadAttr_t SpdLoopTask_attributes = {
  .name = "SpdLoopTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for PosLoopTask */
osThreadId_t PosLoopTaskHandle;
const osThreadAttr_t PosLoopTask_attributes = {
  .name = "PosLoopTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityHigh3,
};
/* Definitions for CurrSenseTask */
osThreadId_t CurrSenseTaskHandle;
const osThreadAttr_t CurrSenseTask_attributes = {
  .name = "CurrSenseTask",
  .stack_size = 6144 * 4,
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for EncSenseTask */
osThreadId_t EncSenseTaskHandle;
const osThreadAttr_t EncSenseTask_attributes = {
  .name = "EncSenseTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal6,
};
/* Definitions for TempSenseTask */
osThreadId_t TempSenseTaskHandle;
const osThreadAttr_t TempSenseTask_attributes = {
  .name = "TempSenseTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for SysCtrlTask */
osThreadId_t SysCtrlTaskHandle;
const osThreadAttr_t SysCtrlTask_attributes = {
  .name = "SysCtrlTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow4,
};
/* Definitions for Rs485RecvTask */
osThreadId_t Rs485RecvTaskHandle;
const osThreadAttr_t Rs485RecvTask_attributes = {
  .name = "Rs485RecvTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh6,
};
/* Definitions for UartDbgTask */
osThreadId_t UartDbgTaskHandle;
const osThreadAttr_t UartDbgTask_attributes = {
  .name = "UartDbgTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal3,
};
/* Definitions for StoreTask */
osThreadId_t StoreTaskHandle;
const osThreadAttr_t StoreTask_attributes = {
  .name = "StoreTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LaunchTask */
osThreadId_t LaunchTaskHandle;
const osThreadAttr_t LaunchTask_attributes = {
  .name = "LaunchTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityHigh7,
};
/* Definitions for TestTask */
osThreadId_t TestTaskHandle;
const osThreadAttr_t TestTask_attributes = {
  .name = "TestTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for KinSolverTask */
osThreadId_t KinSolverTaskHandle;
const osThreadAttr_t KinSolverTask_attributes = {
  .name = "KinSolverTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for LedTask */
osThreadId_t LedTaskHandle;
const osThreadAttr_t LedTask_attributes = {
  .name = "LedTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for CANFDRecvTask */
osThreadId_t CANFDRecvTaskHandle;
const osThreadAttr_t CANFDRecvTask_attributes = {
  .name = "CANFDRecvTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh4,
};
/* Definitions for Rs485RecvQueue */
osMessageQueueId_t Rs485RecvQueueHandle;
const osMessageQueueAttr_t Rs485RecvQueue_attributes = {
  .name = "Rs485RecvQueue"
};
/* Definitions for TestQueue */
osMessageQueueId_t TestQueueHandle;
const osMessageQueueAttr_t TestQueue_attributes = {
  .name = "TestQueue"
};
/* Definitions for EncSenseQueue */
osMessageQueueId_t EncSenseQueueHandle;
const osMessageQueueAttr_t EncSenseQueue_attributes = {
  .name = "EncSenseQueue"
};
/* Definitions for LaunchEvents */
osEventFlagsId_t LaunchEventsHandle;
const osEventFlagsAttr_t LaunchEvents_attributes = {
  .name = "LaunchEvents"
};
/* Definitions for StoreEvents */
osEventFlagsId_t StoreEventsHandle;
const osEventFlagsAttr_t StoreEvents_attributes = {
  .name = "StoreEvents"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
extern void StartCurrLoopTask(void *argument);
extern void StartSpdLoopTask(void *argument);
extern void StartPosLoopTask(void *argument);
extern void StartCurrSenseTask(void *argument);
extern void StartEncSenseTask(void *argument);
extern void StartTempSenseTask(void *argument);
extern void StartSysCtrlTask(void *argument);
extern void StartRs485RecvTask(void *argument);
extern void StartUartDbgTask(void *argument);
extern void StartStoreTask(void *argument);
extern void StartLaunchTask(void *argument);
extern void StartTestTask(void *argument);
extern void StartKinSolverTask(void *argument);
extern void StartLedTask(void *argument);
extern void StartCANFDRecvTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* Hook prototypes */
void configureTimerForRunTimeStats(void);
unsigned long getRunTimeCounterValue(void);

/* USER CODE BEGIN 1 */
/* Functions needed when configGENERATE_RUN_TIME_STATS is on */
__weak void configureTimerForRunTimeStats(void)
{

}

__weak unsigned long getRunTimeCounterValue(void)
{
return 0;
}
/* USER CODE END 1 */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Rs485RecvQueue */
  Rs485RecvQueueHandle = osMessageQueueNew (6, sizeof(void *), &Rs485RecvQueue_attributes);

  /* creation of TestQueue */
  TestQueueHandle = osMessageQueueNew (5, sizeof(void *), &TestQueue_attributes);

  /* creation of EncSenseQueue */
  EncSenseQueueHandle = osMessageQueueNew (100, sizeof(void *), &EncSenseQueue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of CurrLoopTask */
  CurrLoopTaskHandle = osThreadNew(StartCurrLoopTask, NULL, &CurrLoopTask_attributes);

  /* creation of SpdLoopTask */
  SpdLoopTaskHandle = osThreadNew(StartSpdLoopTask, NULL, &SpdLoopTask_attributes);

  /* creation of PosLoopTask */
  PosLoopTaskHandle = osThreadNew(StartPosLoopTask, NULL, &PosLoopTask_attributes);

  /* creation of CurrSenseTask */
  CurrSenseTaskHandle = osThreadNew(StartCurrSenseTask, NULL, &CurrSenseTask_attributes);

  /* creation of EncSenseTask */
  EncSenseTaskHandle = osThreadNew(StartEncSenseTask, NULL, &EncSenseTask_attributes);

  /* creation of TempSenseTask */
  TempSenseTaskHandle = osThreadNew(StartTempSenseTask, NULL, &TempSenseTask_attributes);

  /* creation of SysCtrlTask */
  SysCtrlTaskHandle = osThreadNew(StartSysCtrlTask, NULL, &SysCtrlTask_attributes);

  /* creation of Rs485RecvTask */
  Rs485RecvTaskHandle = osThreadNew(StartRs485RecvTask, NULL, &Rs485RecvTask_attributes);

  /* creation of UartDbgTask */
  UartDbgTaskHandle = osThreadNew(StartUartDbgTask, NULL, &UartDbgTask_attributes);

  /* creation of StoreTask */
  StoreTaskHandle = osThreadNew(StartStoreTask, NULL, &StoreTask_attributes);

  /* creation of LaunchTask */
  LaunchTaskHandle = osThreadNew(StartLaunchTask, NULL, &LaunchTask_attributes);

  /* creation of TestTask */
  TestTaskHandle = osThreadNew(StartTestTask, NULL, &TestTask_attributes);

  /* creation of KinSolverTask */
  KinSolverTaskHandle = osThreadNew(StartKinSolverTask, NULL, &KinSolverTask_attributes);

  /* creation of LedTask */
  LedTaskHandle = osThreadNew(StartLedTask, NULL, &LedTask_attributes);

  /* creation of CANFDRecvTask */
  CANFDRecvTaskHandle = osThreadNew(StartCANFDRecvTask, NULL, &CANFDRecvTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of LaunchEvents */
  LaunchEventsHandle = osEventFlagsNew(&LaunchEvents_attributes);

  /* creation of StoreEvents */
  StoreEventsHandle = osEventFlagsNew(&StoreEvents_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

