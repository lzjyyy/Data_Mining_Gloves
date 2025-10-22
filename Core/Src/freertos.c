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
#include <stdio.h>
#include <string.h>
#include "gpio.h"
#include "usart.h"
#include "adc.h"
#include "rs485.h"
#include "w5500_port.h"
#include "MQTTClient.h"
#include "mqtt_network.h"
#include "mqtt_timer.h"
#include "can.h"
#include "cJSON.h"
#include "gripper.h"
#include "Modbus.h"
#include "rs485.h"
#include "Kinco_Ctrl.h"
#include "canfestival.h"
#include "timer5.h"
#include "servo.h"
#include "can_canopen.h"
#include "FreeRTOS.h"
#include "timers.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  int mode; // 0:common mode, 1:remote mode
  struct {
    int position;
    int speed;
    int torque;
  } left, right;
} GripperCmd_t;

typedef struct {
  uint8_t side;
  uint32_t position;
  uint16_t reached;
  uint16_t warning;
} GripperStatus_t;

typedef struct {
  struct {
    int is_enable;
    int position;
    int velocity;
  }kinco, zeroerr;
} ServoCmd_t;

typedef struct {
  uint16_t status_word;
  uint16_t error_code;
  int32_t position;
  int32_t velocity;
} ServoStatus_t;

typedef struct {
  int type;
  int state;
} GPIOCmd_t;

typedef struct {
  char topic[64];
  char payload[128];
} MqttMsg_t; // message to main controller

typedef enum {
  HOLD_ON = 0,
  TURN_UP = 1,
  TURN_DOWN = 2,
} Lifting_Mode_t;

typedef enum {
  LIGHT_OFF = 0,
  LIGHT_ON = 1,
} Warning_Mode_t;

typedef enum {
  LIFTING = 0,
  WARNING = 1,
} GPIO_Cmd_Type_t;

typedef struct {
  GripperStatus_t left_gripper_status;
  GripperStatus_t right_gripper_status;
  ServoStatus_t sys_kinco_status;
  ServoStatus_t sys_zeroerr_status;
  bool gpio_in_status[6];
} SysStatus_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CMD_QUEUE_SIZE    16
#define STATUS_QUEUE_SIZE 16
#define GRIPPER_SLAVE_ID   1
#define LEFT_GRIPPER       0
#define RIGHT_GRIPPER      1
#define MODBUS_FUN_READ_REGISTER  3
#define MODBUS_FUN_WRITE_REGISTER 6
#define MODBUS_FUN_WRITE_REGISTERS 16
#define STATUS_QUERY_PERIOD_MS 100     // 查询状�?�周�??
#define STATUS_BLOCK_AFTER_CMD 200     // 写命令后屏蔽状�?�查询时�??
#define USE_TEST_TASKS
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern uint8_t W5500_Init_Status;
extern uint8_t g_uart_rx_buf[UART_RECV_LEN];
extern uint16_t g_uart_rx_sta;
extern IWDG_HandleTypeDef hiwdg;
static const char recv_msg[] = "RS232 received msg:\r\n";
static const char end_msg[] = "\r\n";
char rs485a_test_msg[] = "RS485A transmit test.\r\n";
char rs485b_test_msg[] = "RS485B transmit test.\r\n";
char rs485c_test_msg[] = "RS485C transmit test.\r\n";

osThreadId mqttTestTaskHandle;
osThreadId_t modbusMasterTestHandle;
osThreadId_t kincoCtrlTaskHandle;
osThreadId_t zeroerrCtrlTaskHandle;

/* Task handles */
osThreadId_t mqttTaskHandle;
osThreadId_t monitorTaskHandle;
osThreadId_t leftGripperTaskHandle;
osThreadId_t rightGripperTaskHandle;
osThreadId_t gpioTaskHandle;
osThreadId_t monitorTaskHandle;

/* Message queues */
osMessageQId leftGripperQueueHandle;     /* grippers control commands from MQTT */
osMessageQId rightGripperQueueHandle;
osMessageQId statusQueueHandle;
osMessageQId gpioQueueHandle;           /* GPIOs control commands from MQTT */
osMessageQId kincoQueueHandle;          /* kinco control commands from MQTT */
osMessageQId zeroerrQueueHanle;         /* zeroerr control commands from MQTT */

// grippers semaphores
osSemaphoreId_t semQueueLeftHandle;
osSemaphoreId_t semStatusLeftHandle;
osSemaphoreId_t semQueueRightHandle;
osSemaphoreId_t semStatusRightHandle;

// grippers timers
osTimerId_t timer10msLeft;
osTimerId_t timer200msLeft;
osTimerId_t timer10msRight;
osTimerId_t timer200msRight;

/* MQTT globals */
static Network mqttNet;
static MQTTClient mqttClient;
static unsigned char mqttSendBuf[512];
static unsigned char mqttReadBuf[512];
static volatile int mqtt_connected = 0;

// osThreadId rs485TaskHandle;
// osThreadId statusTaskHandle;
/* Definitions for modbusMasterTestTask */
modbusHandler_t L_ModbusH;
modbusHandler_t R_ModbusH;
static uint16_t LeftGripperCmdData[8];
static uint16_t RightGripperCmdData[8];
static modbus_t LeftGripperTelegram;
static modbus_t RightGripperTelegram;
static uint16_t ModbusDATA[8];
static modbus_t telegram[2];
static modbusRecvRawData_t* p_leftRecvRawData = NULL;
static modbusRecvRawData_t* p_rightRecvRawData = NULL;
static bool is_leftRecvRawDataPtr_ok = false;
static bool is_rightRecvRawDataPtr_ok = false;
static bool gpio_in_status[6] = { false, false, false, false, false, false };
static ServoStatus_t kinco_status;
static ServoStatus_t zeroerr_status;
static uint32_t sys_run_cnt = 0;
static uint16_t gripper_err_cnt = 0;
static uint16_t mqtt_err_cnt = 0;
static uint16_t servo_error_cnt = 0;
static SysStatus_t sys_status;
static uint16_t kinco_error_code = 0;
static uint16_t zeroerr_error_code = 0;
static int32_t kinco_actual_vel = 0;
static uint32_t zeroerr_actual_vel = 0;
static const char* sys_status_topic = "robot/status";
static char sys_status_payload[512];  // 足够容纳 JSON
volatile uint32_t idle_counter = 0; // 负载监控用
volatile uint32_t idle_counter_max = 0; // 负载监控用

const osThreadAttr_t modbusMasterTestTask_attributes = {
  .name = "modbusMasterTestTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t kincoCtrlTask_attributes = {
  .name = "kincoCtrlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t zeroerrCtrlTask_attributes = {
  .name = "zeroerrCtrlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t mqttTask_attributes = {
  .name = "mqttTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t leftGripperTask_attributes = {
  .name = "leftGripperTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

const osThreadAttr_t rightGripperTask_attributes = {
  .name = "rightGripperTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t)osPriorityAboveNormal,
};

const osThreadAttr_t gpioTask_attributes = {
  .name = "gpioTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};

const osThreadAttr_t monitorTask_attributes = {
  .name = "monitorTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityLow,
};

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for RS232TestTask */
osThreadId_t RS232TestTaskHandle;
const osThreadAttr_t RS232TestTask_attributes = {
  .name = "RS232TestTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityLow,
};
/* Definitions for RS485TestTask */
osThreadId_t RS485TestTaskHandle;
const osThreadAttr_t RS485TestTask_attributes = {
  .name = "RS485TestTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t)osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void StartMQTTTestTask(void const* argument);
void StartKincoCtrlTask(void const* argument);
void StartZeroErrCtrlTask(void const* argument);
void StartMqttTask(void const* argument);
void StartMonitorTask(void const* argument);
void StartMonitorUpdateTask(void* argument);
void StartModbusMasterTestTask(void* argument);

void MX_Modbus_Init(void);
void LeftGripperTask(void* argument);
void RightGripperTask(void* argument);
void GpioTask(void* argument);
void cjson_memory_hook(void);
static void messageArrived(MessageData* data);
static void gripper_execute(modbusHandler_t* h, int position, int speed, int torque, uint8_t select_gripper);
static bool mqtt_publish_gripper_status(GripperStatus_t* status, uint8_t side);
static void gripper_get_status(modbusHandler_t* h, GripperStatus_t* status, uint8_t side);
static bool get_real_pos(modbusHandler_t* h, uint32_t* p_pos, uint8_t side);
static bool get_reached(modbusHandler_t* h, uint16_t* p_reached, uint8_t side);
static bool get_warning_info(modbusHandler_t* h, uint16_t* p_warning, uint8_t side);
static bool mqtt_publish_gpio_status(void);
static bool mqtt_publish_servos_status(uint8_t type);
static void system_reset(void);
static void ForceCloseSocket(uint8_t sn);
static bool mqtt_publish_sys_status(SysStatus_t* status);
static void mqtt_subscribe_all(void);

// left and right grippers timer callback functions
static void Timer10msLeft_Callback(void* argument);
static void Timer200msLeft_Callback(void* argument);
static void Timer10msRight_Callback(void* argument);
static void Timer200msRight_Callback(void* argument);

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void* argument);
void StartRS232TestTask(void* argument);
void StartRS485TestTask(void* argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  cjson_memory_hook();

  TIM5_Init();
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  // left gripper semaphores
  semQueueLeftHandle = osSemaphoreNew(1, 0, NULL);
  semStatusLeftHandle = osSemaphoreNew(1, 0, NULL);

  // right gripper semaphores
  semQueueRightHandle = osSemaphoreNew(1, 0, NULL);
  semStatusRightHandle = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  // left gripper timers
  timer10msLeft = osTimerNew(Timer10msLeft_Callback, osTimerPeriodic, NULL, NULL);
  timer200msLeft = osTimerNew(Timer200msLeft_Callback, osTimerPeriodic, NULL, NULL);

  timer10msRight = osTimerNew(Timer10msRight_Callback, osTimerPeriodic, NULL, NULL);
  timer200msRight = osTimerNew(Timer200msRight_Callback, osTimerPeriodic, NULL, NULL);
  // right gripper timers

  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  leftGripperQueueHandle = osMessageQueueNew(10, sizeof(GripperCmd_t), NULL);
  rightGripperQueueHandle = osMessageQueueNew(10, sizeof(GripperCmd_t), NULL);
  statusQueueHandle = osMessageQueueNew(10, sizeof(GripperStatus_t), NULL);

  kincoQueueHandle = osMessageQueueNew(10, sizeof(ServoCmd_t), NULL);
  zeroerrQueueHanle = osMessageQueueNew(10, sizeof(ServoCmd_t), NULL);

  gpioQueueHandle = osMessageQueueNew(10, sizeof(GPIOCmd_t), NULL);

  // start timers
  osTimerStart(timer10msLeft, 10);
  osTimerStart(timer200msLeft, 200);

  osTimerStart(timer10msRight, 10);
  osTimerStart(timer200msRight, 200);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  // defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of RS232TestTask */
  // RS232TestTaskHandle = osThreadNew(StartRS232TestTask, NULL, &RS232TestTask_attributes);

  /* creation of RS485TestTask */
  // RS485TestTaskHandle = osThreadNew(StartRS485TestTask, NULL, &RS485TestTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  // osThreadDef(MQTTTestTask, StartMQTTTestTask, osPriorityNormal, 0, 512);
  // mqttTestTaskHandle = osThreadCreate(osThread(MQTTTestTask), NULL);

  kincoCtrlTaskHandle = osThreadNew(StartKincoCtrlTask, NULL, &kincoCtrlTask_attributes);

  zeroerrCtrlTaskHandle = osThreadNew(StartZeroErrCtrlTask, NULL, &zeroerrCtrlTask_attributes);

  // modbusMasterTestHandle = osThreadNew(StartModbusMasterTestTask, NULL, &modbusMasterTestTask_attributes);

  mqttTaskHandle = osThreadNew(StartMqttTask, NULL, &mqttTask_attributes);

  leftGripperTaskHandle = osThreadNew(LeftGripperTask, NULL, &leftGripperTask_attributes);

  rightGripperTaskHandle = osThreadNew(RightGripperTask, NULL, &rightGripperTask_attributes);

  gpioTaskHandle = osThreadNew(GpioTask, NULL, &gpioTask_attributes);

  // monitorTaskHandle = osThreadNew(StartMonitorTask, NULL, &monitorTask_attributes);
  monitorTaskHandle = osThreadNew(StartMonitorUpdateTask, NULL, &monitorTask_attributes);
  /* USER CODE END RTOS_THREADS */

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
#define ECHO_SOCK 0
#define ECHO_PORT 5000
#define ECHO_BUF_SIZE 1024
  /* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void* argument)
{
  /* USER CODE BEGIN StartDefaultTask */

  /* Infinite loop */
  for (;;)
  {
    printf("Default task is running...\r\n");
    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

#ifdef USE_TEST_TASKS
/* USER CODE BEGIN Header_StartRS232TestTask */
/**
* @brief Function implementing the RS232TestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRS232TestTask */
void StartRS232TestTask(void* argument)
{
  /* USER CODE BEGIN StartRS232TestTask */
  uint8_t len = 0;
  HAL_StatusTypeDef send_result = 0;
  /* Infinite loop */
  for (;;)
  {
    printf("RS232 task is running...\r\n");
    Board_Led0_Off();
    if (g_uart_rx_sta & 0x8000)
    {
      Board_Led0_On();
      send_result = UART5_Send((uint8_t*)recv_msg, strlen(recv_msg));
      printf("RS232 send_result:%d\r\n", send_result);
      len = g_uart_rx_sta & 0x3FFF;
      HAL_UART_Transmit(&huart5, (uint8_t*)g_uart_rx_buf, len, 1000);
      while (__HAL_UART_GET_FLAG(&huart5, UART_FLAG_TC) != SET)
      {
        ;
      }
      g_uart_rx_sta = 0;
      UART5_Send((uint8_t*)end_msg, strlen(end_msg));
    }
    osDelay(10);
  }
  /* USER CODE END StartRS232TestTask */
}

/* USER CODE BEGIN Header_StartRS485TestTask */
/**
* @brief Function implementing the RS485TestTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRS485TestTask */
void StartRS485TestTask(void* argument)
{
  /* USER CODE BEGIN StartRS485TestTask */
  /* Infinite loop */
  HAL_StatusTypeDef send_result = 0;
  for (;;)
  {
    printf("RS485 task is running...\r\n");
    send_result = RS485_Send(RS485A_CH, rs485a_test_msg, strlen((char*)rs485a_test_msg), 1000);
    if (send_result == HAL_OK) {
      printf("RS485A_CH send OK!\r\n");
    }
    send_result = RS485_Send(RS485B_CH, rs485b_test_msg, strlen((char*)rs485b_test_msg), 1000);
    if (send_result == HAL_OK) {
      printf("RS485B_CH send OK!\r\n");
    }
    send_result = RS485_Send(RS485C_CH, rs485c_test_msg, strlen((char*)rs485c_test_msg), 1000);
    if (send_result == HAL_OK) {
      printf("RS485C_CH send OK!\r\n");
    }
    osDelay(100);
  }
  /* USER CODE END StartRS485TestTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void StartMQTTTestTask(void const* argument)
{
  Network n;
  MQTTClient c;
  unsigned char sendbuf[128], readbuf[128];
  int rc;

  // uint8_t broker_ip[4] = { 192,168,1,100 };
  // uint16_t broker_port = 1883;
  // int ret = W5500_TCP_Connect_Debug(0, broker_ip, broker_port, 5000);

  // if (ret == 0) {
  //   printf("Connected to broker!\r\n");
  // }
  // else {
  //   printf("Failed to connect, ret=%d\r\n", ret);
  // }
  if (get_w5500_init_status() != 1)
  {
    printf("W5500 init failed,stop StartMQTTTestTask.\r\n");
    vTaskDelete(NULL);   // delete self
  }

  NetworkInit(&n);
  int conn_result = 0;
  conn_result = NetworkConnect(&n, "192.168.1.10", 1883);
  if (conn_result != 0) {
    printf("MQTT Network Connect failed,result:%d\r\n", conn_result);
    vTaskDelete(NULL);
  }

  MQTTClientInit(&c, &n, 1000, sendbuf, sizeof(sendbuf), readbuf, sizeof(readbuf));

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.MQTTVersion = 4;
  data.clientID.cstring = "STM32_Client";

  if ((rc = MQTTConnect(&c, &data)) != 0) {
    printf("MQTT Connect failed, rc=%d\r\n", rc);
    vTaskDelete(NULL);
  }
  printf("MQTT Connected!\r\n");

  while (1)
  {
    MQTTYield(&c, 1000);
    osDelay(10);
  }
}

void StartKincoCtrlTask(void const* argument)
{
  if (can1Init(&Kinco_Ctrl_Data, 500000) != true)
  {
    printf("CAN1 init failed\r\n");
    vTaskDelete(NULL);
  }

  Kinco_MasterNode_Init();

  Kinco_Setup();

  stopSYNC(&Kinco_Ctrl_Data);

  masterSendNMTstateChange(&Kinco_Ctrl_Data, 0x01, NMT_Start_Node);

  ServoCmd_t cmd;
  TickType_t lastWakeTime = xTaskGetTickCount(); // 记录当前tick

  for (;;)
  {
    if (osMessageQueueGet(kincoQueueHandle, &cmd, NULL, 0) == osOK) {
      if ((cmd.kinco.is_enable == 1) && (Get_Curr_Status(0) != OPERATION_ENABLED))
      {
        // Kinco_Enable_PDO();
        int enable_result = 0;
        for (int i = 0; i < 10; i++) {
          enable_result = Kinco_Enable_PDO();
          if (enable_result == ENABLE_OK)
          {
            break;
          }
          osDelay(10);
        }

        if (enable_result == ENABLE_OK)
        {
          printf("Kinco ENABLE_OK\r\n");
        }
        else
        {
          printf("Kinco ENABLE_FAILED\r\n");
        }
      }

      if ((cmd.kinco.is_enable == 1) && (cmd.kinco.position != 0xFFFF))
      {
        Kinco_MovPos_PDO(cmd.kinco.position);
      }

      if (cmd.kinco.velocity > 0 && cmd.kinco.velocity < 3000)
      {
        Kinco_SetVel_PDO(cmd.kinco.velocity);
      }

      if ((cmd.kinco.is_enable == 0) && (Get_Curr_Status(0) == OPERATION_ENABLED))
      {
        printf("Disable kinco\r\n");
        Kinco_Disable_PDO();
      }
    }

    /* 每隔 50ms 读取kinco状态和位置 */
    static uint32_t counter = 0;
    if (counter % 5 == 0) {  // 系统tick=10ms
      sendSYNC(&Kinco_Ctrl_Data);
      // printf("Kinco Status=0x%04X, Pos=%ld\r\n", Statusword, Position_actual_value);
      // kinco_status.status_word = Statusword;
      // kinco_status.position = Position_actual_value;
      sys_status.sys_kinco_status.status_word = Statusword;
      sys_status.sys_kinco_status.position = Position_actual_value;
    }

    if (counter % 50 == 0) {
      if (Kinco_Read_ActuclVel_SDO(&kinco_actual_vel) != 0)
      {
        kinco_actual_vel = 0xFFFF;
        servo_error_cnt++;
      }
      sys_status.sys_kinco_status.velocity = kinco_actual_vel;
      // printf("Kinco actual_vel is:0x%x\r\n", kinco_actual_vel);
    }

    if (counter % 500 == 0) {
      if (Kinco_Read_Error_SDO(&kinco_error_code) != 0)
      {
        kinco_error_code = 0xFFFF;
      }
      if (kinco_error_code != 0)
      {
        printf("Kinco error_code is:0x%x\r\n", kinco_error_code);
      }
      sys_status.sys_kinco_status.error_code = kinco_error_code;
    }

    counter++;

    /* 精确定时，每次循环维持10ms周期 */
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));
  }
}

void StartZeroErrCtrlTask(void const* argument)
{
  if (can2Init(&ZeroErr_Ctrl_Data, 1000000) != true)
  {
    printf("CAN2 init failed\r\n");
    vTaskDelete(NULL);
  }

  osDelay(3000);

  ZeroErr_MasterNode_Init();

  ZeroErr_Setup();

  stopSYNC(&ZeroErr_Ctrl_Data);

  masterSendNMTstateChange(&ZeroErr_Ctrl_Data, 0x01, NMT_Start_Node);

  ServoCmd_t cmd;
  TickType_t lastWakeTime = xTaskGetTickCount(); // 记录当前tick
  for (;;)
  {
    if (osMessageQueueGet(zeroerrQueueHanle, &cmd, NULL, 0) == osOK) {
      if ((cmd.zeroerr.is_enable == 1) && (Get_Curr_Status(1) != OPERATION_ENABLED))
      {
        // ZeroErr_QuickStop_Resume_SDO();
        // ZeroErr_Enable_PDO();
        int enable_result = 0;
        for (int i = 0; i < 10; i++) {
          enable_result = ZeroErr_Enable_PDO();
          if (enable_result == ENABLE_OK)
          {
            break;
          }
        }

        if (enable_result == ENABLE_OK)
        {
          printf("ZeroErr ENABLE_OK\r\n");
        }
        else
        {
          printf("ZeroErr ENABLE_FAILED\r\n");
        }
      }

      if (cmd.zeroerr.velocity > 0 && cmd.zeroerr.velocity <= 30)
      {
        // ZeroErr_SetVel_SDO(cmd.zeroerr.velocity);
        ZeroErr_SetVel_PDO(cmd.zeroerr.velocity);
      }

      if ((cmd.zeroerr.is_enable == 1) && (cmd.zeroerr.position != 0xFFFF))
      {
        ZeroErr_MovPos_PDO(cmd.zeroerr.position);
      }

      if (cmd.zeroerr.is_enable == 0)
      {
        printf("Disable zeroerr\r\n");
        ZeroErr_QuickStop_SDO();
        ZeroErr_Disable_PDO();
      }

    }

    /* 每隔 50ms 读取ZeroErr状态和位置 */
    static uint32_t counter = 0;
    if (counter % 5 == 0) {  // 系统tick=1ms
      sendSYNC(&ZeroErr_Ctrl_Data);
      // printf("ZeroErr Status=0x%04X, Pos=%ld\r\n", status_word_zeroerr, pos_actual_val_zeroerr);
      // zeroerr_status.status_word = status_word_zeroerr;
      // zeroerr_status.position = pos_actual_val_zeroerr;
      sys_status.sys_zeroerr_status.status_word = status_word_zeroerr;
      sys_status.sys_zeroerr_status.position = pos_actual_val_zeroerr;
    }

    if (counter % 50 == 0) {
      if (ZeroErr_Read_ActuclVel_SDO(&zeroerr_actual_vel) != 0)
      {
        zeroerr_actual_vel = 0xFFFF;
        servo_error_cnt++;
      }
      // printf("Zeroerr actual_vel is:0x%x\r\n", zeroerr_actual_vel);
      sys_status.sys_zeroerr_status.velocity = zeroerr_actual_vel;
    }

    if (counter % 500 == 0) {
      if (ZeroErr_Read_Error_SDO(&zeroerr_error_code) != 0)
      {
        zeroerr_error_code = 0xFFFF;
      }
      if (zeroerr_error_code != 0)
      {
        printf("Zeroerr error_code is:0x%x\r\n", zeroerr_error_code);
      }
      sys_status.sys_zeroerr_status.error_code = zeroerr_error_code;
    }

    counter++;

    /* 精确定时，每次循环维持50ms周期 */
    vTaskDelayUntil(&lastWakeTime, pdMS_TO_TICKS(10));  // 1ms
    // osDelay(50);
  }
}

/* USER CODE BEGIN Header_StartTaskMaster */
/**
* @brief Function implementing the myTaskMaster thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskMaster */
void StartModbusMasterTestTask(void* argument)
{
  /* USER CODE BEGIN StartTaskMaster */
  /* Infinite loop */
  uint32_t u32NotificationValue;
  int32_t pos = 9000;
  uint16_t speed = 100;
  uint16_t torque = 100;
  GripperStatus_t left_status;
  GripperStatus_t right_status;

  for (;;)
  {
    pos = 9000;
    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = 16; // function code (this one is registers read)
    telegram[0].u16RegAdd = REG_POS_HIGH; // start address in slave
    telegram[0].u16CoilsNo = 4; // number of elements (coils or registers) to read

    ModbusDATA[0] = (uint16_t)((pos >> 16) & 0xFFFF);
    ModbusDATA[1] = (uint16_t)(pos & 0xFFFF);
    ModbusDATA[2] = speed;
    ModbusDATA[3] = torque;
    telegram[0].u16reg = ModbusDATA; // pointer to a memory array
    printf("Set L gripper params,pos:9000, speed:100, torque:100\r\n");
    ModbusQuery(&L_ModbusH, telegram[0]); // make a query
    // u32NotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // block until query finishes
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); // wait 500ms for the receive response
    if (u32NotificationValue == 0) {
      // timeout, no response
      printf("L gripper modbus query timeout!\r\n");
      // L_ModbusH.i8lastError = ERR_TIME_OUT;
    }
    // else {
    //   printf("L gripper modbus response received.\r\n");
    //   for (int i = 0; i < L_ModbusH.u16regsize; i++) {
    //     printf("Reg[%d] = %u\r\n", i, L_ModbusH.u16regs[i]);
    //   }
    // }

    printf("Set R gripper params,pos:9000, speed:100, torque:100\r\n");
    // ModbusQuery(&R_ModbusH, telegram[0]); // make a query
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); // block until query finishes
    if (u32NotificationValue == 0) {
      // timeout, no response
      printf("R gripper modbus query timeout!\r\n");
      // R_ModbusH.i8lastError = ERR_TIME_OUT;
    }
    // else {
    //   printf("R gripper modbus response received.\r\n");
    //   for (int i = 0; i < R_ModbusH.u16regsize; i++) {
    //     printf("Reg[%d] = %u\r\n", i, R_ModbusH.u16regs[i]);
    //   }
    // }

    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = 6; // function code (this one is registers read)
    telegram[0].u16RegAdd = REG_TRIGGER; // start address in slave
    telegram[0].u16CoilsNo = 1; // number of elements (coils or registers) to read
    ModbusDATA[0] = 1;
    telegram[0].u16reg = ModbusDATA; // pointer to a memory array
    printf("L gripper trigger motion\r\n");
    // ModbusQuery(&L_ModbusH, telegram[0]); // make a query
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); // block until query finishes
    if (u32NotificationValue == 0) {
      // timeout, no response
      printf("L gripper modbus query timeout!\r\n");
      // L_ModbusH.i8lastError = ERR_TIME_OUT;
    }
    // else {
    //   printf("L gripper modbus response received.\r\n");
    //   for (int i = 0; i < L_ModbusH.u16regsize; i++) {
    //     printf("Reg[%d] = %u\r\n", i, L_ModbusH.u16regs[i]);
    //   }
    // }

    printf("R gripper trigger motion\r\n");
    // ModbusQuery(&R_ModbusH, telegram[0]); // make a query
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); // block until query finishes
    if (u32NotificationValue == 0) {
      // timeout, no response
      printf("R gripper modbus query timeout!\r\n");
      // R_ModbusH.i8lastError = ERR_TIME_OUT;
    }
    // else {
    //   printf("R gripper modbus response received.\r\n");
    //   for (int i = 0; i < R_ModbusH.u16regsize; i++) {
    //     printf("Reg[%d] = %u\r\n", i, R_ModbusH.u16regs[i]);
    //   }
    // }
    osDelay(100);

    gripper_get_status(&R_ModbusH, &left_status, LEFT_GRIPPER);

    // gripper_get_status(&R_ModbusH, &right_status, RIGHT_GRIPPER);

    osDelay(50);

    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = 16; // function code (this one is registers read)
    telegram[0].u16RegAdd = REG_POS_HIGH; // start address in slave
    telegram[0].u16CoilsNo = 4; // number of elements (coils or registers) to read
    pos = 0; // open gripper
    ModbusDATA[0] = (uint16_t)((pos >> 16) & 0xFFFF);
    ModbusDATA[1] = (uint16_t)(pos & 0xFFFF);
    ModbusDATA[2] = speed;
    ModbusDATA[3] = torque;
    telegram[0].u16reg = ModbusDATA; // pointer to a memory array
    printf("Set L gripper params,pos:0, speed:100, torque:100\r\n");
    ModbusQuery(&L_ModbusH, telegram[0]); // make a query
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); // block until query finishes
    if (u32NotificationValue == 0) {
      // timeout, no response
      printf("L gripper modbus query timeout!\r\n");
      // L_ModbusH.i8lastError = ERR_TIME_OUT;
    }
    // else {
    //   printf("L gripper modbus response received.\r\n");
    //   for (int i = 0; i < L_ModbusH.u16regsize; i++) {
    //     printf("Reg[%d] = %u\r\n", i, L_ModbusH.u16regs[i]);
    //   }
    // }

    printf("Set R gripper params,pos:0, speed:100, torque:100\r\n");
    ModbusQuery(&R_ModbusH, telegram[0]); // make a query
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); // block until query finishes
    if (u32NotificationValue == 0) {
      // timeout, no response
      printf("R gripper modbus query timeout!\r\n");
      // R_ModbusH.i8lastError = ERR_TIME_OUT;
    }
    // else {
    //   printf("R gripper modbus response received.\r\n");
    //   for (int i = 0; i < R_ModbusH.u16regsize; i++) {
    //     printf("Reg[%d] = %u\r\n", i, R_ModbusH.u16regs[i]);
    //   }
    // }

    telegram[0].u8id = 1; // slave address
    telegram[0].u8fct = 6; // function code (this one is registers read)
    telegram[0].u16RegAdd = REG_TRIGGER; // start address in slave
    telegram[0].u16CoilsNo = 1; // number of elements (coils or registers) to read
    ModbusDATA[0] = 1;
    telegram[0].u16reg = ModbusDATA; // pointer to a memory array
    printf("L gripper trigger motion\r\n");
    ModbusQuery(&L_ModbusH, telegram[0]); // make a query
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); // block until query finishes
    if (u32NotificationValue == 0) {
      // timeout, no response
      printf("L gripper modbus query timeout!\r\n");
      // L_ModbusH.i8lastError = ERR_TIME_OUT;
    }
    // else {
    //   printf("L gripper modbus response received.\r\n");
    //   for (int i = 0; i < L_ModbusH.u16regsize; i++) {
    //     printf("Reg[%d] = %u\r\n", i, L_ModbusH.u16regs[i]);
    //   }
    // }

    printf("R gripper trigger motion\r\n");
    ModbusQuery(&R_ModbusH, telegram[0]); // make a query
    u32NotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)); // block until query finishes
    if (u32NotificationValue == 0) {
      // timeout, no response
      printf("R gripper modbus query timeout!\r\n");
      // R_ModbusH.i8lastError = ERR_TIME_OUT;
    }
    // else {
    //   printf("R gripper modbus response received.\r\n");
    //   for (int i = 0; i < R_ModbusH.u16regsize; i++) {
    //     printf("Reg[%d] = %u\r\n", i, R_ModbusH.u16regs[i]);
    //   }
    // }

    osDelay(50);
  }
  /* USER CODE END StartTaskMaster */
}
#endif
/* cjson memory initialized */
void cjson_memory_hook(void)
{
  cJSON_Hooks hooks;
  hooks.malloc_fn = pvPortMalloc;
  hooks.free_fn = vPortFree;
  cJSON_InitHooks(&hooks);
}

static void ForceCloseSocket(uint8_t sn)
{
  uint8_t status;

  // 先清所有中断标志（避免干扰）
  setSn_IR(sn, 0xFF);

  // 发关闭命令
  setSn_CR(sn, Sn_CR_CLOSE);

  // 最多等待 500ms 等状态切换
  for (int i = 0; i < 50; i++)
  {
    osDelay(10);
    status = getSn_SR(sn);
    if (status == SOCK_CLOSED)
    {
      printf("Socket %d CLOSED\r\n", sn);
      return;
    }
  }

  // 若超时未关闭
  printf("Socket %d close timeout, last state=0x%02X\r\n", sn, status);
}


void StartMqttTask(void const* argument) {
  int rc;

  if (get_w5500_init_status() != 1) {
    printf("W5500 init failed, MQTT task invalid.\r\n");
    // vTaskDelete(NULL);
  }

  NetworkInit(&mqttNet);

reconnect:
  // 初次进入或重连前，确保 socket 被彻底关闭
  ForceCloseSocket(mqttNet.sock);

  // // 等待 socket 变为 CLOSED（避免资源忙）
  // {
  //   const TickType_t wait_start = xTaskGetTickCount();
  //   const TickType_t wait_timeout = pdMS_TO_TICKS(2000); // 最多等 2s
  //   while (getSn_SR(mqttNet.sock) != SOCK_CLOSED) {
  //     if ((xTaskGetTickCount() - wait_start) > wait_timeout) {
  //       // 超时仍未关闭，继续，但打印警告
  //       printf("Warning: socket not closed after 2s, continue to reconnect.\r\n");
  //       break;
  //     }
  //     osDelay(50);
  //   }
  //   // 等一小段时间，给 W5500 状态稳定的机会
  //   osDelay(50);
  // }

  // 等待 PHY link
  while ((W5500_Get_PHYCFGR() & 0x01) == 0) {
    printf("Waiting for PHY Link...\r\n");
    mqtt_err_cnt++;
    osDelay(500);
  }

  // 网络连接（阻塞重试）
  if (NetworkConnect(&mqttNet, "192.168.1.10", 1883) != 0) {
    printf("MQTT Network connect failed, retry in 1s\r\n");
    mqtt_err_cnt += 10;
    int result = W5500_Init();
    if (result != 0) {
      printf("W5500 init failed,result is:%d\r\n", result);
    }
    else
    {
      printf("W5500 init successfully.\r\n");
    }
    osDelay(1000);
    goto reconnect;
  }

  // MQTT 客户端初始化
  MQTTClientInit(&mqttClient, &mqttNet, 2000, mqttSendBuf, sizeof(mqttSendBuf), mqttReadBuf, sizeof(mqttReadBuf));

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.MQTTVersion = 4;
  data.clientID.cstring = "STM32_Client";
  data.keepAliveInterval = 60;   // 增大到 60 秒，降低误判断线概率
  data.cleansession = 1;

  rc = MQTTConnect(&mqttClient, &data);
  if (rc != 0) {
    printf("MQTT Connect failed, rc=%d, retry 1s\r\n", rc);
    osDelay(1000);
    goto reconnect;
  }
  printf("MQTT Connected!\r\n");

  // 订阅主题：逐条检查，便于定位哪条订阅失败
  if ((rc = MQTTSubscribe(&mqttClient, "robot/gpio/cmd", QOS0, messageArrived)) != 0) {
    printf("Subscribe robot/gpio/cmd failed, rc=%d\r\n", rc);
  }
  else {
    printf("Subscribed robot/gpio/cmd\r\n");
  }

  if ((rc = MQTTSubscribe(&mqttClient, "robot/gripper/cmd", QOS0, messageArrived)) != 0) {
    printf("Subscribe robot/gripper/cmd failed, rc=%d\r\n", rc);
  }
  else {
    printf("Subscribed robot/gripper/cmd\r\n");
  }

  if ((rc = MQTTSubscribe(&mqttClient, "robot/servo/cmd", QOS0, messageArrived)) != 0) {
    printf("Subscribe robot/servo/cmd failed, rc=%d\r\n", rc);
  }
  else {
    printf("Subscribed robot/servo/cmd\r\n");
  }

  // 用于断线检测的时间戳：记录上一次成功执行 MQTTYield() 且返回 rc == 0 的时刻
  TickType_t last_success_yield = xTaskGetTickCount();
  const TickType_t yield_timeout_ticks = pdMS_TO_TICKS((data.keepAliveInterval * 1000) * 2); // 两倍 keepalive

  // 发布失败计数，避免一次失败就重连
  int publish_fail_cnt = 0;
  const int publish_fail_threshold = 3;

main_loop:
  for (;;) {
    // 1. 检测 PHY link
    if ((W5500_Get_PHYCFGR() & 0x01) == 0) {
      printf("PHY Link Down, reconnecting...\r\n");
      MQTTDisconnect(&mqttClient);
      NetworkDisconnect(&mqttNet);
      W5500_Init_Status = 0;
      goto reconnect;
    }

    // 2. 检测 Socket 状态
    uint8_t sock_status = getSn_SR(mqttNet.sock);
    if (sock_status != SOCK_ESTABLISHED) {
      printf("Socket lost (status=0x%02X), reconnecting...\r\n", sock_status);
      MQTTDisconnect(&mqttClient);
      NetworkDisconnect(&mqttNet);
      goto reconnect;
    }

    // 3. MQTT 收发（Yield）
    rc = MQTTYield(&mqttClient, 100);  // 100ms
    if (rc < 0) {
      // 严重错误：立即断线重连（并统计）
      printf("MQTTYield returned error rc=%d, reconnecting...\r\n", rc);
      mqtt_err_cnt++;
      MQTTDisconnect(&mqttClient);
      NetworkDisconnect(&mqttNet);
      goto reconnect;
    }
    else {
      // rc == 0 被认为成功（无错误），更新上一次成功时间
      last_success_yield = xTaskGetTickCount();
    }

    // 额外检查 MQTT 是否仍然被认为连接（若使用 Paho 可用此函数）
    if (!MQTTIsConnected(&mqttClient)) {
      printf("MQTTIsConnected returned false, reconnecting...\r\n");
      MQTTDisconnect(&mqttClient);
      NetworkDisconnect(&mqttNet);
      goto reconnect;
    }

    // 如果长时间没有成功的 MQTTYield（例如超过两倍 keepalive），认为死连接，重连
    if ((xTaskGetTickCount() - last_success_yield) > yield_timeout_ticks) {
      printf("No successful MQTTYield for too long, reconnecting...\r\n");
      MQTTDisconnect(&mqttClient);
      NetworkDisconnect(&mqttNet);
      goto reconnect;
    }

    // 4. 发布状态消息（带失败计数）
    if (!mqtt_publish_sys_status(&sys_status)) {
      publish_fail_cnt++;
      printf("Publish robot sys status failed (cnt=%d)\r\n", publish_fail_cnt);
    }
    else {
      publish_fail_cnt = 0;
    }

    // 如果连续多次发布失败，再决定重连（避免抖动导致频繁重连）
    if (publish_fail_cnt >= publish_fail_threshold) {
      printf("Publish failed %d times, reconnecting...\r\n", publish_fail_cnt);
      publish_fail_cnt = 0;
      MQTTDisconnect(&mqttClient);
      NetworkDisconnect(&mqttNet);
      goto reconnect;
    }

    // 5. 小延时，避免任务占满 CPU
    osDelay(10);
  }
}

void messageArrived(MessageData* data)
{
  // printf("Message arrived on topic %.*s\r\n",
  //   data->topicName->lenstring.len,
  //   data->topicName->lenstring.data);

  // printf("Payload: %.*s\r\n",
  //   data->message->payloadlen,
  //   (char*)data->message->payload);

  cJSON* root = cJSON_Parse((char*)data->message->payload);
  if (!root) {
    return;
  }

  /* ================== Gripper 命令解析 ================== */
  GripperCmd_t cmd;
  memset(&cmd, 0, sizeof(cmd));
  cmd.mode = cJSON_GetObjectItem(root, "mode")->valueint;
  // printf("Gripper ctrl mode:%d\r\n", cmd.mode);
  cJSON* left = cJSON_GetObjectItem(root, "left");
  cJSON* right = cJSON_GetObjectItem(root, "right");
  if (left) {
    cmd.left.position = cJSON_GetObjectItem(left, "position")->valueint;
    cmd.left.speed = cJSON_GetObjectItem(left, "speed")->valueint;
    cmd.left.torque = cJSON_GetObjectItem(left, "torque")->valueint;
    // printf("Mqtt msg left gripper info,pos = %d, speed = %d, torque = %d\r\n",
    // cmd.left.position, cmd.left.speed, cmd.left.torque);
    osMessageQueuePut(leftGripperQueueHandle, &cmd, 0, 0);
  }
  if (right) {
    cmd.right.position = cJSON_GetObjectItem(right, "position")->valueint;
    cmd.right.speed = cJSON_GetObjectItem(right, "speed")->valueint;
    cmd.right.torque = cJSON_GetObjectItem(right, "torque")->valueint;
    // printf("Mqtt msg right gripper info,pos = %d, speed = %d, torque = %d\r\n",
    // cmd.right.position, cmd.right.speed, cmd.right.torque);
    osMessageQueuePut(rightGripperQueueHandle, &cmd, 0, 0);
  }

  /* ================== Servo 命令解析 ================== */
  ServoCmd_t servoCmd;
  memset(&servoCmd, 0, sizeof(servoCmd));
  cJSON* kinco = cJSON_GetObjectItem(root, "kinco");
  cJSON* zeroerr = cJSON_GetObjectItem(root, "zeroerr");
  if (kinco) {
    servoCmd.kinco.is_enable = cJSON_GetObjectItem(kinco, "is_enable")->valueint;
    servoCmd.kinco.position = cJSON_GetObjectItem(kinco, "position")->valueint;
    servoCmd.kinco.velocity = cJSON_GetObjectItem(kinco, "velocity")->valueint;
    // printf("Mqtt msg kinco info,is_enable = %d, position = %d, velocity = %d\r\n",
    //   servoCmd.kinco.is_enable, servoCmd.kinco.position, servoCmd.kinco.velocity);
    osMessageQueuePut(kincoQueueHandle, &servoCmd, 0, 0);
  }
  if (zeroerr) {
    servoCmd.zeroerr.is_enable = cJSON_GetObjectItem(zeroerr, "is_enable")->valueint;
    servoCmd.zeroerr.position = cJSON_GetObjectItem(zeroerr, "position")->valueint;
    servoCmd.zeroerr.velocity = cJSON_GetObjectItem(zeroerr, "velocity")->valueint;
    // printf("Mqtt msg zeroerr info,is_enable = %d, position = %d, velocity = %d\r\n",
    //   servoCmd.zeroerr.is_enable, servoCmd.zeroerr.position, servoCmd.zeroerr.velocity);
    osMessageQueuePut(zeroerrQueueHanle, &servoCmd, 0, 0);
  }

  /* ================== GPIO 命令解析 ================== */
  GPIOCmd_t gpioCmd;
  memset(&gpioCmd, 0, sizeof(gpioCmd));

  cJSON* gpio = cJSON_GetObjectItem(root, "gpio");
  if (gpio) {
    cJSON* type_item = cJSON_GetObjectItem(gpio, "type");
    cJSON* state_item = cJSON_GetObjectItem(gpio, "state");

    if (type_item && state_item) {
      gpioCmd.type = type_item->valueint;   // GPIO_Cmd_Type_t
      gpioCmd.state = state_item->valueint;  // Lifting_Mode_t �???????? Warning_Mode_t

      printf("GPIO cmd type=%d, state=%d\r\n", gpioCmd.type, gpioCmd.state);

      osMessageQueuePut(gpioQueueHandle, &gpioCmd, 0, 0);
    }
  }
  cJSON_Delete(root);
}

/* Monitor Task */
/* Monitor Task - System Health Check */
void StartMonitorTask(void const* argument)
{
  static uint32_t last_timer_task_counter = 0;
  static uint32_t timer_stuck_counter = 0;

  for (;;)
  {
    sys_run_cnt++;

    // 打印系统运行时间与错误统计
    printf("sys_run_time: %lu s, gripper_err_cnt: %d, mqtt_err_cnt: %d\r\n",
      sys_run_cnt, gripper_err_cnt, mqtt_err_cnt);

    // 打印定时器任务剩余栈
    UBaseType_t timer_stack_remain = uxTaskGetStackHighWaterMark(xTimerGetTimerDaemonTaskHandle());
    // printf("Timer task stack remaining: %lu\r\n", (unsigned long)timer_stack_remain);

    // 检查 Timer 守护任务是否卡死
    static uint32_t timer_task_counter = 0;
    TaskHandle_t xTimerHandle = xTimerGetTimerDaemonTaskHandle();
    eTaskState timer_state = eTaskGetState(xTimerHandle);

    if (timer_state == eBlocked)
    {
      // Timer task 正常在等待延时中，复位检测计数
      timer_stuck_counter = 0;
    }
    else
    {
      // Timer task 长时间非阻塞状态，可能卡死
      timer_stuck_counter++;
      printf("Warning: Timer task not blocked (%d)\r\n", timer_state);
    }

    // 检测任务运行时间变化（判断是否真的卡死）
    timer_task_counter++;
    if (timer_task_counter == last_timer_task_counter)
    {
      timer_stuck_counter++;
    }
    else
    {
      last_timer_task_counter = timer_task_counter;
      timer_stuck_counter = 0;
    }

    // 如果 Timer 任务连续 5 次（约5秒）未变化，则判断为卡死
    if (timer_stuck_counter > 5)
    {
      printf("Error: Timer task stuck! System reset.\r\n");
      system_reset();
    }

    // 其他错误检测逻辑
    if (gripper_err_cnt >= 30 || mqtt_err_cnt >= 50)
    {
      printf("System Reset due to errors!\r\n");
      system_reset();
    }

    // 看门狗喂狗
    HAL_IWDG_Refresh(&hiwdg);

    osDelay(1000);
  }
}

void MX_Modbus_Init(void)
{
  /* Master initialization */
  L_ModbusH.uModbusType = MB_MASTER;
  L_ModbusH.port = &huart1;
  L_ModbusH.u8id = 0; // For master it must be 0
  L_ModbusH.u16timeOut = 1000;
  L_ModbusH.EN_Port = NULL;
  L_ModbusH.u16regs = ModbusDATA;
  L_ModbusH.u16regsize = sizeof(ModbusDATA) / sizeof(ModbusDATA[0]);
  L_ModbusH.xTypeHW = USART_HW_DMA;

  //Initialize Modbus library
  ModbusInit(&L_ModbusH);
  //Start capturing traffic on serial Port
  ModbusStart(&L_ModbusH);

  R_ModbusH.uModbusType = MB_MASTER;
  R_ModbusH.port = &huart3;
  R_ModbusH.u8id = 0; // For master it must be 0
  R_ModbusH.u16timeOut = 1000;
  R_ModbusH.EN_Port = NULL;
  R_ModbusH.u16regs = ModbusDATA;
  R_ModbusH.u16regsize = sizeof(ModbusDATA) / sizeof(ModbusDATA[0]);
  R_ModbusH.xTypeHW = USART_HW_DMA;
  //Initialize Modbus library
  ModbusInit(&R_ModbusH);
  //Start capturing traffic on serial Port
  ModbusStart(&R_ModbusH);

  p_leftRecvRawData = get_recv_raw_data_addr(&L_ModbusH);
  if (p_leftRecvRawData != NULL)
  {
    is_leftRecvRawDataPtr_ok = true;
  }
  p_rightRecvRawData = get_recv_raw_data_addr(&R_ModbusH);
  if (p_rightRecvRawData != NULL)
  {
    is_rightRecvRawDataPtr_ok = true;
  }
}

static void gripper_execute(modbusHandler_t* h, int position, int speed, int torque, uint8_t select_gripper)
{
  uint32_t notifyVal;
  if (select_gripper == LEFT_GRIPPER) {
    LeftGripperCmdData[0] = (uint16_t)((position >> 16) & 0xFFFF);
    LeftGripperCmdData[1] = (uint16_t)(position & 0xFFFF);
    LeftGripperCmdData[2] = speed;
    LeftGripperCmdData[3] = torque;

    LeftGripperTelegram.u8id = GRIPPER_SLAVE_ID;
    LeftGripperTelegram.u8fct = MODBUS_FUN_WRITE_REGISTERS;
    LeftGripperTelegram.u16RegAdd = REG_POS_HIGH;
    LeftGripperTelegram.u16CoilsNo = 4;
    LeftGripperTelegram.u16reg = LeftGripperCmdData;

    ModbusQuery(h, LeftGripperTelegram);
    notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
    if (notifyVal != ERR_OK_QUERY) {
      // timeout, no response
      printf("Left gripper modbus query timeout!\r\n");
      gripper_err_cnt++;
      // h->i8lastError = ERR_TIME_OUT;
    }

    LeftGripperCmdData[0] = 1;
    LeftGripperTelegram.u8fct = MODBUS_FUN_WRITE_REGISTER;
    LeftGripperTelegram.u16RegAdd = REG_TRIGGER;
    LeftGripperTelegram.u16CoilsNo = 1;
    LeftGripperTelegram.u16reg = LeftGripperCmdData;
    ModbusQuery(h, LeftGripperTelegram);
    notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
    if (notifyVal == 0) {
      // timeout, no response
      printf("Left gripper modbus query timeout!\r\n");
      gripper_err_cnt++;
      // h->i8lastError = ERR_TIME_OUT;
    }
    // else
    // {
    //   printf("Left gripper executed: pos=%d speed=%d torque=%d\r\n", position, speed, torque);
    // }
  }
  else if (select_gripper == RIGHT_GRIPPER) {
    RightGripperCmdData[0] = (uint16_t)((position >> 16) & 0xFFFF);
    RightGripperCmdData[1] = (uint16_t)(position & 0xFFFF);
    RightGripperCmdData[2] = speed;
    RightGripperCmdData[3] = torque;

    RightGripperTelegram.u8id = GRIPPER_SLAVE_ID;
    RightGripperTelegram.u8fct = MODBUS_FUN_WRITE_REGISTERS;
    RightGripperTelegram.u16RegAdd = REG_POS_HIGH;
    RightGripperTelegram.u16CoilsNo = 4;
    RightGripperTelegram.u16reg = RightGripperCmdData;

    ModbusQuery(h, RightGripperTelegram);
    notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
    if (notifyVal == 0) {
      // timeout, no response
      printf("Right gripper modbus query timeout!\r\n");
      gripper_err_cnt++;
      // h->i8lastError = ERR_TIME_OUT;
    }

    RightGripperCmdData[0] = 1;
    RightGripperTelegram.u8fct = MODBUS_FUN_WRITE_REGISTER;
    RightGripperTelegram.u16RegAdd = REG_TRIGGER;
    RightGripperTelegram.u16CoilsNo = 1;
    RightGripperTelegram.u16reg = RightGripperCmdData;
    ModbusQuery(h, RightGripperTelegram);
    notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500));
    if (notifyVal != ERR_OK_QUERY) {
      // timeout, no response
      printf("Right gripper modbus query timeout!\r\n");
      gripper_err_cnt++;
      // h->i8lastError = ERR_TIME_OUT;
    }
    // else
    // {
    //   printf("Right gripper executed: pos=%d speed=%d torque=%d\r\n", position, speed, torque);
    // }
  }
}

bool is_left_get_status = false;
void LeftGripperTask(void* argument)
{
  if (is_leftRecvRawDataPtr_ok == false)
  {
    printf("Acquire leftRecvRawDataPtr failed!\r\n");
    vTaskDelete(NULL);
  }
  GripperCmd_t cmd;
  GripperStatus_t status;
  uint32_t lastStatusTick = 0;   // 上次写命令时间
  uint32_t blockUntil = 0;  // 写命令后屏蔽查询的时间

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    // 1. 处理命令队列
    if (osSemaphoreAcquire(semQueueLeftHandle, 0) == osOK)
    {
      if (osMessageQueueGet(leftGripperQueueHandle, &cmd, NULL, 0) == osOK)
      {
        gripper_execute(&L_ModbusH, cmd.left.position, cmd.left.speed, cmd.left.torque, LEFT_GRIPPER);
        // 写命令后屏蔽状态查询时间
        blockUntil = now + STATUS_BLOCK_AFTER_CMD;
      }
    }

    // 2. 状态查询，间隔控制 + 屏蔽控制
    if (now - lastStatusTick >= STATUS_QUERY_PERIOD_MS && now >= blockUntil)
    {
      if (osSemaphoreAcquire(semStatusLeftHandle, 0) == osOK)
      {
        // is_left_get_status = true;
        // gripper_get_status(&L_ModbusH, &status, LEFT_GRIPPER);
        gripper_get_status(&L_ModbusH, &sys_status.left_gripper_status, LEFT_GRIPPER);
        // is_left_get_status = false;
        // 发布�???????????????? MQTT
        // mqtt_publish_gripper_status(&status, LEFT_GRIPPER);
        // status.side = LEFT_GRIPPER;
        // osMessageQueuePut(statusQueueHandle, &status, 0, 0);
        lastStatusTick = now;
      }
    }
    // else
    // {
    //   printf("left prevent read reg.\r\n");
    // }

    osDelay(10); // 避免空转
  }
}

void RightGripperTask(void* argument)
{
  if (is_rightRecvRawDataPtr_ok == false)
  {
    printf("Acquire rightRecvRawDataPtr failed!\r\n");
    vTaskDelete(NULL);
  }
  GripperCmd_t cmd;
  GripperStatus_t status;
  uint32_t lastStatusTick = 0;
  uint32_t blockUntil = 0;

  for (;;)
  {
    uint32_t now = osKernelGetTickCount();

    // 1. 处理命令队列
    if (osSemaphoreAcquire(semQueueRightHandle, 0) == osOK)
    {
      if (osMessageQueueGet(rightGripperQueueHandle, &cmd, NULL, 0) == osOK)
      {
        gripper_execute(&R_ModbusH, cmd.right.position, cmd.right.speed, cmd.right.torque, RIGHT_GRIPPER);
        // 写命令后屏蔽状�?�查�????????????????
        blockUntil = now + STATUS_BLOCK_AFTER_CMD;
      }
    }

    // 2. 状�?�查询，间隔控制 + 屏蔽控制
    if (now - lastStatusTick >= STATUS_QUERY_PERIOD_MS && now >= blockUntil)
    {
      if (osSemaphoreAcquire(semStatusRightHandle, 0) == osOK)
      {
        while (is_left_get_status == true)
        {
          osDelay(5);
        }
        // gripper_get_status(&R_ModbusH, &status, RIGHT_GRIPPER);
        gripper_get_status(&R_ModbusH, &sys_status.right_gripper_status, RIGHT_GRIPPER);
        // mqtt_publish_gripper_status(&status, RIGHT_GRIPPER);
        // status.side = RIGHT_GRIPPER;
        // osMessageQueuePut(statusQueueHandle, &status, 0, 0);
        lastStatusTick = now;
      }
    }
    // else
    // {
    //   printf("Right prevent read reg.\r\n");
    // }

    osDelay(10);
  }
}


static bool mqtt_publish_gripper_status(GripperStatus_t* status, uint8_t side)
{
  char topic[64];
  char payload[128];

  snprintf(topic, sizeof(topic), "robot/gripper/%s/status", side == LEFT_GRIPPER ? "left" : "right");
  snprintf(payload, sizeof(payload), "{\"pos\":%d,\"reached\":%d,\"warning\":%d}",
    status->position, status->reached, status->warning);

  MQTTMessage message;
  message.qos = QOS0;
  message.retained = 0;
  message.payload = payload;
  message.payloadlen = strlen(payload);

  int rc = MQTTPublish(&mqttClient, topic, &message);
  if (rc != 0) {
    printf("MQTT publish Gripper Status failed, rc=%d\r\n", rc);
    mqtt_err_cnt++;
    return false;
  }
  return true;
}

void gripper_get_status(modbusHandler_t* h, GripperStatus_t* status, uint8_t side)
{
  uint32_t notifyVal;
  bool result = false;
  uint32_t pos = 0;
  uint16_t reached = 0;
  uint16_t warning = 0;
  if (side == LEFT_GRIPPER) {
    status->side = 0;
    // printf("L read POS\r\n");
    result = get_real_pos(h, &pos, LEFT_GRIPPER);
    if (result != true) {
      printf("Read left REG_REALTIME_POS failed!\r\n");
      status->position = 0xFFFFFFFF;
      gripper_err_cnt++;
    }
    else
    {
      // printf("Read left REG_REALTIME_POS:0x%x\r\n", pos);
      status->position = pos;
    }

    // printf("L read REACHED\r\n");
    result = get_reached(h, &reached, LEFT_GRIPPER);
    if (result != true) {
      printf("L read left REG_POS_REACHED failed!\r\n");
      status->reached = 0xFFFF;
      gripper_err_cnt++;
    }
    else
    {
      // printf("L read left REG_POS_REACHED:0x%x\r\n", reached);
      status->reached = reached;
    }

    // printf("L read WARNING\r\n");
    result = get_warning_info(h, &warning, LEFT_GRIPPER);
    if (result != true) {
      printf("L read left REG_WARNING_INFO failed!\r\n");
      status->warning = 0xFFFF;
      gripper_err_cnt++;
    }
    else
    {
      // printf("L read left REG_WARNING_INFO:0x%x\r\n", warning);
      status->warning = warning;
    }
  }
  else {
    status->side = 1;
    // printf("R read POS\r\n");
    result = get_real_pos(h, &pos, RIGHT_GRIPPER);
    if (result != true) {
      printf("R read right REG_REALTIME_POS failed!\r\n");
      status->position = 0xFFFFFFFF;
      gripper_err_cnt++;
    }
    else
    {
      // printf("R read right REG_REALTIME_POS:0x%x\r\n", pos);
      status->position = pos;
    }

    // printf("R read REACHED\r\n");
    result = get_reached(h, &reached, RIGHT_GRIPPER);
    if (result != true) {
      printf("R read right REG_POS_REACHED failed!\r\n");
      status->reached = 0xFFFF;
      gripper_err_cnt++;
    }
    else
    {
      // printf("R read right REG_POS_REACHED:0x%x\r\n", reached);
      status->reached = reached;
    }

    // printf("R read WARNING\r\n");
    result = get_warning_info(h, &warning, RIGHT_GRIPPER);
    if (result != true) {
      printf("R read right REG_WARNING_INFO failed!\r\n");
      status->warning = 0xFFFF;
      gripper_err_cnt++;
    }
    else
    {
      // printf("R read right REG_WARNING_INFO:0x%x\r\n", warning);
      status->warning = warning;
    }
  }
}

// left gripper timers callback
static void Timer10msLeft_Callback(void* argument)
{
  osSemaphoreRelease(semQueueLeftHandle);
}

static void Timer200msLeft_Callback(void* argument)
{
  osSemaphoreRelease(semStatusLeftHandle);
}

// right gripper timers callback
static void Timer10msRight_Callback(void* argument)
{
  osSemaphoreRelease(semQueueRightHandle);
}

static void Timer200msRight_Callback(void* argument)
{
  osSemaphoreRelease(semStatusRightHandle);
}

static bool get_real_pos(modbusHandler_t* h, uint32_t* p_pos, uint8_t side)
{
  uint32_t notifyVal = 0;
  if (side == LEFT_GRIPPER) {
    // printf("Read left REG_REALTIME_POS:\r\n");
    // acquire actual position
    LeftGripperTelegram.u8id = GRIPPER_SLAVE_ID;
    LeftGripperTelegram.u8fct = MODBUS_FUN_READ_REGISTER;
    LeftGripperTelegram.u16RegAdd = REG_REALTIME_POS_HIGH;
    LeftGripperTelegram.u16CoilsNo = 2;

    notifyVal = 0;
    reset_is_fc3_processed(h);
    ModbusQuery(h, LeftGripperTelegram);
    // osDelay(1);
    // uint32_t pos = (uint32_t)(((p_leftRecvRawData->u16RawData[0]) << 16) | p_leftRecvRawData->u16RawData[1]);
    // printf("pos is:0x%x\r\n", pos);
    notifyVal = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notifyVal != ERR_OK_QUERY) {
      // timeout, no response
      printf("L read POS notifyVal:%d\r\n", notifyVal);
      return false;
    }
    else
    {
      if (get_is_fc3_processed(h))
      {
        // printf("Received: ");
        // for (int i = 0; i < p_leftRecvRawData->u8RawDataCnt; i++)
        // {
        //   printf("0x%x ", p_leftRecvRawData->u16RawData[i]);
        // }
        // printf("\r\n");
        *p_pos = (uint32_t)(((p_leftRecvRawData->u16RawData[0]) << 16) | p_leftRecvRawData->u16RawData[1]);
        return true;
      }
      return false;
    }
  }
  else
  {
    // printf("Read right REG_REALTIME_POS:\r\n");
    // acquire actual position
    RightGripperTelegram.u8id = GRIPPER_SLAVE_ID;
    RightGripperTelegram.u8fct = MODBUS_FUN_READ_REGISTER;
    RightGripperTelegram.u16RegAdd = REG_REALTIME_POS_HIGH;
    RightGripperTelegram.u16CoilsNo = 2;

    notifyVal = 0;
    reset_is_fc3_processed(h);
    ModbusQuery(h, RightGripperTelegram);
    notifyVal = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notifyVal != ERR_OK_QUERY) {
      // timeout, no response
      // printf("Left gripper modbus query timeout!\r\n");
      printf("R read POS notifyVal:%d\r\n", notifyVal);
      return false;
    }
    else
    {
      if (get_is_fc3_processed(h))
      {
        // printf("Received: ");
        // for (int i = 0; i < p_rightRecvRawData->u8RawDataCnt; i++)
        // {
        //   printf("0x%x ", p_rightRecvRawData->u16RawData[i]);
        // }
        // printf("\r\n");
        *p_pos = (uint32_t)(((p_rightRecvRawData->u16RawData[0]) << 16) | p_rightRecvRawData->u16RawData[1]);
        return true;
      }
      return false;
    }
  }
}

static bool get_reached(modbusHandler_t* h, uint16_t* p_reached, uint8_t side)
{
  uint32_t notifyVal;
  if (side == LEFT_GRIPPER)
  {
    // printf("Read left REG_POS_REACHED:\r\n");
    LeftGripperTelegram.u8id = GRIPPER_SLAVE_ID;
    LeftGripperTelegram.u8fct = MODBUS_FUN_READ_REGISTER;
    LeftGripperTelegram.u16RegAdd = REG_POS_REACHED;
    LeftGripperTelegram.u16CoilsNo = 1;
    reset_is_fc3_processed(h);
    ModbusQuery(h, LeftGripperTelegram);

    notifyVal = 0;
    notifyVal = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notifyVal != ERR_OK_QUERY) {
      // timeout, no response
      // printf("Left gripper modbus query timeout!\r\n");
      printf("L read REACHED notifyVal:%d\r\n", notifyVal);
      // h->i8lastError = ERR_TIME_OUT;
      return false;
    }
    else
    {
      if (get_is_fc3_processed(h))
      {
        // printf("Received:");
        // for (int i = 0; i < p_leftRecvRawData->u8RawDataCnt; i++)
        // {
        //   printf("0x%x ", p_leftRecvRawData->u16RawData[i]);
        // }
        // printf("\r\n");
        *p_reached = p_leftRecvRawData->u16RawData[0];
        return true;
      }
      return false;
    }
  }
  else
  {
    // printf("Read right REG_POS_REACHED:\r\n");
    RightGripperTelegram.u8id = GRIPPER_SLAVE_ID;
    RightGripperTelegram.u8fct = MODBUS_FUN_READ_REGISTER;
    RightGripperTelegram.u16RegAdd = REG_POS_REACHED;
    RightGripperTelegram.u16CoilsNo = 1;

    notifyVal = 0;
    reset_is_fc3_processed(h);
    ModbusQuery(h, RightGripperTelegram);
    notifyVal = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notifyVal != ERR_OK_QUERY) {
      // timeout, no response
      // printf("Right gripper modbus query timeout!\r\n");
      printf("R read REACHED notifyVal:%d\r\n", notifyVal);
      // h->i8lastError = ERR_TIME_OUT;
      return false;
    }
    else
    {
      if (get_is_fc3_processed(h))
      {
        // printf("Received:");
        // for (int i = 0; i < p_rightRecvRawData->u8RawDataCnt; i++)
        // {
        //   printf("0x%x ", p_rightRecvRawData->u16RawData[i]);
        // }
        // printf("\r\n");
        *p_reached = p_rightRecvRawData->u16RawData[0];
        return true;
      }
      return false;
    }
  }
}

static bool get_warning_info(modbusHandler_t* h, uint16_t* p_warning, uint8_t side)
{
  uint32_t notifyVal;
  if (side == LEFT_GRIPPER)
  {
    // printf("Read left REG_POS_REACHED:\r\n");
    LeftGripperTelegram.u8id = GRIPPER_SLAVE_ID;
    LeftGripperTelegram.u8fct = MODBUS_FUN_READ_REGISTER;
    LeftGripperTelegram.u16RegAdd = REG_WARNING_INFO;
    LeftGripperTelegram.u16CoilsNo = 1;

    notifyVal = 0;
    reset_is_fc3_processed(h);
    ModbusQuery(h, LeftGripperTelegram);
    // notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    notifyVal = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notifyVal != ERR_OK_QUERY) {
      // timeout, no response
      // printf("Left gripper modbus query timeout!\r\n");
      printf("L read WARNING notifyVal:%d\r\n", notifyVal);
      // h->i8lastError = ERR_TIME_OUT;
      return false;
    }
    else
    {
      if (get_is_fc3_processed(h))
      {
        // printf("Received:");
        // for (int i = 0; i < p_leftRecvRawData->u8RawDataCnt; i++)
        // {
        //   printf("0x%x ", p_leftRecvRawData->u16RawData[i]);
        // }
        // printf("\r\n");
        *p_warning = p_leftRecvRawData->u16RawData[0];
        return true;
      }
      return false;
    }
  }
  else
  {
    // printf("Read right REG_POS_REACHED:\r\n");
    RightGripperTelegram.u8id = GRIPPER_SLAVE_ID;
    RightGripperTelegram.u8fct = MODBUS_FUN_READ_REGISTER;
    RightGripperTelegram.u16RegAdd = REG_WARNING_INFO;
    RightGripperTelegram.u16CoilsNo = 1;

    notifyVal = 0;
    reset_is_fc3_processed(h);
    ModbusQuery(h, RightGripperTelegram);
    // notifyVal = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
    notifyVal = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (notifyVal != ERR_OK_QUERY) {
      // timeout, no response
      // printf("Right gripper modbus query timeout!\r\n");
      printf("R read WARNING notifyVal:%d\r\n", notifyVal);
      // h->i8lastError = ERR_TIME_OUT;
      return false;
    }
    else
    {
      if (get_is_fc3_processed(h))
      {
        // printf("Received:");
        // for (int i = 0; i < p_rightRecvRawData->u8RawDataCnt; i++)
        // {
        //   printf("0x%x ", p_rightRecvRawData->u16RawData[i]);
        // }
        // printf("\r\n");
        *p_warning = p_rightRecvRawData->u16RawData[0];
        return true;
      }
      return false;
    }
  }
}

void GpioTask(void* argument)
{
  GPIOCmd_t cmd;
  for (;;) {
    if (osMessageQueueGet(gpioQueueHandle, &cmd, NULL, osWaitForever) == osOK) {
      switch (cmd.type) {
      case LIFTING:
        printf("[GPIO-LIFTING] state=%d\r\n", cmd.state);
        switch (cmd.state) {
        case HOLD_ON:
          printf("Lifting HOLD_ON set\r\n");
          Lift_Hold();
          break;
        case TURN_UP:
          printf("Lifting TURN_UP set\r\n");
          Lift_Up();
          break;
        case TURN_DOWN:
          printf("Lifting TURN_DOWN set\r\n");
          Lift_Down();
          break;
        default:
          break;
        }
        break;

      case WARNING:
        printf("[GPIO-WARNING] state=%d\r\n", cmd.state);
        switch (cmd.state) {
        case LIGHT_ON:
          printf("Waring LIGHT_ON set\r\n");
          Warning_Light_On();
          break;
        case LIGHT_OFF:
          printf("Waring LIGHT_OFF set\r\n");
          Warning_Light_Off();
          break;
        default:
          break;
        }
        break;

      default:
        printf("[GPIO] Unknown type=%d\r\n", cmd.type);
        break;
      }
    }

    // gpio_in_status[0] = Chk_Distance_Reached();
    // gpio_in_status[1] = Chk_UpperLimit_Reached();
    sys_status.gpio_in_status[0] = Chk_Distance_Reached();
    sys_status.gpio_in_status[1] = Chk_UpperLimit_Reached();
    // if (gpio_in_status[1] == true)
    if (sys_status.gpio_in_status[1] == true)
    {
      Lift_Hold();
    }
    // gpio_in_status[2] = Chk_UpperLimit_Reached();
    sys_status.gpio_in_status[2] = Chk_UpperLimit_Reached();
    // if (gpio_in_status[2] == true)
    if (sys_status.gpio_in_status[2] == true)
    {
      Lift_Hold();
    }

    osDelay(50);
  }
}

static bool mqtt_publish_gpio_status(void)
{
  char payload[128];
  snprintf(payload, sizeof(payload),
    "{ \"gpio_in_status\": [%d,%d,%d,%d,%d,%d] }",
    gpio_in_status[0] ? 1 : 0,
    gpio_in_status[1] ? 1 : 0,
    gpio_in_status[2] ? 1 : 0,
    gpio_in_status[3] ? 1 : 0,
    gpio_in_status[4] ? 1 : 0,
    gpio_in_status[5] ? 1 : 0);

  MQTTMessage message;
  message.qos = QOS0;
  message.retained = 0;
  message.payload = (void*)payload;
  message.payloadlen = strlen(payload);

  int rc = MQTTPublish(&mqttClient, "robot/gpio/status", &message);
  if (rc != 0) {
    printf("Publish GPIO status failed, rc=%d\r\n", rc);
    return false;
  }
  else {
    // printf("Publish GPIO status: %s\r\n", payload);
    return true;
  }
}

static bool mqtt_publish_servos_status(uint8_t type)
{
  char topic[64];
  char payload[128];

  snprintf(topic, sizeof(topic), "robot/servos/%s/status", type == 0 ? "kinco" : "zeroerr");
  if (type == 0)
  {
    snprintf(payload, sizeof(payload), "{\"statusWord\":%d,\"position\":%d}",
      kinco_status.status_word, kinco_status.position);
  }
  else
  {
    snprintf(payload, sizeof(payload), "{\"statusWord\":%d,\"position\":%d}",
      zeroerr_status.status_word, zeroerr_status.position);
  }

  MQTTMessage message;
  message.qos = QOS0;
  message.retained = 0;
  message.payload = payload;
  message.payloadlen = strlen(payload);

  int rc = MQTTPublish(&mqttClient, topic, &message);
  if (rc != 0) {
    printf("MQTT publish Servo Status failed, rc=%d\r\n", rc);
    return false;
  }
  return true;
}

static void system_reset(void)
{
  __disable_irq();          // 可选：先关闭全局中断，避免中途打断
  NVIC_SystemReset();       // 调用 Cortex-M4 内核提供的系统复位函
}

static bool mqtt_publish_sys_status(SysStatus_t* status)
{
  if (!status) return false;

  // 生成 JSON payload
  int len = snprintf(sys_status_payload, sizeof(sys_status_payload),
    "{"
    "\"left_gripper\":{\"pos\":%u,\"reached\":%u,\"warning\":%u},"
    "\"right_gripper\":{\"pos\":%u,\"reached\":%u,\"warning\":%u},"
    "\"sys_kinco\":{\"status_word\":%u,\"error_code\":%u,\"position\":%ld,\"velocity\":%ld},"
    "\"sys_zeroerr\":{\"status_word\":%u,\"error_code\":%u,\"position\":%ld,\"velocity\":%ld},"
    "\"gpio_in\":[%d,%d,%d,%d,%d,%d]"
    "}",
    status->left_gripper_status.position,
    status->left_gripper_status.reached,
    status->left_gripper_status.warning,
    status->right_gripper_status.position,
    status->right_gripper_status.reached,
    status->right_gripper_status.warning,
    status->sys_kinco_status.status_word,
    status->sys_kinco_status.error_code,
    status->sys_kinco_status.position,
    status->sys_kinco_status.velocity,
    status->sys_zeroerr_status.status_word,
    status->sys_zeroerr_status.error_code,
    status->sys_zeroerr_status.position,
    status->sys_zeroerr_status.velocity,
    status->gpio_in_status[0],
    status->gpio_in_status[1],
    status->gpio_in_status[2],
    status->gpio_in_status[3],
    status->gpio_in_status[4],
    status->gpio_in_status[5]
  );

  if (len < 0 || len >= sizeof(sys_status_payload)) {
    printf("MQTT JSON payload overflow!\r\n");
    return false;
  }

  MQTTMessage message;
  message.qos = QOS0;
  message.retained = 0;
  message.dup = 0;
  message.payload = sys_status_payload;
  message.payloadlen = strlen(sys_status_payload);

  int rc = MQTTPublish(&mqttClient, sys_status_topic, &message);
  if (rc != 0) {
    printf("MQTT publish SysStatus failed, rc=%d\r\n", rc);
    return false;
  }

  return true;
}

static void mqtt_subscribe_all(void)
{
  int rc;
  if ((rc = MQTTSubscribe(&mqttClient, "robot/gpio/cmd", QOS0, messageArrived)) == 0)
    printf("Subscribed robot/gpio/cmd\r\n");
  else
    printf("Subscribe robot/gpio/cmd failed (%d)\r\n", rc);

  if ((rc = MQTTSubscribe(&mqttClient, "robot/gripper/cmd", QOS0, messageArrived)) == 0)
    printf("Subscribed robot/gripper/cmd\r\n");
  else
    printf("Subscribe robot/gripper/cmd failed (%d)\r\n", rc);

  if ((rc = MQTTSubscribe(&mqttClient, "robot/servo/cmd", QOS0, messageArrived)) == 0)
    printf("Subscribed robot/servo/cmd\r\n");
  else
    printf("Subscribe robot/servo/cmd failed (%d)\r\n", rc);
}

#define MONITOR_INTERVAL_MS 1000 // 监控周期
void StartMonitorUpdateTask(void* argument)
{
  for (;;)
  {
    // 喂看门狗
    HAL_IWDG_Refresh(&hiwdg);
    sys_run_cnt++;

    // 打印系统运行时间与错误统计
    printf("t:%lu s, e0:%d, e1:%d, e2:%d\r\n",
      sys_run_cnt, gripper_err_cnt, mqtt_err_cnt, servo_error_cnt);
    // 其他错误检测
    if (gripper_err_cnt >= 30 || mqtt_err_cnt >= 50 || servo_error_cnt >= 50)
    {
      printf("System Reset due to errors!\r\n");
      system_reset();
    }

    osDelay(MONITOR_INTERVAL_MS);
  }
}

/* USER CODE END Application */
