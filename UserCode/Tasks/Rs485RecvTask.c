/**/
#include "cmsis_os2.h" 
#include "FreeRTOS.h"
#include "queue.h"
/**/
#include "Rs485RecvTask.h"
#include "usart.h"
#include "dma.h"
#include "data_manager.h"
#include "base_convert.h"
#include "crc.h"
#include "data_manager.h"
#include "PosLoopTask.h"
#include "Kin_Slover.h"
#include "arm_math.h"

#include "rs485_protocol.h"
#include "rs485_handlers.h"

#include <SysCtrlTask.h>

#include "LaunchTask.h"

static rs485_proto_ctx_t g_rs485_ctx;

static void RS485_Proto_Init(void)
{
    //从eeprom读取的地址已经存放在g_rs485_cfg.slave_addr中
    printf("slave addr=%X, baudrate=%ld\r\n", g_rs485_cfg.slave_addr, g_rs485_cfg.baudrate);
    rs485_proto_init(&g_rs485_ctx, g_rs485_cfg.slave_addr); 
    rs485_handlers_init(&g_rs485_ctx);      /* 注册功能码处理器 */
}


static void RS485_Proto_SyncConfig(void)
{
    if (g_rs485_ctx.slave_addr != g_rs485_cfg.slave_addr) {
        g_rs485_ctx.slave_addr = g_rs485_cfg.slave_addr;
    }
}


// static uint8_t rs485RecPosDataByte[12]={0};
// static uint8_t rs485RecSpdDataByte[12]={0};



//extern float posRef[MOTOR_COUNT];
//extern float spdTar[MOTOR_COUNT];

/* 先定义好存放数据的空间，队列大小是5，这里就定义5个*/
UART3_RX_TypeDef uart3_rx_data_t[UART3_BUFFER_QUANTITY];
/* 定义一个变量用作分配上面定义的数据*/
uint8_t uart3_buff_ctrl = 0;
/* 引用串口DMA的handle*/
extern DMA_HandleTypeDef hdma_usart3_rx;
/* 队列handle,去freertos.c文件里面找CubeMX给你创建好的*/
extern osMessageQueueId_t Rs485RecvQueueHandle;

static void RS485_ReinitUart3Baudrate(uint32_t baudrate)
{
    g_rs485_cfg.baudrate = baudrate;
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
    HAL_UART_DMAStop(&huart3);
    __HAL_UART_DISABLE_IT(&huart3, UART_IT_IDLE);
    HAL_UART_DeInit(&huart3);

    huart3.Instance = USART3;
    huart3.Init.BaudRate = baudrate;
    huart3.Init.WordLength = UART_WORDLENGTH_8B;
    huart3.Init.StopBits = UART_STOPBITS_1;
    huart3.Init.Parity = UART_PARITY_NONE;
    huart3.Init.Mode = UART_MODE_TX_RX;
    huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart3.Init.OverSampling = UART_OVERSAMPLING_16;
    huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    if (HAL_UART_Init(&huart3) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK) {
        Error_Handler();
    }

    HAL_UART_Receive_DMA(&huart3, uart3_rx_data_t[uart3_buff_ctrl].buffer, UART3_BUFFER_SIZE);
    __HAL_UART_CLEAR_IDLEFLAG(&huart3);
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
}



void USART3_DMA_Handler(void)
{
	if(__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE)!=RESET)/*判断是否是空闲中断*/ 
	{
        __HAL_UART_CLEAR_IDLEFLAG(&huart3);
        uint16_t len = UART3_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);
        if (len == 0)
        {
            /* 长度为 0，多半是误触发 / 错误后 DMA 被关掉，强制重启 DMA */
            HAL_UART_DMAStop(&huart3);
            HAL_UART_Receive_DMA(&huart3,
                                 uart3_rx_data_t[uart3_buff_ctrl].buffer,
                                 UART3_BUFFER_SIZE);
            printf("uart3 idle irq, zero length, restart dma\r\n");
            return;
        }

		UART3_RX_TypeDef *pUartData;         /*定义指向创建串口数据的指针 */

		HAL_UART_DMAStop(&huart3);           /*停止本次DMA传输*/        
		/* 计算接收到的数据长度，放进串口数据结构体中 */
		uart3_rx_data_t[uart3_buff_ctrl].size = len;
		/* 将这个串口数据的结构体地址给指针 */
		pUartData = &uart3_rx_data_t[uart3_buff_ctrl];
		/* 把指向串口接收数据的指针放入消息队列；注意，这里传的是指针的地址，也就是指针串口数据结构体的指针的地址不能传pUartData本身 */
		xQueueSendFromISR(Rs485RecvQueueHandle, &pUartData, NULL);
		/* 这里进行加一，使其下一次DMA传输时将接收的数据放到下一个串口缓冲区中 */
		uart3_buff_ctrl++;
		/* 取余操作防止越界 */
		uart3_buff_ctrl %= UART3_BUFFER_QUANTITY;
		/* 重启开始DMA传输 */
		HAL_UART_Receive_DMA(&huart3, uart3_rx_data_t[uart3_buff_ctrl].buffer, UART3_BUFFER_SIZE);
	}
}

/*-----------------------------------------------------------------------------------------------------------------------100hz线程处理*/
void StartRs485RecvTask(void *argument)
{
    /* 485 方向：接收 */
    HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);

    UART3_RX_TypeDef *pRs485RecvData = NULL;

    RS485_Proto_Init();

    for (;;)
    {
        /* 阻塞等待直到 ISR 投递一帧 */
        if (xQueueReceive(Rs485RecvQueueHandle, &pRs485RecvData, portMAX_DELAY) == pdPASS && pRs485RecvData)
        {
            rs485_reply_t reply;
            reply.is_OTA_reply = false;
            reply.is_reset_reply = false;
            reply.is_baud_change_reply = false;
            reply.new_baudrate = g_rs485_cfg.baudrate;
            RS485_Proto_SyncConfig();
            bool parsed = rs485_proto_dispatch(&g_rs485_ctx,
                                               pRs485RecvData->buffer,
                                               pRs485RecvData->size,
                                               &reply);
            RS485_Proto_SyncConfig();

            if (parsed && reply.has_reply && reply.len > 0) {
                /* 切到发送 */
                HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_SET);

                HAL_UART_Transmit_DMA(&huart3, reply.txbuf, reply.len);
								printf("aaaaaaa\r\n");
                /* 回到接收 */
                //等待485发送完成
                while (huart3.gState != HAL_UART_STATE_READY) {
                    osDelay(1);
                }
                while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET) {
                    osDelay(1);
                }
								printf("bbbbbbbbbb\r\n");
                if(reply.is_baud_change_reply == true)
                {
                    RS485_ReinitUart3Baudrate(reply.new_baudrate);
                }
                if(reply.is_OTA_reply==true)
                {
									printf("ccccccccc\r\n");
                    ota_reply_complete_flag = 1; //设置OTA回复完成标志
                }
                if (reply.is_reset_reply == true)
                {
                    //判断LAUNCH_DONE_BIT是否置位，即是否完成lunch
                    if((osEventFlagsGet(LaunchEventsHandle) & LAUNCH_DONE_BIT) != 0){
                        system_reset_request = 1; //设置系统复位请求标志
                    }
                        
                }
                HAL_GPIO_WritePin(RS485_RE_GPIO_Port, RS485_RE_Pin, GPIO_PIN_RESET);
            }
        }
    }
}


