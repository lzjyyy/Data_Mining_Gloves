#include "rs485.h"

static void RS485_SetSendMode(uint8_t ch)
{
    if (ch == RS485C_CH)
    {
        HAL_GPIO_WritePin(RS485C_DIR_PORT, RS485C_DIR_PIN, GPIO_PIN_SET);
    }
    else
    {
        return;
    }
}

static void RS485_SetRecvMode(uint8_t ch)
{
    if (ch == RS485C_CH)
    {
        HAL_GPIO_WritePin(RS485C_DIR_PORT, RS485C_DIR_PIN, GPIO_PIN_RESET);
    }
    else
    {
        return;
    }
}

void RS485_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };

    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitStruct.Pin = RS485C_DIR_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(RS485C_DIR_PORT, &GPIO_InitStruct);

    RS485_SetRecvMode(RS485C_CH);
}

HAL_StatusTypeDef RS485_Send(uint8_t ch, uint8_t* pData, uint16_t size, uint32_t timeout)
{
    if (ch == RS485A_CH)
    {
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart1, pData, size, timeout);

        while (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_TC) == RESET)
        {
            ;
        }
        return status;
    }
    else if (ch == RS485B_CH)
    {
        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, pData, size, timeout);

        while (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_TC) == RESET)
        {
            ;
        }
        return status;
    }
    else if (ch == RS485C_CH)
    {
        RS485_SetSendMode(RS485C_CH);
        HAL_Delay(5);

        HAL_StatusTypeDef status = HAL_UART_Transmit(&huart4, pData, size, timeout);

        while (__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET)
        {
            ;
        }

        RS485_SetRecvMode(RS485C_CH);
        return status;
    }
    else
    {
        printf("Invalid RS485 CH!\r\n");
        return -1;
    }
}

HAL_StatusTypeDef RS485_Receive(uint8_t ch, uint8_t* pData, uint16_t size, uint32_t timeout)
{
    if (ch == RS485A_CH)
    {
        return HAL_UART_Receive(&huart1, pData, size, timeout);
    }
    else if (ch == RS485B_CH)
    {
        return HAL_UART_Receive(&huart3, pData, size, timeout);
    }
    else if (ch == RS485C_CH)
    {
        RS485_SetRecvMode(RS485C_CH);
        return HAL_UART_Receive(&huart4, pData, size, timeout);
    }
}
