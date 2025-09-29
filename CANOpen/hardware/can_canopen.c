#include "can_canopen.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern CO_Data Kinco_Ctrl_Data;
extern CO_Data ZeroErr_Ctrl_Data;

static void printCanMessage(uint32_t id, uint8_t dlc, uint8_t* data, const char* prefix);

bool can1Init(CO_Data* d, uint32_t bitrate)
{
    (void)d;
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        printf("CAN1 start failed,stop StartCanTestTask!\r\n");
        return false;
    }

    /* Enable FIFO0 message pending interrupt */
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        printf("Enable CAN1 notification failed!\r\n");
        return false;
    }

    return true;
}

bool can2Init(CO_Data* d, uint32_t bitrate)
{
    (void)d;
    if (HAL_CAN_Start(&hcan2) != HAL_OK)
    {
        printf("CAN2 start failed,stop StartCanTestTask!\r\n");
        return false;
    }

    /* Enable FIFO0 message pending interrupt */
    if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
    {
        printf("Enable CAN2 notification failed!\r\n");
        return false;
    }

    return true;
}

// Send a CAN message passed from the CANopen stack
unsigned char canSend(CAN_PORT canHandle, Message* m)
{
    if (Kinco_Ctrl_Data.canHandle == canHandle)
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t TxMailbox;
        HAL_StatusTypeDef hal_status;
        int i;

        // CAN header 
        TxHeader.StdId = m->cob_id;           // standard ID
        TxHeader.ExtId = 0;                   // not use Extended ID
        TxHeader.IDE = CAN_ID_STD;          // Standard Frame
        TxHeader.RTR = (m->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA);
        TxHeader.DLC = m->len;              // payload length
        TxHeader.TransmitGlobalTime = DISABLE;

        printCanMessage(TxHeader.StdId, TxHeader.DLC, m->data, "CAN1 TX");
        // transmit data
        hal_status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, m->data, &TxMailbox);

        if (hal_status != HAL_OK) {
            printf("CAN1 TX ERROR: HAL status=%d\r\n", hal_status);
            return 1;   // error
        }

        return 0;       // successful
    }
    else if (ZeroErr_Ctrl_Data.canHandle == canHandle)
    {
        CAN_TxHeaderTypeDef TxHeader;
        uint32_t TxMailbox;
        HAL_StatusTypeDef hal_status;
        int i;

        // CAN header 
        TxHeader.StdId = m->cob_id;           // standard ID
        TxHeader.ExtId = 0;                   // not use Extended ID
        TxHeader.IDE = CAN_ID_STD;          // Standard Frame
        TxHeader.RTR = (m->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA);
        TxHeader.DLC = m->len;              // payload length
        TxHeader.TransmitGlobalTime = DISABLE;

        printCanMessage(TxHeader.StdId, TxHeader.DLC, m->data, "CAN2 TX");
        // transmit data
        hal_status = HAL_CAN_AddTxMessage(&hcan2, &TxHeader, m->data, &TxMailbox);

        if (hal_status != HAL_OK) {
            printf("CAN2 TX ERROR: HAL status=%d\r\n", hal_status);
            return 1;   // error
        }

        return 0;       // successful
    }
}

unsigned char canChangeBaudRate_driver(CAN_HANDLE fd, char* baud)
{
    return 0;
}

// 打印 CAN 报文
static void printCanMessage(uint32_t id, uint8_t dlc, uint8_t* data, const char* prefix)
{
    printf("%s: ID=0x%03X DLC=%d Data=", prefix, id, dlc);
    for (int i = 0; i < dlc; i++) {
        printf("%02X ", data[i]);
    }
    printf("\r\n");
}