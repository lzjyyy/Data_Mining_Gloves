#include "can1.h"

extern CAN_HandleTypeDef hcan1;
extern CO_Data Kinco_Ctrl_Data;
static CO_Data* p_co_date = NULL;
static void printCanMessage(uint32_t id, uint8_t dlc, uint8_t* data, const char* prefix);

unsigned char CAN1_Init(CO_Data* d, uint32_t bitrate)
{

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

        printCanMessage(TxHeader.StdId, TxHeader.DLC, m->data, "CAN TX");
        // transmit data
        hal_status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, m->data, &TxMailbox);

        if (hal_status != HAL_OK) {
            printf("CAN TX ERROR: HAL status=%d\r\n", hal_status);
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