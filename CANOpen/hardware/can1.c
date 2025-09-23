#include "can1.h"

extern CAN_HandleTypeDef hcan1; 

static CO_Data* p_co_date = NULL;

unsigned char CAN1_Init(CO_Data* d, uint32_t bitrate)
{

}

// Send a CAN message passed from the CANopen stack
unsigned char canSend(CAN_PORT notused, Message* m)
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;
    HAL_StatusTypeDef hal_status;
    int i;

    // CAN header 
    TxHeader.StdId = m->cob_id;           // standard ID
    TxHeader.ExtId = 0;                   // not use Extended ID
    TxHeader.IDE   = CAN_ID_STD;          // Standard Frame
    TxHeader.RTR   = (m->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA);
    TxHeader.DLC   = m->len;              // payload length
    TxHeader.TransmitGlobalTime = DISABLE;

    // transmit data
    hal_status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader, m->data, &TxMailbox);

    if (hal_status != HAL_OK) {
        return 0;   // error
    }

    return 1;       // successful
}

unsigned char canChangeBaudRate_driver(CAN_HANDLE fd, char* baud)
{
    return 0;
}