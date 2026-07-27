#include "bsp_fdcan.h"
#include "main.h"
#include <stdio.h>

uint8_t rx_data1[64] = {0};
uint16_t rec_id1;
volatile uint8_t fdcan1_rx_len = 0;
uint8_t rx_data2[8] = {0};
uint16_t rec_id2;


/* ========== 内部函数 ========== */

void bsp_can_init(void)
{
		can_filter_init();
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK)
    {
        printf("HAL_FDCAN_Start FAIL\r\n");
    }
    else
    {
        printf("HAL_FDCAN_Start OK\r\n");
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        printf("ActivateNotification FAIL\r\n");
    }
    else
    {
        printf("ActivateNotification OK\r\n");
    }
}

void can_filter_init(void)
{
	FDCAN_FilterTypeDef fdcan_filter;
	fdcan_filter.IdType = FDCAN_STANDARD_ID;                     
	fdcan_filter.FilterIndex = 0;                                                
	fdcan_filter.FilterType = FDCAN_FILTER_MASK;                   
	fdcan_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;          
	fdcan_filter.FilterID1 = 0x00;                               
	fdcan_filter.FilterID2 = 0x00;
	HAL_FDCAN_ConfigFilter(&hfdcan1,&fdcan_filter); 		 				 
	HAL_FDCAN_ConfigGlobalFilter(&hfdcan1,FDCAN_REJECT,FDCAN_REJECT,FDCAN_REJECT_REMOTE,FDCAN_REJECT_REMOTE);
	HAL_FDCAN_ConfigFifoWatermark(&hfdcan1, FDCAN_CFG_RX_FIFO0, 1);
}


/* ========== 对外接口 ========== */


uint8_t fdcanx_send_data(FDCAN_HandleTypeDef *hfdcan, uint16_t id, uint8_t *data, uint32_t len)
{
    FDCAN_TxHeaderTypeDef pTxHeader = {0};

    if (hfdcan == NULL || data == NULL)
        return 2;

    pTxHeader.Identifier          = id;
    pTxHeader.IdType              = FDCAN_STANDARD_ID;
    pTxHeader.TxFrameType         = FDCAN_DATA_FRAME;
    pTxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    pTxHeader.BitRateSwitch       = FDCAN_BRS_ON;
    pTxHeader.FDFormat            = FDCAN_FD_CAN;
    pTxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    pTxHeader.MessageMarker       = 0;

    switch (len)
    {
        case 0:  pTxHeader.DataLength = FDCAN_DLC_BYTES_0;  break;
        case 1:  pTxHeader.DataLength = FDCAN_DLC_BYTES_1;  break;
        case 2:  pTxHeader.DataLength = FDCAN_DLC_BYTES_2;  break;
        case 3:  pTxHeader.DataLength = FDCAN_DLC_BYTES_3;  break;
        case 4:  pTxHeader.DataLength = FDCAN_DLC_BYTES_4;  break;
        case 5:  pTxHeader.DataLength = FDCAN_DLC_BYTES_5;  break;
        case 6:  pTxHeader.DataLength = FDCAN_DLC_BYTES_6;  break;
        case 7:  pTxHeader.DataLength = FDCAN_DLC_BYTES_7;  break;
        case 8:  pTxHeader.DataLength = FDCAN_DLC_BYTES_8;  break;
        case 12: pTxHeader.DataLength = FDCAN_DLC_BYTES_12; break;
        case 16: pTxHeader.DataLength = FDCAN_DLC_BYTES_16; break;
        case 20: pTxHeader.DataLength = FDCAN_DLC_BYTES_20; break;
        case 24: pTxHeader.DataLength = FDCAN_DLC_BYTES_24; break;
        case 32: pTxHeader.DataLength = FDCAN_DLC_BYTES_32; break;
        case 48: pTxHeader.DataLength = FDCAN_DLC_BYTES_48; break;
        case 64: pTxHeader.DataLength = FDCAN_DLC_BYTES_64; break;
        default:
            return 2;
    }

    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &pTxHeader, data) != HAL_OK)
        return 1;

    return 0;
}

uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint16_t *rec_id, uint8_t *buf)
{
    FDCAN_RxHeaderTypeDef pRxHeader = {0};

    if (hfdcan == NULL || rec_id == NULL || buf == NULL)
        return 0;

    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &pRxHeader, buf) != HAL_OK)
        return 0;

    *rec_id = pRxHeader.Identifier;

    switch (pRxHeader.DataLength)
    {
        case FDCAN_DLC_BYTES_0:  return 0;
        case FDCAN_DLC_BYTES_1:  return 1;
        case FDCAN_DLC_BYTES_2:  return 2;
        case FDCAN_DLC_BYTES_3:  return 3;
        case FDCAN_DLC_BYTES_4:  return 4;
        case FDCAN_DLC_BYTES_5:  return 5;
        case FDCAN_DLC_BYTES_6:  return 6;
        case FDCAN_DLC_BYTES_7:  return 7;
        case FDCAN_DLC_BYTES_8:  return 8;
        case FDCAN_DLC_BYTES_12: return 12;
        case FDCAN_DLC_BYTES_16: return 16;
        case FDCAN_DLC_BYTES_20: return 20;
        case FDCAN_DLC_BYTES_24: return 24;
        case FDCAN_DLC_BYTES_32: return 32;
        case FDCAN_DLC_BYTES_48: return 48;
        case FDCAN_DLC_BYTES_64: return 64;
        default: return 0;
    }
}

void fdcan1_rx_callback(void)
{
		fdcan1_rx_len =fdcanx_receive(&hfdcan1, &rec_id1, rx_data1);
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if(hfdcan == &hfdcan1)
		{
				fdcan1_rx_callback();
		}

}











