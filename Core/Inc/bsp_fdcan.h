#ifndef __BSP_FDCAN_H__
#define __BSP_FDCAN_H__
#include "main.h"
#include "fdcan.h"

/**
 * 初始化：
 * bsp_can_init() 系统启动调用一次即可
 * 
 * 接口函数:
 * fdcanx_send_data() 		发送一阵CANFD数据
 * fdcan1_rx_len > 0  		接受到数据
 * 										extern uint8_t rx_data1[64];
 * 										extern uint16_t rec_id1;
 *									  extern volatile uint8_t fdcan1_rx_len;
 */


#define hcan_t FDCAN_HandleTypeDef

void bsp_can_init(void);
void can_filter_init(void);
uint8_t fdcanx_send_data(hcan_t *hfdcan, uint16_t id, uint8_t *data, uint32_t len);
uint8_t fdcanx_receive(hcan_t *hfdcan, uint16_t *rec_id, uint8_t *buf);
void fdcan1_rx_callback(void);


#endif /* __BSP_FDCAN_H_ */

