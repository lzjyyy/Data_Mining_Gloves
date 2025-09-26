#ifndef _KINCO_H_
#define _KINCO_H_

#include "timer5.h"
#include "data.h"
#include "canfestival.h"
#include "Kinco_Ctrl.h"
#include "can.h"

#define KINCO_MASTER_MODE_ID 2
#define KINCO_SLAVE_NODE_ID 1
#define CAN1_CH  0
#define CAN2_CH  1

#define SERVO_WORK_MODE_INDEX   0x6060 // 1 byte (int8_t)
#define VALID_WORK_MODE_INDEX   0x6061 // 1 byte (int8_t)
#define TARGET_POS_INDEX        0x607A // 4 byte (int32_t)
#define ACTUAL_POS_INDEX        0x6063 // 4 byte (int32_t)
#define ACTUAL_POS_INDEX_ZEROERR  0x6064 // 4 byte (int32_t)
#define PROFILED_VELOCITY_INDEX 0x6081 // 4 byte (int32_t)
#define PROFILED_ACC_INDEX      0x6083 // 4 byte (int32_t) 
#define PROFILED_DEC_INDEX      0x6084 // 4 byte (int32_t)
#define TARGET_VELOCITY_INDEX   0x60FF // 4 byte (int32_t)
#define ACTUAL_VELOCITY_INDEX   0x606C // 4 byte (int32_t)
#define SERVO_CTRL_WORD_INDEX   0x6040 // 2 byte (unt16_t)
#define SERVO_STATUS_WORD_INDEX 0x6041 // 2 byte (uint16_t)
#define ACTUAL_CURRENT_INDEX    0x6078 // 2 byte (int16_t)
#define STOP_OPTION_INDEX       0x605A // 2 byte (uint16_t)
#define ERROR_CODE_INDEX        0x603F // 2 byte (uint16_t)
#define RPDO1_PARAM_INDEX       0x1400 // 2 byte (uint16_t)
#define RPDO2_PARAM_INDEX       0x1401 // 2 byte (uint16_t)
#define RPDO1_MAPPING_INDEX     0x1600 // 2 byte (uint16_t)
#define RPDO2_MAPPING_INDEX     0x1601 // 2 byte (uint16_t)
#define TPDO1_PARAM_INDEX       0x1800 // 2 byte (uint16_t)
#define TPDO2_PARAM_INDEX       0x1801 // 2 byte (uint16_t)
#define TPDO1_MAPPING_INDEX     0x1A00 // 2 byte (uint16_t)
#define TPDO2_MAPPING_INDEX     0x1A01 // 2 byte (uint16_t)

#define POSITION_MODE   1
#define KINCO_RESOLUTION    65536
#define KINCO_RPM_TO_DEC_MULT   0.273067
#define KINCO_ACC_DEC_MULT    0.016384
#define KINCO_DEFAULT_PROFILED_VELOCITY     200 // 200 rpm 
#define KINCO_DEFAULT_PROFILED_ACC        100 // 200 rpm/s 
#define KINCO_DEFAULT_PROFILED_DEC        100 // 200 rpm/s 

#define SDO_OK              0   // 成功
#define SDO_ERR_TYPE       -1   // 不支持的数据类型
#define SDO_ERR_SEND       -2   // 写请求发送失败
#define SDO_ERR_ABORT      -3   // 从站返回Abort
#define SDO_ERR_TIMEOUT    -4   // 超时未完成

int SDO_WriteRequest(CO_Data* d, uint8_t nodeId, uint16_t index, uint8_t subIndex,
    void* data, uint8_t dataType);
int SDO_ReadRequest(CO_Data* d, uint8_t nodeId, uint16_t index, uint8_t subIndex,
    void* data, uint8_t dataType);
void Kinco_MasterNode_Init(void);
void Kinco_Setup(void);
void Kinco_Enable_PDO(void);
void Kinco_Disable_PDO(void);
void Kinco_MovPos_PDO(uint32_t pos);
void Kinco_SetVel_PDO(uint32_t vel);

#endif