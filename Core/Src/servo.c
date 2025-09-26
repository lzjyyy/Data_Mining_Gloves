#include "servo.h"

static uint8_t can1_write_data_u8[8] = { 0 };
static uint16_t can1_write_data_u16[8] = { 0 };
static uint32_t can1_write_data_u32[8] = { 0 };
static uint8_t can2_write_data_u8[8] = { 0 };
static uint16_t can2_write_data_u16[8] = { 0 };
static uint32_t can2_write_data_u32[8] = { 0 };

int SDO_WriteRequest(CO_Data* d, uint8_t nodeId, uint16_t index, uint8_t subIndex,
    void* data, uint8_t dataType)
{
    uint8_t count = (dataType == uint8) ? 1 :
        (dataType == uint16) ? 2 :
        (dataType == uint32) ? 4 : 0;

    if (count == 0)
        return SDO_ERR_TYPE;  // 不支持的数据类型

    uint8_t res = writeNetworkDict(d, nodeId, index, subIndex, count, dataType, data, 0);
    if (res != 0)
        return SDO_ERR_SEND;  // 写请求发送失败

    uint32_t abortCode = 0;
    uint8_t state = 0;

    // for (int i = 0; i < 1000; i++)   // 最多等100ms
    // {
    //     state = getWriteResultNetworkDict(d, nodeId, &abortCode);
    //     if (state == SDO_FINISHED)
    //     {
    //         return SDO_OK;   // 写成功
    //     }
    //     else if (state != SDO_UPLOAD_IN_PROGRESS)
    //     {
    //         printf("SDO Write Error: 0x%08X\n", abortCode);
    //         return SDO_ERR_ABORT;  // 从站返回错误
    //     }
    //     osDelay(1);
    // }
    uint32_t cnt = 0;
    while (getWriteResultNetworkDict(d, nodeId, &abortCode) != SDO_FINISHED)
    {
        cnt++;
        osDelay(1);
    }
    printf("Write time:%d\r\n", cnt);

    return SDO_OK;
}

int SDO_ReadRequest(CO_Data* d, uint8_t nodeId, uint16_t index, uint8_t subIndex,
    void* data, uint8_t dataType)
{
    uint32_t expectedCount = (dataType == uint8) ? 1 :
        (dataType == uint16) ? 2 :
        (dataType == uint32) ? 4 : 0;

    if (expectedCount == 0)
        return SDO_ERR_TYPE;  // 不支持的数据类型

    uint8_t res = readNetworkDict(d, nodeId, index, subIndex, dataType, 0);
    if (res != 0)
        return SDO_ERR_SEND;  // 读请求发送失败

    uint32_t abortCode = 0;
    uint8_t state = 0;
    uint32_t tmpData = 0;  // 临时变量用于 getReadResultNetworkDict

    // for (int i = 0; i < 1000; i++)  // 最多等100ms
    // {
    //     state = getReadResultNetworkDict(d, nodeId, &tmpData, &expectedCount, &abortCode);
    //     if (state == SDO_FINISHED)
    //     {
    //         // 根据 dataType 拷贝到用户提供的缓冲
    //         switch (dataType)
    //         {
    //         case uint8:  *(uint8_t*)data = (uint8_t)tmpData; break;
    //         case uint16: *(uint16_t*)data = (uint16_t)tmpData; break;
    //         case uint32: *(uint32_t*)data = tmpData; break;
    //         }
    //         return SDO_OK;
    //     }
    //     else if (state != SDO_UPLOAD_IN_PROGRESS)
    //     {
    //         printf("SDO Read Error: 0x%08X\n", abortCode);
    //         return SDO_ERR_ABORT;
    //     }
    //     osDelay(1);
    // }
    uint32_t cnt = 0;
    while (getReadResultNetworkDict(d, nodeId, &tmpData, &expectedCount, &abortCode) != SDO_FINISHED)
    {
        cnt++;
        osDelay(1);
    }
    printf("Read time:%d\r\n", cnt);

    switch (dataType)
    {
    case uint8:  *(uint8_t*)data = (uint8_t)tmpData; break;
    case uint16: *(uint16_t*)data = (uint16_t)tmpData; break;
    case uint32: *(uint32_t*)data = tmpData; break;
    }
    return SDO_OK;


    return SDO_ERR_TIMEOUT;
}

void Kinco_Setup(void)
{
    int result = 0;
    uint8_t work_mode = 1;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, SERVO_WORK_MODE_INDEX, 0x00, &work_mode, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write SERVO_WORK_MODE_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write SERVO_WORK_MODE_INDEX Failed\r\n");
    }

    work_mode = 0;
    result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, SERVO_WORK_MODE_INDEX, 0x00, &work_mode, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO read SERVO_WORK_MODE_INDEX Success,work_mode:0x%x\r\n", work_mode);
    }
    else
    {
        printf("Kinco SDO read SERVO_WORK_MODE_INDEX Success\r\n");
    }
    if (work_mode == POSITION_MODE)
    {
        printf("Kinco set position mode Success\r\n");
    }
    else
    {
        printf("Kinco set position mode Failed\r\n");
    }

    uint32_t set_val_u32 = (uint32_t)(KINCO_DEFAULT_PROFILED_VELOCITY * (KINCO_RESOLUTION * KINCO_RPM_TO_DEC_MULT));
    uint32_t profiled_val = set_val_u32;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_VELOCITY_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write PROFILED_VELOCITY_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write PROFILED_VELOCITY_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_VELOCITY_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO read PROFILED_VELOCITY_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("Kinco SDO read PROFILED_VELOCITY_INDEX Success\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        printf("Kinco set profiled velocity to %d rpm Success\r\n", KINCO_DEFAULT_PROFILED_VELOCITY);
    }
    else
    {
        printf("Kinco set profiled velocity to %d rpm Failed\r\n", KINCO_DEFAULT_PROFILED_VELOCITY);
    }

    set_val_u32 = (uint32_t)(KINCO_DEFAULT_PROFILED_ACC * (KINCO_RESOLUTION * KINCO_ACC_DEC_MULT));
    profiled_val = set_val_u32;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_ACC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write PROFILED_ACC_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write PROFILED_ACC_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_ACC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO read PROFILED_ACC_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("Kinco SDO read PROFILED_ACC_INDEX Success\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        printf("Kinco set profiled acceleration to %d rpm Success\r\n", KINCO_DEFAULT_PROFILED_ACC);
    }
    else
    {
        printf("Kinco set profiled acceleration to %d rpm Failed\r\n", KINCO_DEFAULT_PROFILED_ACC);
    }

    set_val_u32 = (uint32_t)(KINCO_DEFAULT_PROFILED_DEC * (KINCO_RESOLUTION * KINCO_ACC_DEC_MULT));
    profiled_val = set_val_u32;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_DEC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write PROFILED_DEC_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write PROFILED_DEC_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_DEC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO read PROFILED_DEC_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("Kinco SDO read PROFILED_DEC_INDEX Success\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        printf("Kinco set profiled deceleration to %d rpm Success\r\n", KINCO_DEFAULT_PROFILED_DEC);
    }
    else
    {
        printf("Kinco set profiled deceleration to %d rpm Failed\r\n", KINCO_DEFAULT_PROFILED_DEC);
    }

    set_val_u32 = 0x08000021;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }

    uint8_t set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60400010;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x607A0020;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x02, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u8 = 0x02;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x200 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }

    set_val_u32 = 0x8000021 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x00;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60810020;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x201 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }

    set_val_u32 = 0x80000180 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x00;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60410010;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }

    set_val_u32 = 0x60630020;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x02, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }

    set_val_u32 = 0x60780010;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x03, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }

    set_val_u8 = 0x03;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x00000180 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        print_can1_recv_msg();
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
}

void Kinco_Enable(void)
{
    // Kinco_Ctrl_Data.scanIndexOD = Kinco_Ctrl_scanIndexOD;
    // Kinco_Ctrl_Data.valueRangeTest = Kinco_Ctrl_valueRangeTest;
    // Kinco_Ctrl_Data.storeODSubIndex = NULL;

    printf("scanIndexOD = %p\r\n", Kinco_Ctrl_Data.scanIndexOD);

    uint32_t ctrl_word = 0x0006;
    uint32_t size = 2;

    // const indextable* ptrTable;
    // UNS32 err;
    // ptrTable = Kinco_Ctrl_scanIndexOD(0x6040, &err, NULL);
    // if (err != OD_SUCCESSFUL) while (1); // 错误
    // printf("Kinco_Ctrl_scanIndexOD:%p\r\n", Kinco_Ctrl_scanIndexOD);
    // if (ptrTable->pSubindex[0].pObject == NULL) while (1); // 错误
    // printf("ptrTable:%p,ptrTable->pSubindex[0].pObject:%p\r\n", ptrTable, ptrTable->pSubindex[0].pObject);

    // setODentry(&Kinco_Ctrl_Data, 0x2000, 0x0, &ctrl_word, &size, 0);
    Controlword = 0x0006;
    // sendPDOrequest(&Kinco_Ctrl_Data, 0x1800);
    // writeLocalDict(&Kinco_Ctrl_Data, 0x6040, 0x00, &ctrl_word, (uint32_t*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);
}