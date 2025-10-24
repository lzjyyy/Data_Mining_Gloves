#include "servo.h"

static Servo_Status_t kinco_curr_status;
static Servo_Status_t zeroerr_curr_status;
static bool kinco_lock;
static bool zeroerr_lock;

#define SDO_READ_TIMEOUT_MS 500  // 500ms超时，可根据实际调整

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
    // uint8_t state = 0;

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
    // printf("Write time:%d\r\n", cnt);

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
    // uint8_t state = 0;
    uint32_t tmpData = 0;  // 临时变量用于 getReadResultNetworkDict
    uint32_t timeout = 0;
    uint32_t cnt = 0;

    while (getReadResultNetworkDict(d, nodeId, &tmpData, &expectedCount, &abortCode) != SDO_FINISHED)
    {
        cnt++;
        osDelay(1);
        timeout++;

        if (timeout > SDO_READ_TIMEOUT_MS)
        {
            closeSDOtransfer(d, nodeId, SDO_CLIENT); // 🔸强制关闭SDO会话，防止状态机挂死
            return SDO_ERR_TIMEOUT;
        }
    }
    // printf("Read time:%d\r\n", cnt);

    switch (dataType)
    {
    case uint8:  *(uint8_t*)data = (uint8_t)tmpData; break;
    case uint16: *(uint16_t*)data = (uint16_t)tmpData; break;
    case uint32: *(uint32_t*)data = tmpData; break;
    }
    return SDO_OK;
}

void Kinco_MasterNode_Init(void)
{
    setNodeId(&Kinco_Ctrl_Data, KINCO_MASTER_NODE_ID);
    Kinco_Ctrl_Data.canHandle = 0x01;
    setState(&Kinco_Ctrl_Data, Initialisation);
    //  setState(&Kinco_Ctrl_Data, Disconnected);
    setState(&Kinco_Ctrl_Data, Pre_operational);
    setState(&Kinco_Ctrl_Data, Operational);
    // stopSYNC(&Kinco_Ctrl_Data);

    masterSendNMTstateChange(&Kinco_Ctrl_Data, 0x01, NMT_Stop_Node);
    // Step 1: Reset Communication
    masterSendNMTstateChange(&Kinco_Ctrl_Data, 0x01, NMT_Reset_Comunication);
    osDelay(200); // 等待200ms，保证从站复位完成

    // Step 2: Enter Pre - Operational
    masterSendNMTstateChange(&Kinco_Ctrl_Data, 0x01, NMT_Enter_PreOperational);
    osDelay(50);
}

void Kinco_Setup(void)
{
    int result = 0;
    uint8_t work_mode = 1;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, SERVO_WORK_MODE_INDEX, 0x00, &work_mode, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write SERVO_WORK_MODE_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write SERVO_WORK_MODE_INDEX Failed\r\n");
    }

    work_mode = 0;
    result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, SERVO_WORK_MODE_INDEX, 0x00, &work_mode, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO read SERVO_WORK_MODE_INDEX Success,work_mode:0x%x\r\n", work_mode);
    }
    else
    {
        printf("Kinco SDO read SERVO_WORK_MODE_INDEX Failed\r\n");
    }
    if (work_mode == POSITION_MODE)
    {
        // printf("Kinco set position mode Success\r\n");
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
        // print_can1_recv_msg();
        // printf("Kinco SDO Write PROFILED_VELOCITY_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write PROFILED_VELOCITY_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_VELOCITY_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO read PROFILED_VELOCITY_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("Kinco SDO read PROFILED_VELOCITY_INDEX Failed\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        // printf("Kinco set profiled velocity to %d rpm Success\r\n", KINCO_DEFAULT_PROFILED_VELOCITY);
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
        // print_can1_recv_msg();
        // printf("Kinco SDO Write PROFILED_ACC_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write PROFILED_ACC_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_ACC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO read PROFILED_ACC_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("Kinco SDO read PROFILED_ACC_INDEX Failed\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        // printf("Kinco set profiled acceleration to %d rpm Success\r\n", KINCO_DEFAULT_PROFILED_ACC);
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
        // print_can1_recv_msg();
        // printf("Kinco SDO Write PROFILED_DEC_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write PROFILED_DEC_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, PROFILED_DEC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO read PROFILED_DEC_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("Kinco SDO read PROFILED_DEC_INDEX Failed\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        // printf("Kinco set profiled deceleration to %d rpm Success\r\n", KINCO_DEFAULT_PROFILED_DEC);
    }
    else
    {
        printf("Kinco set profiled deceleration to %d rpm Failed\r\n", KINCO_DEFAULT_PROFILED_DEC);
    }

    set_val_u32 = 0x08000021;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Failed\r\n");
    }

    uint8_t set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60400010;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x607A0020;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x02, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u8 = 0x02;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x200 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u32 = 0x8000021 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x00;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60810020;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Failed\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x201 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write RPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u32 = 0x80000180 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x00;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60410010;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60630020;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x02, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    // set_val_u32 = 0x60780010;
    // result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x03, &set_val_u32, uint32);
    // if (result == SDO_OK)
    // {
    //     // print_can1_recv_msg();
    //     // printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    // }
    // else
    // {
    //     printf("Kinco SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    // }

    set_val_u8 = 0x02;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x00000180 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u32 = 0x80000181 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO2_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x00;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u32 = 0x606C0020;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO2_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u32 = 0x00000181 + KINCO_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, TPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can1_recv_msg();
        // printf("Kinco SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("Kinco SDO Write TPDO2_PARAM_INDEX Failed\r\n");
    }
}

int Kinco_Enable_PDO(void)
{
    if (kinco_lock)
    {
        return ENABLE_BUSY;
    }

    kinco_lock = true;
    for (int i = 0; i < 10; i++)
    {
        Get_Parse_StatusWord(0);
        if (kinco_curr_status != INVALID) {
            break;
        }
    }

    if (kinco_curr_status == INVALID) {
        printf("Kinco enable failed, GET_STATUS_FAILED.\r\n");
        kinco_lock = false;
        return GET_STATUS_FAILED;
    }
    else if (kinco_curr_status == SWITCH_ON_DISABLED) {
        // printf("Kinco current status is SWITCH_ON_DISABLED, continue...\r\n");
    }

    uint32_t size;
    ctrl_word = 0x06;
    size = 2;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2000, 0x00, &ctrl_word, (UNS32*)&size, RW);
    target_pos = 0x0000;
    size = 4;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2001, 0x00, &target_pos, (UNS32*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);
    for (int i = 0; i < 10; i++) {
        Get_Parse_StatusWord(0);
        if (kinco_curr_status == READY_TO_SWITCH_ON) {
            break;
        }
    }

    if (kinco_curr_status != READY_TO_SWITCH_ON) {
        printf("Kinco enable failed, READY_SWITCH_ON_FAILED.\r\n");
        kinco_lock = false;
        return READY_SWITCH_ON_FAILED;
    }
    // else {
    //     printf("Kinco current status is READY_TO_SWITCH_ON, continue...\r\n");
    // }
    osDelay(5);

    ctrl_word = 0x07;
    size = 2;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2000, 0x00, &ctrl_word, (UNS32*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);
    for (int i = 0; i < 10; i++)
    {
        Get_Parse_StatusWord(0);
        if (kinco_curr_status == SWITCHED_ON) {
            break;
        }
    }

    if (kinco_curr_status != SWITCHED_ON) {
        printf("Kinco enable failed, SWITCHED_ON_FAILED.\r\n");
        kinco_lock = false;
        return SWITCHED_ON_FAILED;
    }
    // else {
    //     printf("Kinco current status is SWITCHED_ON, continue...\r\n");
    // }
    osDelay(5);

    ctrl_word = 0x0F;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2000, 0x00, &ctrl_word, (UNS32*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);
    for (int i = 0; i < 10; i++)
    {
        osDelay(50);
        Get_Parse_StatusWord(0);
        if (kinco_curr_status == OPERATION_ENABLED) {
            break;
        }
    }

    if (kinco_curr_status != OPERATION_ENABLED) {
        printf("Kinco enable failed, OPERATION_ENABLED_FAILED.\r\n");
        kinco_lock = false;
        return OPERATION_ENABLED_FAILED;
    }
    else {
        printf("Kinco current status is OPERATION_ENABLED, enable completed.\r\n");
    }

    kinco_lock = false;
    return ENABLE_OK;
}

int  Kinco_Disable_PDO(void)
{
    if (kinco_lock)
    {
        return DISABLE_BUSY;
    }

    uint32_t size = 2;
    ctrl_word = 0x06;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2000, 0x00, &ctrl_word, (uint32_t*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);
    for (int i = 0; i < 10; i++) {
        osDelay(50);
        Get_Parse_StatusWord(0);
        if (kinco_curr_status == READY_TO_SWITCH_ON) {
            break;
        }
    }

    if (kinco_curr_status != READY_TO_SWITCH_ON) {
        printf("Kinco enable failed, READY_SWITCH_ON_FAILED.\r\n");
        kinco_lock = false;
        return READY_SWITCH_ON_FAILED;
    }
    // else {
    //     printf("Kinco current status is READY_TO_SWITCH_ON, continue...\r\n");
    // }
    osDelay(5);

    ctrl_word = 0x00;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2000, 0x00, &ctrl_word, (uint32_t*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);
    for (int i = 0; i < 10; i++) {
        Get_Parse_StatusWord(0);
        if (kinco_curr_status == SWITCH_ON_DISABLED) {
            break;
        }
    }

    if (kinco_curr_status != SWITCH_ON_DISABLED) {
        printf("Kinco enable failed, SWITCH_ON_DISABLED_FAILED.\r\n");
        kinco_lock = false;
        return SWITCH_ON_DISABLED_FAILED;
    }
    else {
        printf("Kinco current status is SWITCH_ON_DISABLED, disable completed.\r\n");
        return DISABLE_OK;
    }
}

void Kinco_MovPos_PDO(uint32_t pos)
{
    uint32_t size;
    ctrl_word = 0x2F;
    size = 2;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2000, 0x00, &ctrl_word, (UNS32*)&size, RW);
    target_pos = pos;
    // printf("target_pos:0x%x\r\n", target_pos);
    size = 4;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2001, 0x00, &target_pos, (UNS32*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);

    ctrl_word = 0x3F;
    writeLocalDict(&Kinco_Ctrl_Data, 0x2000, 0x00, &ctrl_word, (UNS32*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);
    sendSYNC(&Kinco_Ctrl_Data);
    // print_can1_recv_msg();
    osDelay(5);

    // printf("Statusword:0x%x, Position_actual_value:0x%x, Current_actual_value:0x%x\r\n",
    //     Statusword, Position_actual_value, Current_actual_value);
}

void Kinco_SetVel_PDO(uint32_t vel)
{
    uint32_t size = 4;
    Profile_velocity = (uint32_t)(vel * (KINCO_RESOLUTION * KINCO_RPM_TO_DEC_MULT));
    writeLocalDict(&Kinco_Ctrl_Data, 0x2002, 0x00, &Profile_velocity, (UNS32*)&size, RW);
    sendPDOevent(&Kinco_Ctrl_Data);
    sendSYNC(&Kinco_Ctrl_Data);
    // print_can1_recv_msg();
}

int Kinco_Read_Error_SDO(uint16_t* p_error_code)
{
    uint16_t error_code = 0;
    int result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, ERROR_CODE_INDEX, 0x00, &error_code, uint16);
    if (result == SDO_OK)
    {
        *p_error_code = error_code;
    }
    else
    {
        printf("Kinco SDO read ERROR_CODE_INDEX Failed\r\n");
    }
    return result;
}

int Kinco_Read_ActuclVel_SDO(uint32_t* p_actual_vel)
{
    uint32_t vel = 0;
    int result = SDO_ReadRequest(&Kinco_Ctrl_Data, KINCO_SLAVE_NODE_ID, ACTUAL_VELOCITY_INDEX, 0x00, &vel, uint32);
    if (result == SDO_OK)
    {
        *p_actual_vel = vel;
    }
    else
    {
        printf("Kinco SDO read ACTUAL_VELOCITY_INDEX Failed\r\n");
    }
    return result;
}

void ZeroErr_MasterNode_Init(void)
{
    setNodeId(&ZeroErr_Ctrl_Data, ZEROERR_MASTER_NODE_ID);
    ZeroErr_Ctrl_Data.canHandle = 0x02;
    setState(&ZeroErr_Ctrl_Data, Initialisation);
    setState(&ZeroErr_Ctrl_Data, Pre_operational);
    setState(&ZeroErr_Ctrl_Data, Operational);

    masterSendNMTstateChange(&ZeroErr_Ctrl_Data, 0x01, NMT_Stop_Node);
    // Step 1: Reset Communication
    masterSendNMTstateChange(&ZeroErr_Ctrl_Data, 0x01, NMT_Reset_Comunication);
    osDelay(200); // 等待200ms，保证从站复位完成

    // Step 2: Enter Pre - Operational
    masterSendNMTstateChange(&ZeroErr_Ctrl_Data, 0x01, NMT_Enter_PreOperational);
    osDelay(50);
}

void ZeroErr_Setup(void)
{
    int result = 0;
    uint8_t work_mode = 1;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_WORK_MODE_INDEX, 0x00, &work_mode, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write SERVO_WORK_MODE_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write SERVO_WORK_MODE_INDEX Failed\r\n");
    }

    work_mode = 0;
    result = SDO_ReadRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_WORK_MODE_INDEX, 0x00, &work_mode, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO read SERVO_WORK_MODE_INDEX Success,work_mode:0x%x\r\n", work_mode);
    }
    else
    {
        printf("ZeroErr SDO read SERVO_WORK_MODE_INDEX Failed\r\n");
    }
    if (work_mode == POSITION_MODE)
    {
        // printf("ZeroErr set position mode Success\r\n");
    }
    else
    {
        printf("ZeroErr set position mode Failed\r\n");
    }

    uint32_t set_val_u32 = (uint32_t)(ZEROERR_DEFAULT_PROFILED_VELOCITY * ZEROERR_DPS_TO_DEC_MULT);
    uint32_t profiled_val = set_val_u32;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, PROFILED_VELOCITY_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("Zeroerr SDO Write PROFILED_VELOCITY_INDEX Success\r\n");
    }
    else
    {
        printf("Zeroerr SDO Write PROFILED_VELOCITY_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, PROFILED_VELOCITY_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("Zeroerr SDO read PROFILED_VELOCITY_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("Zeroerr SDO read PROFILED_VELOCITY_INDEX Failed\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        // printf("Zeroerr set profiled velocity to %d dps Success\r\n", ZEROERR_DEFAULT_PROFILED_VELOCITY);
    }
    else
    {
        printf("Zeroerr set profiled velocity to %d dps Failed\r\n", ZEROERR_DEFAULT_PROFILED_VELOCITY);
    }

    set_val_u32 = (uint32_t)(ZEROERR_DEFAULT_PROFILED_ACC * ZEROERR_ACC_DEC_MULT);
    profiled_val = set_val_u32;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, PROFILED_ACC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write PROFILED_ACC_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write PROFILED_ACC_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, PROFILED_ACC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO read PROFILED_ACC_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("ZeroErr SDO read PROFILED_ACC_INDEX Failed\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        // printf("ZeroErr set profiled acceleration to %d dpss Success\r\n", KINCO_DEFAULT_PROFILED_ACC);
    }
    else
    {
        printf("ZeroErr set profiled acceleration to %d dpss Failed\r\n", ZEROERR_DEFAULT_PROFILED_ACC);
    }

    set_val_u32 = (uint32_t)(ZEROERR_DEFAULT_PROFILED_DEC * ZEROERR_ACC_DEC_MULT);
    profiled_val = set_val_u32;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, PROFILED_DEC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write PROFILED_DEC_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write PROFILED_DEC_INDEX Failed\r\n");
    }

    profiled_val = 0;
    result = SDO_ReadRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, PROFILED_DEC_INDEX, 0x00, &profiled_val, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO read PROFILED_DEC_INDEX Success,profiled_val:0x%x\r\n", profiled_val);
    }
    else
    {
        printf("ZeroErr SDO read PROFILED_DEC_INDEX Failed\r\n");
    }

    if (profiled_val == set_val_u32)
    {
        // printf("ZeroErr set profiled deceleration to %d dpss Success\r\n", KINCO_DEFAULT_PROFILED_DEC);
    }
    else
    {
        printf("ZeroErr set profiled deceleration to %d dpss Failed\r\n", ZEROERR_DEFAULT_PROFILED_DEC);
    }

    set_val_u32 = 0x80;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, 0x1005, 0, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
    }
    else
    {
        printf("ZeroErr SDO disable SYNC Failed\r\n");
    }

    set_val_u32 = 1000;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, 0x1006, 0, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
    }
    else
    {
        printf("ZeroErr SDO disable comm cycle period Failed\r\n");
    }

    set_val_u32 = 0x80000180 + ZEROERR_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Failed\r\n");
    }

    uint8_t set_val_u8 = 0x01;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x00;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60410010;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60640020;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x02, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u8 = 0x02;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x00000180 + ZEROERR_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Failed\r\n");
    }

    // 禁用 TPDO2
    set_val_u32 = 0x80000181 + ZEROERR_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO2_PARAM_INDEX Failed\r\n");
    }

    // 设置传输类型 = 1（同步发送）
    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO2_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO2_PARAM_INDEX Failed\r\n");
    }

    // 清空映射
    set_val_u8 = 0x00;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO2_MAPPING_INDEX Failed\r\n");
    }

    // 映射第1个对象：0x606C:00（32-bit）
    set_val_u32 = 0x606C0020;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO2_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO2_MAPPING_INDEX Failed\r\n");
    }

    // 映射条目数 = 1
    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO2_MAPPING_INDEX Failed\r\n");
    }

    // 重新启用 TPDO2（COB-ID = 0x181 + NodeID）
    set_val_u32 = 0x00000181 + ZEROERR_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, TPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write TPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u32 = 0x80000200 + ZEROERR_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x1;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60400010;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x607A0020;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x02, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u8 = 0x02;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO1_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO1_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO1_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x200 + ZEROERR_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO1_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO1_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO1_PARAM_INDEX Failed\r\n");
    }

    set_val_u32 = 0x80000201 + ZEROERR_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x02, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO2_PARAM_INDEX Failed\r\n");
    }

    set_val_u8 = 0x00;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO2_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x60810020;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO2_MAPPING_INDEX Failed\r\n");
    }

    set_val_u8 = 0x01;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO2_MAPPING_INDEX, 0x00, &set_val_u8, uint8);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO2_MAPPING_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO2_MAPPING_INDEX Failed\r\n");
    }

    set_val_u32 = 0x201 + ZEROERR_SLAVE_NODE_ID;
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, RPDO2_PARAM_INDEX, 0x01, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO2_PARAM_INDEX Failed\r\n");
    }
}

int ZeroErr_Enable_PDO(void)
{
    if (zeroerr_lock)
    {
        return ENABLE_BUSY;
    }

    for (int i = 0; i < 10; i++)
    {
        Get_Parse_StatusWord(1);
        if (zeroerr_curr_status != INVALID) {
            break;
        }
    }

    if (zeroerr_curr_status == INVALID) {
        printf("ZeroErr enable failed, GET_STATUS_FAILED.\r\n");
        zeroerr_lock = false;
        return GET_STATUS_FAILED;
    }
    else if (zeroerr_curr_status == SWITCH_ON_DISABLED) {
        // printf("ZeroErr current status is SWITCH_ON_DISABLED, continue...\r\n");
    }
    else if (zeroerr_curr_status == OPERATION_ENABLED)
    {
        printf("ZeroErr current status is OPERATION_ENABLED\r\n");
        return ENABLE_OK;
    }

    uint32_t size;
    ctrl_word_zeroerr = 0x06;
    size = 2;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2000, 0x00, &ctrl_word_zeroerr, (UNS32*)&size, RW);
    target_pos_zeroerr = 0x0000;
    size = 4;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2001, 0x00, &target_pos_zeroerr, (UNS32*)&size, RW);
    sendPDOevent(&ZeroErr_Ctrl_Data);
    for (int i = 0; i < 10; i++) {
        Get_Parse_StatusWord(1);
        if (zeroerr_curr_status == READY_TO_SWITCH_ON) {
            break;
        }
    }

    if (zeroerr_curr_status != READY_TO_SWITCH_ON) {
        printf("ZeroErr enable failed, READY_SWITCH_ON_FAILED.\r\n");
        zeroerr_lock = false;
        return READY_SWITCH_ON_FAILED;
    }
    // else {
    //     printf("ZeroErr current status is READY_TO_SWITCH_ON, continue...\r\n");
    // }
    osDelay(5);

    ctrl_word_zeroerr = 0x07;
    size = 2;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2000, 0x00, &ctrl_word_zeroerr, (UNS32*)&size, RW);
    sendPDOevent(&ZeroErr_Ctrl_Data);
    for (int i = 0; i < 10; i++)
    {
        Get_Parse_StatusWord(1);
        if (zeroerr_curr_status == SWITCHED_ON) {
            break;
        }
    }

    if (zeroerr_curr_status != SWITCHED_ON) {
        printf("ZeroErr enable failed, SWITCHED_ON_FAILED.\r\n");
        zeroerr_lock = false;
        return SWITCHED_ON_FAILED;
    }
    // else {
    //     printf("ZeroErr current status is SWITCHED_ON, continue...\r\n");
    // }
    osDelay(5);

    ctrl_word_zeroerr = 0x0F;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2000, 0x00, &ctrl_word_zeroerr, (UNS32*)&size, RW);
    sendPDOevent(&ZeroErr_Ctrl_Data);
    for (int i = 0; i < 10; i++)
    {
        osDelay(50);
        Get_Parse_StatusWord(1);
        if (zeroerr_curr_status == OPERATION_ENABLED) {
            break;
        }
    }

    if (zeroerr_curr_status != OPERATION_ENABLED) {
        printf("ZeroErr enable failed, OPERATION_ENABLED_FAILED.\r\n");
        zeroerr_lock = false;
        return OPERATION_ENABLED_FAILED;
    }
    else {
        printf("ZeroErr current status is OPERATION_ENABLED, enable completed.\r\n");
    }

    zeroerr_lock = false;
    return ENABLE_OK;
}

int ZeroErr_Disable_PDO(void)
{
    if (zeroerr_lock)
    {
        return DISABLE_BUSY;
    }
    uint32_t size = 2;
    ctrl_word_zeroerr = 0x06;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2000, 0x00, &ctrl_word_zeroerr, (uint32_t*)&size, RW);
    sendPDOevent(&ZeroErr_Ctrl_Data);
    for (int i = 0; i < 10; i++) {
        osDelay(50);
        Get_Parse_StatusWord(1);
        if (zeroerr_curr_status == READY_TO_SWITCH_ON) {
            break;
        }
    }

    if (zeroerr_curr_status != READY_TO_SWITCH_ON) {
        printf("ZeroErr enable failed, READY_SWITCH_ON_FAILED.\r\n");
        zeroerr_lock = false;
        return READY_SWITCH_ON_FAILED;
    }
    // else {
    //     printf("ZeroErr current status is READY_TO_SWITCH_ON, continue...\r\n");
    // }
    osDelay(5);

    ctrl_word_zeroerr = 0x00;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2000, 0x00, &ctrl_word_zeroerr, (uint32_t*)&size, RW);
    sendPDOevent(&ZeroErr_Ctrl_Data);
    for (int i = 0; i < 10; i++) {
        osDelay(100);
        Get_Parse_StatusWord(1);
        if (zeroerr_curr_status == SWITCH_ON_DISABLED) {
            break;
        }
    }

    if (zeroerr_curr_status != SWITCH_ON_DISABLED) {
        printf("ZeroErr enable failed, SWITCH_ON_DISABLED_FAILED.\r\n");
        zeroerr_lock = false;
        return SWITCH_ON_DISABLED_FAILED;
    }
    else {
        printf("ZeroErr current status is SWITCH_ON_DISABLED, disable completed.\r\n");
        return DISABLE_OK;
    }
}

void ZeroErr_Set_QuickStop_option(uint16_t option)
{
    uint16_t set_val_u16 = option;
    int result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, STOP_OPTION_INDEX, 0x00, &set_val_u16, uint16);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write RPDO2_PARAM_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write RPDO2_PARAM_INDEX Failed\r\n");
    }
}

void ZeroErr_MovPos_PDO(float degree)
{
    uint32_t size;
    ctrl_word_zeroerr = 0x2F;
    size = 2;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2000, 0x00, &ctrl_word_zeroerr, (UNS32*)&size, RW);
    target_pos_zeroerr = (uint32_t)(degree * ZEROERR_RESOLUTION / 360);
    // printf("target_pos:0x%x\r\n", target_pos_zeroerr);
    size = 4;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2001, 0x00, &target_pos_zeroerr, (UNS32*)&size, RW);
    sendPDOevent(&ZeroErr_Ctrl_Data);

    ctrl_word_zeroerr = 0x3F;
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2000, 0x00, &ctrl_word_zeroerr, (UNS32*)&size, RW);
    sendPDOevent(&ZeroErr_Ctrl_Data);
    sendSYNC(&ZeroErr_Ctrl_Data);
    // print_can2_recv_msg();
    osDelay(5);

    // printf("status_word_error:0x%x, Position_actual_value:0x%x\r\n",
    //     status_word_zeroerr, pos_actual_val_zeroerr);
}

void ZeroErr_SetVel_PDO(uint32_t vel)
{
    uint32_t size = 4;
    target_vel_zeroerr = (uint32_t)(vel * ZEROERR_DPS_TO_DEC_MULT);
    writeLocalDict(&ZeroErr_Ctrl_Data, 0x2002, 0x00, &target_vel_zeroerr, (UNS32*)&size, RW);
    sendPDOevent(&ZeroErr_Ctrl_Data);
    sendSYNC(&ZeroErr_Ctrl_Data);
    // print_can2_recv_msg();
}

void ZeroErr_SetVel_SDO(uint32_t vel)
{
    uint32_t set_val_u32 = (uint32_t)(vel * ZEROERR_DPS_TO_DEC_MULT);
    int result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, PROFILED_VELOCITY_INDEX, 0x00, &set_val_u32, uint32);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("Zeroerr SDO Write PROFILED_VELOCITY_INDEX Success\r\n");
    }
    else
    {
        printf("Zeroerr SDO Write PROFILED_VELOCITY_INDEX Failed\r\n");
    }
}

void ZeroErr_QuickStop_SDO(void)
{
    uint16_t cur_ctrl_word = 0;
    int result = SDO_ReadRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_CTRL_WORD_INDEX, 0x00, &cur_ctrl_word, uint16);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("Zeroerr SDO read SERVO_CTRL_WORD_INDEX Success,cur_ctrl_word:0x%x\r\n", cur_ctrl_word);
    }
    else
    {
        printf("Zeroerr SDO read SERVO_CTRL_WORD_INDEX Failed\r\n");
    }

    cur_ctrl_word &= ~(1 << 2);
    // printf("cur_ctrl_word:0x%x\r\n", cur_ctrl_word);
    result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_CTRL_WORD_INDEX, 0x00, &cur_ctrl_word, uint16);
    if (result == SDO_OK)
    {
        // print_can2_recv_msg();
        // printf("ZeroErr SDO Write SERVO_CTRL_WORD_INDEX Success\r\n");
    }
    else
    {
        printf("ZeroErr SDO Write SERVO_CTRL_WORD_INDEX Failed\r\n");
    }
}

void ZeroErr_Clr_Fault_CWord_SDO(void)
{
    uint32_t cur_ctrl_word = 0x80;
    int result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_CTRL_WORD_INDEX, 0x00, &cur_ctrl_word, uint32);
}

void ZeroErr_Clr_Zero_CWord_SDO(void)
{
    uint32_t cur_ctrl_word = 0x00;
    int result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_CTRL_WORD_INDEX, 0x00, &cur_ctrl_word, uint32);
}

void ZeroErr_ShutDown_CWord_SDO(void)
{
    uint32_t cur_ctrl_word = 0x06;
    int result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_CTRL_WORD_INDEX, 0x00, &cur_ctrl_word, uint32);
}

void ZeroErr_SwitchOn_CWord_SDO(void)
{
    uint32_t cur_ctrl_word = 0x07;
    int result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_CTRL_WORD_INDEX, 0x00, &cur_ctrl_word, uint32);
}

void ZeroErr_EnOper_CWord_SDO(void)
{
    uint32_t cur_ctrl_word = 0x0F;
    int result = SDO_WriteRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, SERVO_CTRL_WORD_INDEX, 0x00, &cur_ctrl_word, uint32);
}

bool ZeroErr_QuickStop_Resume_SDO(void)
{
    Get_Parse_StatusWord(1);
    if (zeroerr_curr_status == FAULT ||
        zeroerr_curr_status == FAULT_REACTION_ACT)
    {
        ZeroErr_Clr_Fault_CWord_SDO();
        osDelay(10);
        ZeroErr_Clr_Zero_CWord_SDO();
        osDelay(10);
        if (!ZeroErr_Wait_Status(SWITCH_ON_DISABLED))
        {
            printf("zeroerr FAULT_RESET failed\r\n");
            return false;
        }
    }

    ZeroErr_ShutDown_CWord_SDO();
    if (!ZeroErr_Wait_Status(READY_TO_SWITCH_ON))
    {
        printf("zeroerr READY_TO_SWITCH_ON failed\r\n");
        return false;
    }

    ZeroErr_SwitchOn_CWord_SDO();
    if (!ZeroErr_Wait_Status(SWITCHED_ON))
    {
        printf("zeroerr SWITCHED_ON failed\r\n");
        return false;
    }

    ZeroErr_EnOper_CWord_SDO();
    if (!ZeroErr_Wait_Status(OPERATION_ENABLED))
    {
        printf("zeroerr OPERATION_ENABLED failed\r\n");
        return false;
    }

    printf("zeroerr RESUME ok\r\n");
    return true;
}

bool ZeroErr_Wait_Status(Servo_Status_t target_state)
{
    for (int i = 0; i < 50; i++)
    {
        Get_Parse_StatusWord(1);
        if (zeroerr_curr_status == target_state)
        {
            return true;
        }
        osDelay(5);
    }
    return false;
}

int ZeroErr_Read_Error_SDO(uint16_t* p_error_code)
{
    uint16_t error_code = 0;
    int result = SDO_ReadRequest(&ZeroErr_Ctrl_Data, ZEROERR_SLAVE_NODE_ID, ERROR_CODE_INDEX, 0x00, &error_code, uint16);
    if (result == SDO_OK)
    {
        *p_error_code = error_code;
    }
    else
    {
        printf("Zeroerr SDO read ERROR_CODE_INDEX Failed\r\n");
        return SDO_ERR_SEND;
    }
    return SDO_OK;
}

int ZeroErr_Read_ActuclVel_SDO(uint32_t* p_actual_vel)
{
    uint32_t vel = 0;
    int result = SDO_ReadRequest(&ZeroErr_Ctrl_Data, KINCO_SLAVE_NODE_ID, ACTUAL_VELOCITY_INDEX, 0x00, &vel, uint32);
    if (result == SDO_OK)
    {
        *p_actual_vel = vel;
    }
    else
    {
        printf("Zeroerr SDO read ACTUAL_VELOCITY_INDEX Failed\r\n");
    }
    return result;
}

void Get_Parse_StatusWord(uint8_t servo_type)
{
    if (servo_type == 0) {
        sendSYNC(&Kinco_Ctrl_Data);
        // print_can1_recv_msg();
        osDelay(1);
        if ((Statusword & 0x004F) == 0x0000) {
            // printf("Kinco State: Not ready to switch on\r\n");
            kinco_curr_status = NOT_READY_TO_SWITCH_ON;
        }
        else if ((Statusword & 0x004F) == 0x0040) {
            // printf("Kinco State: Switch on disabled\r\n");
            kinco_curr_status = SWITCH_ON_DISABLED;
        }
        else if ((Statusword & 0x006F) == 0x0021) {
            // printf("Kinco State: Ready to switch on\r\n");
            kinco_curr_status = READY_TO_SWITCH_ON;
        }
        else if ((Statusword & 0x006F) == 0x0023) {
            // printf("Kinco State: Switched on\r\n");
            kinco_curr_status = SWITCHED_ON;
        }
        else if ((Statusword & 0x006F) == 0x0027) {
            // printf("Kinco State: Operation enabled\r\n");
            kinco_curr_status = OPERATION_ENABLED;
        }
        else if ((Statusword & 0x006F) == 0x0007) {
            // printf("Kinco State: Quick stop active\r\n");
            kinco_curr_status = QUICK_STOP_ACT;
        }
        else if ((Statusword & 0x004F) == 0x000F) {
            // printf("Kinco State: Fault reaction active\r\n");
            kinco_curr_status = FAULT_REACTION_ACT;
        }
        else if ((Statusword & 0x004F) == 0x0008) {
            // printf("Kinco State: Fault\r\n");
            kinco_curr_status = FAULT;
        }
    }
    else
    {
        sendSYNC(&ZeroErr_Ctrl_Data);
        // print_can2_recv_msg();
        osDelay(1);
        if ((status_word_zeroerr & 0x004F) == 0x0000) {
            // printf("ZeroErr State: Not ready to switch on\r\n");
            zeroerr_curr_status = NOT_READY_TO_SWITCH_ON;
        }
        else if ((status_word_zeroerr & 0x004F) == 0x0040) {
            // printf("ZeroErr State: Switch on disabled\r\n");
            zeroerr_curr_status = SWITCH_ON_DISABLED;
        }
        else if ((status_word_zeroerr & 0x006F) == 0x0021) {
            // printf("ZeroErr State: Ready to switch on\r\n");
            zeroerr_curr_status = READY_TO_SWITCH_ON;
        }
        else if ((status_word_zeroerr & 0x006F) == 0x0023) {
            // printf("ZeroErr State: Switched on\r\n");
            zeroerr_curr_status = SWITCHED_ON;
        }
        else if ((status_word_zeroerr & 0x006F) == 0x0027) {
            // printf("ZeroErr State: Operation enabled\r\n");
            zeroerr_curr_status = OPERATION_ENABLED;
        }
        else if ((status_word_zeroerr & 0x006F) == 0x0007) {
            // printf("ZeroErr State: Quick stop active\r\n");
            zeroerr_curr_status = QUICK_STOP_ACT;
        }
        else if ((status_word_zeroerr & 0x004F) == 0x000F) {
            // printf("ZeroErr State: Fault reaction active\r\n");
            zeroerr_curr_status = FAULT_REACTION_ACT;
        }
        else if ((status_word_zeroerr & 0x004F) == 0x0008) {
            // printf("ZeroErr: Fault\r\n");
            zeroerr_curr_status = FAULT;
        }
    }
}

Servo_Status_t Get_Curr_Status(uint8_t servo_type)
{
    if (servo_type == 0)
    {
        return kinco_curr_status;
    }
    else
    {
        return zeroerr_curr_status;
    }
}
