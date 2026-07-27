#include "data_manager.h"
#include "sysConfig.h"
/*------------------------------------------------------------------------------
 * 定义两个全局数据缓冲区，实现双缓冲机制
 *
 * buffer1 和 buffer2 分别作为当前的读缓存和写缓存。
 * 数据采集或处理线程将新数据写入 pWriteBuffer，提交时通过临界区交换指针，
 * 使得 pReadBuffer 总是提供一份完整、一致的全局数据快照。
 *------------------------------------------------------------------------------*/
static GlobalData_t buffer1;
static GlobalData_t buffer2;

/*------------------------------------------------------------------------------
 * 指向当前只读和写缓冲区的指针
 * 初始状态下：
 *  - pReadBuffer 指向 buffer1（供外部读取）
 *  - pWriteBuffer 指向 buffer2（供内部数据写入）
 *------------------------------------------------------------------------------*/
static GlobalData_t *pReadBuffer = &buffer1;
static GlobalData_t *pWriteBuffer = &buffer2;

void DataManager_Init(void)
{
    uint8_t i;
    /* 初始化电机数据 */
    for(i = 0; i<MOTOR_COUNT; i++) {
        buffer1.motorSpeedMea[i]    = 0.0f;
        buffer1.motorPositionMea[i] = 0.0f;
        buffer1.motorCurrentMea[i]  = 0.0f;
        buffer1.motorSpeedRef[i]    = 0.0f;
        buffer1.motorPositionRef[i] = 0.0f;
        buffer1.motorCurrentRef[i]  = 0.0f;	
        buffer1.angleCmd[i]        = 0.0f;
        buffer1.speedCmd[i]       = 0.0f;
        buffer1.speedTar[i]       = 0.0f;	
        buffer1.angleDu[i]       = 0.0f;
        buffer1.angleSudu[i]       = 0.0f;		
		
        buffer2.motorSpeedMea[i]    = 0.0f;
        buffer2.motorPositionMea[i] = 0.0f;
        buffer2.motorCurrentMea[i]  = 0.0f;
        buffer2.motorSpeedRef[i]    = 0.0f;
        buffer2.motorPositionRef[i] = 0.0f;
        buffer2.motorCurrentRef[i]  = 0.0f;
        buffer2.angleCmd[i]        = 0.0f;
        buffer2.speedCmd[i]       = 0.0f;
        buffer2.speedTar[i]       = 0.0f;
        buffer2.angleDu[i]       = 0.0f;
        buffer2.angleSudu[i]       = 0.0f;
    }
    /* 初始化其他数据 */
    buffer1.temperatureMea = 0.0f;
    buffer2.temperatureMea = 0.0f;
    
    buffer1.communicationStatus = 0;
    buffer2.communicationStatus = 0;
}

const GlobalData_t* DataManager_GetReadBuffer(void)
{
    return pReadBuffer;
}

GlobalData_t* DataManager_GetWriteBuffer(void)
{
    return pWriteBuffer;
}

void DataManager_Commit(void)
{
	/* 进入临界区，确保指针交换的原子性 */
	taskENTER_CRITICAL();
	{
		GlobalData_t *temp = pReadBuffer;
		pReadBuffer = pWriteBuffer;
		pWriteBuffer = temp;
	}
	taskEXIT_CRITICAL();
}

/*------------------------------------------------------------------------------
 * 电机电流数据操作接口
 *------------------------------------------------------------------------------*/
/*更新单个电机的“电流”“测量”值*/
void DataManager_SetMotorCurrentMea(uint8_t motorIndex, float currentVal)
{
    if(motorIndex < MOTOR_COUNT) {
        pWriteBuffer->motorCurrentMea[motorIndex] = currentVal;
    }
}

/*更新单个电机的“电流”“参考”值*/
void DataManager_SetMotorCurrentRef(uint8_t motorIndex, float currentVal)
{
    if(motorIndex < MOTOR_COUNT) {
        pWriteBuffer->motorCurrentRef[motorIndex] = currentVal;
    }
}

/*获取单个电机的“电流”“测量”值*/
float DataManager_GetMotorCurrentMea(uint8_t motorIndex)
{
    if(motorIndex < MOTOR_COUNT) {
        return pReadBuffer->motorCurrentMea[motorIndex];
    }
    return 0.0f;
}

/*获取单个电机的“电流”“参考”值*/
float DataManager_GetMotorCurrentRef(uint8_t motorIndex)
{
    if(motorIndex < MOTOR_COUNT) {
        return pReadBuffer->motorCurrentRef[motorIndex];
    }
    return 0.0f;
}

/*更新所有电机的“电流”“测量”值*/
void DataManager_SetAllMotorCurrentsMea(const float *pCurrents, uint8_t count)
{
    if (pCurrents == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->motorCurrentMea[i] = pCurrents[i];
    }
}

/*更新所有电机的“电流”“参考”值*/
void DataManager_SetAllMotorCurrentsRef(const float *pCurrents, uint8_t count)
{
	if(pCurrents == NULL) 
	{
		return; // 不做处理
	}
	if(count>MOTOR_COUNT) 
	{
		count = MOTOR_COUNT; // 避免越界
	}
	for(uint8_t i = 0; i < count; i++) 
	{
		pWriteBuffer->motorCurrentRef[i] = pCurrents[i];
	}
}

/*获取所有电机的“电流”“测量”值*/
void DataManager_GetAllMotorCurrentsMea(float *pOutCurrents, uint8_t count)
{
    if (pOutCurrents == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pOutCurrents[i] = pReadBuffer->motorCurrentMea[i];
    }
}

/*获取所有电机的“电流”“参考”值*/
void DataManager_GetAllMotorCurrentsRef(float *pOutCurrents, uint8_t count)
{
    if (pOutCurrents == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pOutCurrents[i] = pReadBuffer->motorCurrentRef[i];
    }
}

/*------------------------------------------------------------------------------
 * 电机速度数据操作接口
 *------------------------------------------------------------------------------*/
/*更新单个电机的“速度”“测量”值*/
void DataManager_SetMotorSpeedMea(uint8_t motorIndex, float speed)
{
    if(motorIndex < MOTOR_COUNT) {
        pWriteBuffer->motorSpeedMea[motorIndex] = speed;
    }
}

/*更新单个电机的“速度”“参考”值*/
void DataManager_SetMotorSpeedRef(uint8_t motorIndex, float speed)
{
    if(motorIndex < MOTOR_COUNT) {
        pWriteBuffer->motorSpeedRef[motorIndex] = speed;
    }
}

/*获取单个电机的“速度”“测量”值*/
float DataManager_GetMotorSpeedMea(uint8_t motorIndex)
{
    if(motorIndex < MOTOR_COUNT) {
        return pReadBuffer->motorSpeedMea[motorIndex];
    }
    return 0.0f;
}

/*获取单个电机的“速度”“参考”值*/
float DataManager_GetMotorSpeedRef(uint8_t motorIndex)
{
    if(motorIndex < MOTOR_COUNT) {
        return pReadBuffer->motorSpeedRef[motorIndex];
    }
    return 0.0f;
}

/*更新所有电机的“速度”“测量”值*/
void DataManager_SetAllMotorSpeedsMea(const float *pSpeeds, uint8_t count)
{
    if (pSpeeds == NULL) {
        return;
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT;
    }
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->motorSpeedMea[i] = pSpeeds[i];
    }
}

/*更新所有电机的“速度”“参考”值*/
void DataManager_SetAllMotorSpeedsRef(const float *pSpeeds, uint8_t count)
{
    if (pSpeeds == NULL) {
        return;
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT;
    }
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->motorSpeedRef[i] = pSpeeds[i];
    }
}

/*获取所有电机的“速度”“测量”值*/
void DataManager_GetAllMotorSpeedsMea(float *pOutSpeeds, uint8_t count)
{
    if (pOutSpeeds == NULL) {
        return;
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT;
    }
    for (uint8_t i = 0; i < count; i++) {
        pOutSpeeds[i] = pReadBuffer->motorSpeedMea[i];
    }
}

/*获取所有电机的“速度”“参考”值*/
void DataManager_GetAllMotorSpeedsRef(float *pOutSpeeds, uint8_t count)
{
    if (pOutSpeeds == NULL) {
        return;
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT;
    }
    for (uint8_t i = 0; i < count; i++) {
        pOutSpeeds[i] = pReadBuffer->motorSpeedRef[i];
    }
}

/*------------------------------------------------------------------------------
 * 电机位置数据操作接口
 *------------------------------------------------------------------------------*/
/*更新单个电机的“位置”“测量”值*/
void DataManager_SetMotorPositionMea(uint8_t motorIndex, float position)
{
    if(motorIndex < MOTOR_COUNT) {
        pWriteBuffer->motorPositionMea[motorIndex] = position;
    }
}

/*更新单个电机的“位置”“参考”值*/
void DataManager_SetMotorPositionRef(uint8_t motorIndex, float position)
{
    if(motorIndex < MOTOR_COUNT) {
        pWriteBuffer->motorPositionRef[motorIndex] = position;
    }
}

/*获取单个电机的“位置”“测量”值*/
float DataManager_GetMotorPositionMea(uint8_t motorIndex)
{
    if(motorIndex < MOTOR_COUNT) {
        return pReadBuffer->motorPositionMea[motorIndex];
    }
    return 0.0f;
}

/*获取单个电机的“位置”“参考”值*/
float DataManager_GetMotorPositionRef(uint8_t motorIndex)
{
    if(motorIndex < MOTOR_COUNT) {
        return pReadBuffer->motorPositionRef[motorIndex];
    }
    return 0.0f;
}

/*更新所有电机的“位置”“测量”值*/
void DataManager_SetAllMotorPositionsMea(const volatile float *pPositions, uint8_t count)
{
    if (pPositions == NULL) {
        return;
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT;
    }
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->motorPositionMea[i] = pPositions[i];
    }
}

/*更新所有电机的“位置”“参考”值*/
void DataManager_SetAllMotorPositionsRef(const volatile float *pPositions, uint8_t count)
{
    if (pPositions == NULL) {
        return;
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT;
    }
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->motorPositionRef[i] = pPositions[i];
    }
}

/*获取所有电机的“位置”“测量”值*/
void DataManager_GetAllMotorPositionsMea(float *pOutPositions, uint8_t count)
{
    if (pOutPositions == NULL) {
        return;
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT;
    }
    for (uint8_t i = 0; i < count; i++) {
        pOutPositions[i] = pReadBuffer->motorPositionMea[i];
    }
}

/*获取所有电机的“位置”“参考”值*/
void DataManager_GetAllMotorPositionsRef(float *pOutPositions, uint8_t count)
{
    if (pOutPositions == NULL) {
        return;
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT;
    }
    for (uint8_t i = 0; i < count; i++) {
        pOutPositions[i] = pReadBuffer->motorPositionRef[i];
    }
}
/*------------------------------------------------------------------------------
 * 温度数据操作接口
 *------------------------------------------------------------------------------*/
/*更新“温度”“测量”值*/
void DataManager_SetTemperatureMea(float temp)
{
    pWriteBuffer->temperatureMea = temp;
}

/*获取“温度”“测量”值*/
float DataManager_GetTemperatureMea(void)
{
    return pReadBuffer->temperatureMea;
}



 //用户速度数据操作接口
void DataManager_SetSpeedCmdAll(const float *pSpeeds, uint8_t count)
{
    if (pSpeeds == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->speedCmd[i] = pSpeeds[i];
    }
}

void DataManager_GetSpeedCmdAll(float *pOutSpeeds, uint8_t count)
{
    if (pOutSpeeds == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pOutSpeeds[i] = pReadBuffer->speedCmd[i];
    }
}

//用户角度数据操作接口
void DataManager_SetAngleCmdAll(const float *pAngles, uint8_t count)
{
    if (pAngles == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->angleCmd[i] = pAngles[i];
    }
}

void DataManager_GetAngleCmdAll(float *pOutAngles, uint8_t count)
{
    if (pOutAngles == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pOutAngles[i] = pReadBuffer->angleCmd[i];
    }
}

//模型速度目标数据操作接口
void DataManager_SetSpeedTarAll(const float *pSpeeds, uint8_t count)
{
    if (pSpeeds == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->speedTar[i] = pSpeeds[i];
    }
}

void DataManager_GetSpeedTarAll(float *pOutSpeeds, uint8_t count)
{
    if (pOutSpeeds == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pOutSpeeds[i] = pReadBuffer->speedTar[i];
    }
}


//模型角度目标数据操作接口
void DataManager_SetAngleDuAll(const float *pAngles, uint8_t count)
{
    if (pAngles == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->angleDu[i] = pAngles[i];
    }
}
void DataManager_GetAngleDuAll(float *pOutAngles, uint8_t count)
{
    if (pOutAngles == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pOutAngles[i] = pReadBuffer->angleDu[i];
    }
}

//模型速度目标数据操作接口
void DataManager_SetAngleSuduAll(const float *pSpeeds, uint8_t count)
{
    if (pSpeeds == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pWriteBuffer->angleSudu[i] = pSpeeds[i];
    }
}

void DataManager_GetAngleSuduAll(float *pOutSpeeds, uint8_t count)
{
    if (pOutSpeeds == NULL) {
        return; // 不做处理
    }
    if (count > MOTOR_COUNT) {
        count = MOTOR_COUNT; // 避免越界
    }
    
    for (uint8_t i = 0; i < count; i++) {
        pOutSpeeds[i] = pReadBuffer->angleSudu[i];
    }
}


/*------------------------------------------------------------------------------
 * 若需要添加其它数据（如通讯数据等）的操作接口，可遵循类似模式：
 *
 * - 写操作：操作 pWriteBuffer 的对应成员
 * - 读操作：从 pReadBuffer 中读取对应成员
 *
 * 注意：所有数据更新应在所有写操作完成后调用 DataManager_Commit，
 *       以保证读到的数据是一份完整、一致的快照。
 *------------------------------------------------------------------------------*/
