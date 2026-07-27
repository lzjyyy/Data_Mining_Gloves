#ifndef DATA_MANAGER_H
#define DATA_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include "sysConfig.h"
/* -----------------------------------------------------------------------------------------------变量声明 */

/**
 * @brief 全局数据结构，用于存储整个系统的实时数据
 *
 * 除了电机速度、位置、电流外，还包括温度、通讯数据等信息。
 */
typedef struct {
    float motorSpeedMea[MOTOR_COUNT];     /**< 电机速度数据 */
    float motorPositionMea[MOTOR_COUNT];  /**< 电机位置数据 */
    float motorCurrentMea[MOTOR_COUNT];   /**< 电机电流数据 */
    float temperatureMea;                /**< 系统温度数据 */
		float motorSpeedRef[MOTOR_COUNT];		 //电机目标速度(最终给到速度环的)
		float motorPositionRef[MOTOR_COUNT];  //电机目标位置(模型计算过后的)
		float motorCurrentRef[MOTOR_COUNT];   //电机目标电流(由RS485通讯得到)
    float angleCmd[MOTOR_COUNT];        /** 用户给出的角度命令，单位度 */
    float speedCmd[MOTOR_COUNT];          /** 用户给出的速度，单位度/秒 */
    float speedTar[MOTOR_COUNT];          /** 经过运动学解算的速度目标值 */
    float angleDu[MOTOR_COUNT];          /** 模型计算电机角度，单位度 */
    float angleSudu[MOTOR_COUNT];       /** 模型计算电机速度，单位度/秒 */
    uint32_t communicationStatus;     /**< 通讯状态数据 */
} GlobalData_t;
/* -----------------------------------------------------------------------------------------------基础接口：初始化、读取写缓冲区、提交数据更新 */
/**
 * @brief 初始化双缓冲数据管理模块
 *
 * 初始化内部两个缓冲区，将所有数据置为初始状态，并设置读写指针。
 */
void DataManager_Init(void);

/**
 * @brief 获取当前只读数据缓冲区指针
 *
 * 返回当前经过提交、稳定的数据快照，供实时任务读取。
 *
 * @return 指向 GlobalData_t 只读缓冲区的常量指针
 */
const GlobalData_t* DataManager_GetReadBuffer(void);

/**
 * @brief 获取当前写数据缓冲区指针
 *
 * 返回数据采集或处理线程用于写入新数据的缓冲区指针。
 *
 * @return 指向 GlobalData_t 写缓冲区的指针
 */
GlobalData_t* DataManager_GetWriteBuffer(void);

/**
 * @brief 提交写缓冲区数据
 *
 * 通过进入临界区后交换读写缓冲区指针，确保数据更新的原子性，
 * 保证外部读取的数据是一份完整、一致的全局快照。
 */
void DataManager_Commit(void);


/* -----------------------------------------------------------------------------------------------电机电流数据操作接口 */
/**
 * @brief 设置指定电机的电流数据（写缓冲区）
 *
 * ADC数据处理线程在滤波后调用此接口将电流数据写入写缓冲区。
 *
 * @param motorIndex 电机索引（0 ~ MOTOR_COUNT-1）
 * @param currentVal 滤波后的电流值
 */
void DataManager_SetMotorCurrentMea(uint8_t motorIndex, float currentVal);
void DataManager_SetMotorCurrentRef(uint8_t motorIndex, float currentVal);
/**
 * @brief 获取指定电机的电流数据（读缓冲区）
 *
 * 外部任务调用此接口从稳定的只读缓冲区中获取电流数据。
 *
 * @param motorIndex 电机索引（0 ~ MOTOR_COUNT-1）
 * @return 对应电机的电流值；索引无效时返回 0.0f
 */
float DataManager_GetMotorCurrentMea(uint8_t motorIndex);
float DataManager_GetMotorCurrentRef(uint8_t motorIndex);
/**
 * @brief 一次性写入多路（最多 MOTOR_COUNT 路）电机电流数据
 *
 * @param pCurrents 指向待写入的电流数组指针
 * @param count     准备写入的数量，应 <= MOTOR_COUNT
 */
void DataManager_SetAllMotorCurrentsMea(const float *pCurrents, uint8_t count);
void DataManager_SetAllMotorCurrentsRef(const float *pCurrents, uint8_t count);
/**
 * @brief 一次性读取多路（最多 MOTOR_COUNT 路）电机电流数据
 *
 * @param pOutCurrents 用于接收读出数据的数组指针
 * @param count        准备读取的数量，应 <= MOTOR_COUNT
 */
void DataManager_GetAllMotorCurrentsMea(float *pOutCurrents, uint8_t count);
void DataManager_GetAllMotorCurrentsRef(float *pOutCurrents, uint8_t count);
/* -----------------------------------------------------------------------------------------------电机速度数据操作接口 */
/**
 * @brief 设置指定电机的速度数据（写缓冲区）
 *
 * 数据采集线程调用此接口将电机速度数据写入写缓冲区。
 *
 * @param motorIndex 电机索引（0 ~ MOTOR_COUNT-1）
 * @param speed      电机速度值
 */
void DataManager_SetMotorSpeedMea(uint8_t motorIndex, float speed);
void DataManager_SetMotorSpeedRef(uint8_t motorIndex, float speed);
/**
 * @brief 获取指定电机的速度数据（读缓冲区）
 *
 * 外部任务调用此接口从只读缓冲区获取电机速度数据。
 *
 * @param motorIndex 电机索引（0 ~ MOTOR_COUNT-1）
 * @return 电机对应的速度值；索引无效时返回 0.0f
 */
float DataManager_GetMotorSpeedMea(uint8_t motorIndex);
float DataManager_GetMotorSpeedRef(uint8_t motorIndex);
/**
 * @brief 一次性写入多路（最多 MOTOR_COUNT 路）电机速度数据
 *
 * @param pSpeeds 指向待写入的速度数组
 * @param count   写入的数量，最多为 MOTOR_COUNT
 */
void DataManager_SetAllMotorSpeedsMea(const float *pSpeeds, uint8_t count);
void DataManager_SetAllMotorSpeedsRef(const float *pSpeeds, uint8_t count);
/**
 * @brief 一次性读取多路（最多 MOTOR_COUNT 路）电机速度数据
 *
 * @param pOutSpeeds 指向接收速度数据的数组
 * @param count     读取数量，最多为 MOTOR_COUNT
 */
void DataManager_GetAllMotorSpeedsMea(float *pOutSpeeds, uint8_t count);
void DataManager_GetAllMotorSpeedsRef(float *pOutSpeeds, uint8_t count);

/* -----------------------------------------------------------------------------------------------电机位置数据操作接口 */
/**
 * @brief 设置指定电机的位置数据（写缓冲区）
 *
 * 数据采集线程调用此接口将电机位置数据写入写缓冲区。
 *
 * @param motorIndex 电机索引（0 ~ MOTOR_COUNT-1）
 * @param position   电机位置数据
 */
void DataManager_SetMotorPositionMea(uint8_t motorIndex, float position);
void DataManager_SetMotorPositionRef(uint8_t motorIndex, float position);
/**
 * @brief 获取指定电机的位置数据（读缓冲区）
 *
 * 外部任务调用此接口从只读缓冲区获取电机位置数据。
 *
 * @param motorIndex 电机索引（0 ~ MOTOR_COUNT-1）
 * @return 电机对应的位置值；索引无效时返回 0.0f
 */
float DataManager_GetMotorPositionMea(uint8_t motorIndex);
float DataManager_GetMotorPositionRef(uint8_t motorIndex);
/**
 * @brief 一次性写入多路（最多 MOTOR_COUNT 路）电机位置数据
 *
 * @param pPositions 指向待写入的位置数据数组
 * @param count      写入的数量，最多为 MOTOR_COUNT
 */
void DataManager_SetAllMotorPositionsMea(const volatile float *pPositions, uint8_t count);
void DataManager_SetAllMotorPositionsRef(const volatile float *pPositions, uint8_t count);
/**
 * @brief 一次性读取多路（最多 MOTOR_COUNT 路）电机位置数据
 *
 * @param pOutPositions 指向接收位置数据的数组
 * @param count         读取数量，最多为 MOTOR_COUNT
 */
void DataManager_GetAllMotorPositionsMea(float *pOutPositions, uint8_t count);
void DataManager_GetAllMotorPositionsRef(float *pOutPositions, uint8_t count);
/* -----------------------------------------------------------------------------------------------温度数据操作接口示例 */
/**
 * @brief 设置系统温度数据（写缓冲区）
 *
 * 数据处理线程调用此接口将温度数据写入写缓冲区。
 *
 * @param temp 当前温度值
 */
void DataManager_SetTemperatureMea(float temp);

/**
 * @brief 获取系统温度数据（读缓冲区）
 *
 * 外部任务通过此接口获取最新温度数据。
 *
 * @return 当前温度值
 */
float DataManager_GetTemperatureMea(void);

//用户角度数据操作接口
void DataManager_SetAngleCmdAll(const float *pAngles, uint8_t count);
void DataManager_GetAngleCmdAll(float *pOutAngles, uint8_t count);

//用户速度数据操作接口
void DataManager_SetSpeedCmdAll(const float *pSpeeds, uint8_t count);
void DataManager_GetSpeedCmdAll(float *pOutSpeeds, uint8_t count);

//模型速度目标数据操作接口
void DataManager_SetSpeedTarAll(const float *pSpeeds, uint8_t count);
void DataManager_GetSpeedTarAll(float *pOutSpeeds, uint8_t count);

//模型角度目标数据操作接口
void DataManager_SetAngleDuAll(const float *pAngles, uint8_t count);
void DataManager_GetAngleDuAll(float *pOutAngles, uint8_t count);

//模型速度目标数据操作接口
void DataManager_SetAngleSuduAll(const float *pSpeeds, uint8_t count);
void DataManager_GetAngleSuduAll(float *pOutSpeeds, uint8_t count);












#ifdef __cplusplus
}
#endif

#endif // DATA_MANAGER_H
