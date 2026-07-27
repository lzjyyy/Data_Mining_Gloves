#include "angle_Mapping.h"


static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}


//用户角度到模型角度映射
void mapInputAngleToModelAngle(const float* inputAngles, float* modelAngles, int count)
{
    //先将输入角度限幅
    


    //区分为大拇指两个电机和其他四个电机，都是线性映射关系
    for (int i = 0; i < count; i++) 
    {
        if (i == 0) // 大拇指①号电机(弯曲)
        {
            modelAngles[i] = inputAngles[i] * (THUMB_BEND_MODEL_ANGLE_BEND - THUMB_BEND_MODEL_ANGLE_STRAIGHT) / 
                              THUMB_BEND_ANGLE_BEND + THUMB_BEND_MODEL_ANGLE_STRAIGHT;
        } 
        else if (i == 1) // 大拇指②号电机(摆动)
        {
            modelAngles[i] = inputAngles[i] * (THUMB_SWING_MODEL_ANGLE_BEND - THUMB_SWING_MODEL_ANGLE_STRAIGHT) / 
                              THUMB_SWING_ANGLE_BEND + THUMB_SWING_MODEL_ANGLE_STRAIGHT;
        } 
        else // 其他四个电机
        {
            modelAngles[i] = inputAngles[i] * (FINGER_BEND_MODEL_ANGLE_BEND - FINGER_BEND_MODEL_ANGLE_STRAIGHT) / 
                              FINGER_BEND_ANGLE_BEND + FINGER_BEND_MODEL_ANGLE_STRAIGHT;
        }
    }
}

//模型角度到用户角度映射
void mapModelAngleToInputAngle(const float* modelAngles, float* inputAngles, int count)
{
    //区分为大拇指两个电机和其他四个电机，都是线性映射关系
    for (int i = 0; i < count; i++) 
    {
        if (i == 0) // 大拇指①号电机(弯曲)
        {
            inputAngles[i] = (modelAngles[i] - THUMB_BEND_MODEL_ANGLE_STRAIGHT) * 
                              THUMB_BEND_ANGLE_BEND / (THUMB_BEND_MODEL_ANGLE_BEND - THUMB_BEND_MODEL_ANGLE_STRAIGHT);
        } 
        else if (i == 1) // 大拇指②号电机(摆动)
        {
            inputAngles[i] = (modelAngles[i] - THUMB_SWING_MODEL_ANGLE_STRAIGHT) * 
                              THUMB_SWING_ANGLE_BEND / (THUMB_SWING_MODEL_ANGLE_BEND - THUMB_SWING_MODEL_ANGLE_STRAIGHT);
        } 
        else // 其他四个电机
        {
            inputAngles[i] = (modelAngles[i] - FINGER_BEND_MODEL_ANGLE_STRAIGHT) * 
                              FINGER_BEND_ANGLE_BEND / (FINGER_BEND_MODEL_ANGLE_BEND - FINGER_BEND_MODEL_ANGLE_STRAIGHT);
        }
    }
}
