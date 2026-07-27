#ifndef __ANGLE_MAPPING_H__
#define __ANGLE_MAPPING_H__

//定义输入角度范围
//大拇指①号电机(弯曲)
//伸直和弯曲
#define THUMB_BAND_ANGLE_STRAIGHT   0.0f 
#define THUMB_BEND_ANGLE_BEND       43.5f
//大拇指②号电机(摆动)
#define THUMB_SWING_ANGLE_STRAIGHT 0.0f 
#define THUMB_SWING_ANGLE_BEND 86.0f
//其他四指弯曲
#define FINGER_BEND_ANGLE_STRAIGHT 0.0f 
#define FINGER_BEND_ANGLE_BEND 80.0f


//定义模型角度范围
//大拇指①号电机(弯曲)
#define THUMB_BEND_MODEL_ANGLE_STRAIGHT  138.33		//138.22f
#define THUMB_BEND_MODEL_ANGLE_BEND 105//83.21				//92.23f
//大拇指②号电机(摆动)
#define THUMB_SWING_MODEL_ANGLE_STRAIGHT 6.47			//7.27f
#define THUMB_SWING_MODEL_ANGLE_BEND 95.34				//96.1f
//其他四个电机
#define FINGER_BEND_MODEL_ANGLE_STRAIGHT 172.90		//173.1f    
#define FINGER_BEND_MODEL_ANGLE_BEND 92.50				//84.4f



// 用户输入角度映射到模型角度
void mapInputAngleToModelAngle(const float* inputAngles, float* modelAngles, int count);

// 模型角度映射到用户输入角度
void mapModelAngleToInputAngle(const float* modelAngles, float* inputAngles, int count);

#endif // __ANGLE_MAPPING_H__
