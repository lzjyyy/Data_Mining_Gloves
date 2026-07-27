
#ifndef ESO_H_
#define ESO_H_
#ifdef __cplusplus
extern "C" {
#endif

typedef struct{
    float z1, z2, error;
    float T, beta1, beta2, b0; // 每个实例自带参数
} ESOState;

void ESO_Init(ESOState* s);                                 // 默认参数
void ESO_InitWithParams(ESOState* s, float Ts, float w, float b0);
void ESO_Step(ESOState* s, float u_volt, float y_meas);

// 通用批量初始化（
void ESO_ALLInit(ESOState* arr, int count, const float* Ts, const float* w, const float* b0);

extern ESOState eso[];

#ifdef __cplusplus
}
#endif
#endif
