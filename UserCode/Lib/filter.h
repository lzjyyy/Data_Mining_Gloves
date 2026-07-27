#ifndef FILTER_H
#define FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include "CurrSenseTask.h"

/**
 * @brief 平均滑动滤波器上下文结构体（供线程独立使用）
 * 每个线程保持自己的上下文实例直到线程结束
 */
typedef struct {
    float* buffer;         // 循环缓冲区
    uint32_t bufferSize;   // 缓冲区大小（窗口大小）
    uint32_t currentIndex; // 当前索引位置
    float sum;             // 当前窗口内数据和
    uint8_t initialized;   // 初始化标志
} FilterContext_t;


/**
 * @brief 平均滑动滤波函数
 * 处理输入数据并输出滤波结果
 * 
 * @param input 输入数据数组
 * @param output 输出数据数组
 * @param dataSize 数据维度（处理数据的长度）
 * @param windowSize 滑动窗口大小
 * @param context 滤波器上下文指针（每个线程独立）
 * @return int 0表示成功，负值表示错误：-1参数错误，-2内存分配失败
 */
int movingAverageFilter(const float* input, float* output, uint32_t dataSize, uint32_t windowSize, FilterContext_t* context) ;




/***********************************************************************************************************
 * @brief 中值滤波
 *
 * 对传入的 ADC_Frame_t 数组中每个通道的 100 帧数据求中值，
 * 得到每个通道的中值输出。
 *
 * @param[in]  frames     指向 ADC 数据帧数组的指针
 * @param[in]  numFrames  帧数（例如 ADC_FRAME_POOL_SIZE，即100）
 * @param[out] outMedian  输出数组（大小 ADC_CHANNEL_NUM），存放每个通道的中值
 */
void computeMedian(const ADC_Frame_t* frames, size_t numFrames, float outMedian[ADC_CHANNEL_NUM]);




/***********************************************************************************************************
 * @brief 计算多帧数据的均值
 *
 * 对传入的ADC_Frame_t数组中每个通道的数据求和取平均，
 * 得到每通道的初步均值，用作后续滤波的输入。
 *
 * @param[in]  frames     存储多帧ADC数据的数组（大小为numFrames）
 * @param[in]  numFrames  帧数（例如ADC_FRAME_POOL_SIZE，即200）
 * @param[out] outMean    大小为ADC_CHANNEL_NUM，用于存放每通道均值
 */
void computeMeanFromFrames(const ADC_Frame_t* frames, size_t numFrames, float outMean[ADC_CHANNEL_NUM]);




/***********************************************************************************************************
 * @brief 滑动平均滤波
 *
 * 利用固定窗口（50点）的循环缓冲区，对输入数据进行滑动平均，
 * 进一步平滑初步均值。
 *
 * @param[in]  in   输入数组（大小ADC_CHANNEL_NUM），通常来自computeMeanFromFrames的输出
 * @param[out] out  输出数组（大小ADC_CHANNEL_NUM），存放滑动平均结果
 */
void updateMovingAverage(const float in[ADC_CHANNEL_NUM], float out[ADC_CHANNEL_NUM]);




/***********************************************************************************************************
 * @brief FIR低通滤波
 *
 * 使用5阶加权FIR滤波器对输入数据进行低通滤波，
 * 滤波系数为 {0.1, 0.2, 0.4, 0.2, 0.1}，有效衰减高频干扰。
 *
 * @param[in]  in   输入数组（大小ADC_CHANNEL_NUM），通常来自滑动平均输出
 * @param[out] out  输出数组（大小ADC_CHANNEL_NUM），存放FIR滤波结果
 */
void updateFIRFilter(const float in[ADC_CHANNEL_NUM], float out[ADC_CHANNEL_NUM]);




/***********************************************************************************************************
 * @brief 卡尔曼滤波
 *
 * 对输入数据（大小ADC_CHANNEL_NUM）分别应用一维卡尔曼滤波，
 * 得到最终平滑输出。该滤波器基于简单恒定电流模型，
 * 参数（过程噪声和测量噪声）经过工程调优，适合本系统要求。
 *
 * @param[in]  in   输入数组（大小ADC_CHANNEL_NUM），通常来自FIR滤波输出
 * @param[out] out  输出数组（大小ADC_CHANNEL_NUM），存放卡尔曼滤波结果
 */
void KalmanFilter(const float in[ADC_CHANNEL_NUM], float out[ADC_CHANNEL_NUM]);




/***********************************************************************************************************
 * @brief RMS 有效值计算
 *
 * 对传入的一段单通道浮点数据进行平方、平均后开根号，
 * 返回该数据段的RMS值。此函数用于单独测试数字RMS算法效果，
 * 与 ADC_CHANNEL_NUM 无关。
 *
 * @param[in]  sampleData  输入数据数组（单通道）
 * @param[in]  numSamples  数组中数据的个数
 * @return                 计算得到的 RMS 值
 */
float computeRMS(const float sampleData[], size_t numSamples);





/* --------------------------------------------- 中值滤波 -------------------------------------- */
/**
 * 对输入数据进行中值滤波并减少帧数
 * 
 * @param inputData       输入数据数组的指针（原始帧）
 * @param inputSize       输入数据数组的大小
 * @param outputData      输出数据数组的指针（滤波后的帧）
 * @param outputSize      输出中期望的帧数
 * @param windowSize      中值滤波窗口大小（必须为奇数）
 * 
 * @return                成功返回0，否则返回错误代码
 */
int medianFilterAndReduce(float *inputData, size_t inputSize, float *outputData, size_t outputSize,int windowSize);






int notch_filter_and_downsample(const float* input_data, int input_length, 
                               float* output_data, float fs, float notch_freq, 
                               float quality_factor, int downsample_factor);


															 
float compute_cycle_rms(const float *data, int n); 															 
															 
															 

void savitzky_golay_filter(const float *input, float *output, int n);

void fir_filter(const float *input, float *output, int n, const float *coeff, int filter_order);												 
			

void multiStageFilter(const float *in, float *out, int num_channels, int stages, float ALPHA);
															 
#ifdef __cplusplus
}
#endif

#endif /* FILTER_H */


