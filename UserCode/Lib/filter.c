#include "filter.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>


/*滑动平均滤波*/
#define MA_WINDOW_SIZE 60
static float maBuffer[ADC_CHANNEL_NUM][MA_WINDOW_SIZE] = {0};
static size_t maIndex = 0;
/*FIR低通滤波*/
#define FIR_TAPS 5
static const float firCoeffs[FIR_TAPS] = {0.1f, 0.2f, 0.4f, 0.2f, 0.1f};/*固定加权FIR系数*/
static float firBuffer[ADC_CHANNEL_NUM][FIR_TAPS] = {0};
static size_t firIndex = 0;


/*卡尔曼滤波*/
static float kalman_prev[ADC_CHANNEL_NUM] = {8000.f,8000.f,8000.f,8000.f,8000.f,8000.f};/*初始估计值（根据测量预期设置）*/ 
static float kalman_p[ADC_CHANNEL_NUM] = {300000.0f,300000.0f,300000.0f,300000.0f,300000.0f,300000.0f};   /*初始估计误差协方差*/ 
static const float kalman_q = 5000.0f; /*过程噪声协方差*/ 
static const float kalman_r = 31700.0f;/*测量噪声协方差*/ 


/* ----------------- 多帧均值 ----------------- */
void computeMeanFromFrames(const ADC_Frame_t* frames, size_t numFrames, float outMean[ADC_CHANNEL_NUM])
{
    for (size_t ch = 0; ch < ADC_CHANNEL_NUM; ch++) 
		{
        uint32_t sum = 0;
        for (size_t i = 0; i < numFrames; i++) 
				{
            sum += frames[i].data[ch];
        }
        outMean[ch] = (float)sum / numFrames;
    }
}

/* ----------------- 滑动平均滤波 ----------------- */
void updateMovingAverage(const float in[ADC_CHANNEL_NUM], float out[ADC_CHANNEL_NUM])
{
    for (size_t ch = 0; ch < ADC_CHANNEL_NUM; ch++) 
		{
        maBuffer[ch][maIndex] = in[ch];
    }
    maIndex = (maIndex + 1) % MA_WINDOW_SIZE;
    
    for (size_t ch = 0; ch < ADC_CHANNEL_NUM; ch++) 
		{
        float sum = 0.0f;
        for (size_t i = 0; i < MA_WINDOW_SIZE; i++) 
				{
            sum += maBuffer[ch][i];
        }
        out[ch] = sum / MA_WINDOW_SIZE;
    }
}

/* ----------------- FIR低通滤波 ----------------- */
void updateFIRFilter(const float in[ADC_CHANNEL_NUM], float out[ADC_CHANNEL_NUM])
{
    for (size_t ch = 0; ch < ADC_CHANNEL_NUM; ch++) 
		{
        firBuffer[ch][firIndex] = in[ch];
    }
    firIndex = (firIndex + 1) % FIR_TAPS;
    
    /* 计算卷积和：按固定系数加权 */
    for (size_t ch = 0; ch < ADC_CHANNEL_NUM; ch++) 
		{
        float sum = 0.0f;
        for (size_t i = 0; i < FIR_TAPS; i++) 
				{
            /* 这里采用简单循环访问，假设firCoeffs[0]对应最旧数据 */
            size_t idx = (firIndex + i) % FIR_TAPS;
            sum += firCoeffs[i] * firBuffer[ch][idx];
        }
        out[ch] = sum;
    }
}

/* ----------------- 卡尔曼滤波 ----------------- */
void KalmanFilter(const float in[ADC_CHANNEL_NUM], float out[ADC_CHANNEL_NUM])
{
    for (size_t ch = 0; ch < ADC_CHANNEL_NUM; ch++) 
		{
        kalman_p[ch] += kalman_q;
        float kGain = kalman_p[ch] / (kalman_p[ch] + kalman_r);
        float kalman_est = kalman_prev[ch] + kGain * (in[ch] - kalman_prev[ch]);
        kalman_p[ch] = (1 - kGain) * kalman_p[ch];
        kalman_prev[ch] = kalman_est;
        out[ch] = kalman_est;
    }
}


/* ----------------- RMS有效值计算 ----------------- */
/**
 * @brief 计算单通道浮点数据的 RMS 值
 *
 * 对输入数组的每个样本进行平方、平均，再开根号，
 * 返回计算得到的RMS值。
 *
 * @param[in] sampleData  输入数据数组（单通道）
 * @param[in] numSamples  数组中数据的个数
 * @return                计算得到的 RMS 值
 */
float computeRMS(const float sampleData[], size_t numSamples)
{
    double sumSq = 0.0;
    for (size_t i = 0; i < numSamples; i++) 
		{
        sumSq += (double)sampleData[i] * (double)sampleData[i];
    }
    return (float)sqrt(sumSq / numSamples);
}






/* --------------------------------------------- 滑动平均滤波 -------------------------------------- */

int movingAverageFilter(const float* input, float* output, uint32_t dataSize, uint32_t windowSize, FilterContext_t* context) 
{
    /*①参数检查*/
    if (!input || !output || !context || windowSize == 0 || dataSize == 0) 
		{
        return -1;
    }
    /*②首次使用或窗口大小变化时初始化/重置上下文*/ 
    if (!context->initialized || (context->bufferSize != windowSize && windowSize > 0)) {
        if (context->buffer)// 释放之前可能存在的缓冲区 
				{
            free(context->buffer);
        }
        context->buffer = (float*)calloc(windowSize, sizeof(float));// 分配新的缓冲区并初始化为0
        if (!context->buffer) 
				{
            return -2;// 内存分配失败
        }
        context->bufferSize = windowSize;
        context->currentIndex = 0;
        context->sum = 0.0f;
        context->initialized = 1;
    }
    /*③处理每个输入数据点*/ 
    for (uint32_t i = 0; i < dataSize; i++) 
		{
        context->sum -= context->buffer[context->currentIndex];// 减去将被替换的旧值
        context->buffer[context->currentIndex] = input[i]; // 添加新值
        context->sum += input[i]; 
        context->currentIndex = (context->currentIndex + 1) % windowSize;// 更新索引，实现循环缓冲
        output[i] = context->sum / windowSize;// 计算平均值
    }
    return 0;
}


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
int medianFilterAndReduce(float *inputData, size_t inputSize, float *outputData, size_t outputSize,int windowSize)
{
    //参数验证
    if (inputData == NULL || outputData == NULL) 
		{
        return -1;  // 无效指针
    }
    if (outputSize > inputSize || outputSize == 0) 
		{
        return -2;  // 无效的输出大小
    }
    if (windowSize <= 0 || windowSize % 2 == 0) 
		{
        return -3;  // 窗口大小必须是正奇数
    }
    // 计算用于帧减少的步长
    float stepSize = (float)inputSize / outputSize;
    int halfWindow = windowSize / 2;
    // 为中值计算创建临时数组
    float *windowValues = (float *)malloc(windowSize * sizeof(float));
    if (windowValues == NULL) 
		{
        return -4;  // 内存分配失败
    }
    // 处理每一个输出帧
    for (size_t outIdx = 0; outIdx < outputSize; outIdx++) 
		{
        // 计算输入数组中的中心位置
        int centerPos = (int)(outIdx * stepSize);
        // 用中心位置周围的值填充窗口数组
        int windowCount = 0;
        for (int i = -halfWindow; i <= halfWindow; i++) 
				{
            int pos = centerPos + i;
            // 处理边界条件
            if (pos >= 0 && pos < (int)inputSize) 
						{
                windowValues[windowCount++] = inputData[pos];
            }
        }
        // 对窗口值进行排序 - 小窗口使用简单的插入排序
        for (int i = 1; i < windowCount; i++) 
				{
            float key = windowValues[i];
            int j = i - 1;
            while (j >= 0 && windowValues[j] > key) 
						{
                windowValues[j + 1] = windowValues[j];
                j--;
            }
            windowValues[j + 1] = key;
        }
        // 获取中值
        if (windowCount % 2 == 0 && windowCount > 0) 
				{
            // 偶数个元素，取中间两个的平均值
            outputData[outIdx] = (windowValues[windowCount/2 - 1] + windowValues[windowCount/2]) / 2.0f;
        } 
				else if (windowCount > 0) 
				{
            // 奇数个元素，取中间一个
            outputData[outIdx] = windowValues[windowCount/2];
        } 
				else 
				{
            // 窗口中没有有效数据点
            outputData[outIdx] = 0.0f;
        }
    }
    free(windowValues);
    return 0;  // 成功
}









/**
 * @brief  多阶级联一阶低通滤波器
 *
 * 说明：
 *  - 每次输入一个包含 num_channels 个数据的向量（比如一个周期得到的卡尔曼滤波后 1×N 数据）。
 *  - 级联 stages 个一阶低通滤波器，公式：new = ALPHA * current + (1 - ALPHA) * prev
 *  - 每一级的输出作为下一一级的输入。
 *  - 滤波器状态保存在一个静态二维数组中，保证连续调用时状态得以保存。
 *  - 时间复杂度为 O(stages * num_channels)。
 *
 * @param in           当前周期输入数据，数组长度为 num_channels
 * @param out          平滑后的输出数据，数组长度为 num_channels
 * @param num_channels 数据的通道数
 * @param stages       级联滤波器的级数（比如 3 阶、4 阶等）
 * @param ALPHA        平滑系数（在0到1之间，值越小平滑效果越明显，但响应越慢）
 */
void multiStageFilter(const float *in, float *out, int num_channels, int stages, float ALPHA)
{
    // 这里不检查 num_channels 和 stages 是否超出 MAX_CHANNELS / MAX_STAGES，
    // 假设调用时确保 num_channels <= MAX_CHANNELS 且 stages <= MAX_STAGES。
    // 静态状态数组：state[stage][channel]
    static double state[18][10] = {0};  //滤波器最大通道数18，最大级数10，初始化为0
    for (int ch = 0; ch < num_channels; ch++) 
		{
        float value = in[ch];
        // 级联每一阶滤波器
        for (int stage = 0; stage < stages; stage++) 
				{
            state[stage][ch] = ALPHA * value + (1 - ALPHA) * state[stage][ch];
            value = state[stage][ch];  // 将当前级输出作为下一级的输入
        }
        out[ch] = value;               // 最后一级的输出为最终平滑值
    }
}








