#include "base_convert.h"


/**
 * @file data_conversion.c
 * @brief 提供 32 位 float 与 16 位 IEEE 754 half-precision 数之间的相互转换函数。
 *
 * 函数：
 * - uint16_t float_to_half(float f);
 * - float half_to_float(uint16_t h);
 */

#include <stdint.h>
#include <string.h>

/*---------------------------------------------------------------------------------------------------------------float_to_half*/
/**
 * @brief 将 32 位浮点数转换为 16 位 IEEE 754 half-precision 表示。
 *
 * 该函数根据 IEEE 754 标准，将 float 的符号、指数和尾数分别提取，
 * 并转换到 half 格式中。对于超出 half 表示范围的数，会以无穷大表示，
 * 对于极小数则转换为次正规数或零。
 *
 * @param f 要转换的 32 位浮点数
 * @return uint16_t 转换后的 16 位二进制表示
 */
uint16_t float_to_half(float f) 
{
    uint32_t x;
    memcpy(&x, &f, sizeof(x));
    uint16_t sign = (x >> 16) & 0x8000;		// 提取符号（位31）并移至 half 的符号位（位15）
    uint32_t exponent = (x >> 23) & 0xFF; // 提取 float 的指数和尾数
    uint32_t mantissa = x & 0x7FFFFF;
    if (exponent == 0xFF)									// 处理 NaN 或无穷大情况 
		{
        if (mantissa != 0) 
				{
          return sign | 0x7E00; 					// 保留部分尾数信息，返回 NaN（这里返回规范 NaN）
        } 
				else 
				{
          return sign | 0x7C00; 					// 无穷大
         }
    }
    int32_t new_exp = (int32_t)exponent - 127 + 15; // 调整指数：float 的偏移是 127，half 的偏移是 15
    if (new_exp >= 0x1F) 
		{
        return sign | 0x7C00;							// 指数溢出，返回无穷大
    } 
		else if (new_exp <= 0) 
		{
        if (new_exp < -10)								// 指数不足，可能为次正规数或零
				{
            return sign;									// 数值太小，直接返回零
        }
																					// 处理次正规数：恢复隐含的1后右移以对齐尾数
        mantissa |= 0x800000;  // 补充隐含1
        uint32_t sub = mantissa >> (1 - new_exp + 13);
        return sign | (uint16_t)sub;
     } 
		 else 
		 {
        uint16_t half = sign | ((uint16_t)new_exp << 10) | ((uint16_t)(mantissa >> 13));// 处理正规数：截取尾数的高10位
        return half;
     }
}
/*------------------------------------------------------------------------------------------------------------------half_to_float*/
/**
 * @brief 将 16 位 IEEE 754 half-precision 数转换为 32 位浮点数。
 *
 * 该函数将 half 数中的符号、指数和尾数分别提取，
 * 并扩展到 32 位 float 的格式中，对于次正规数会进行归一化处理。
 *
 * @param h 16 位 half-precision 二进制数
 * @return float 转换得到的 32 位浮点数
 */
float half_to_float(uint16_t h) 
{ 
    uint32_t sign = (uint32_t)(h & 0x8000) << 16;// 提取 half 的各部分：符号、指数和尾数
    uint32_t exponent = (h >> 10) & 0x1F;
    uint32_t mantissa = h & 0x3FF;
    uint32_t f_bits;
    if (exponent == 0) 
		{
        if (mantissa == 0) 
				{
            f_bits = sign; 											// 零
        } 
				else 
				{
																								// 次正规数：归一化 
            while ((mantissa & 0x400) == 0) 		// 将次正规数转换为正规数（归一化时计算真实指数）
						{
                mantissa <<= 1;
                exponent--;
            }
            mantissa &= 0x3FF; 									// 去掉隐含的1          
            uint32_t new_exp = (uint32_t)(1 + (127 - 15));  // 次正规数对应的实际指数为：1 - 15（half偏移）后转换为 float（偏移127）
            f_bits = sign | (new_exp << 23) | (mantissa << 13);
        }
    } 
		else if (exponent == 0x1F) 
		{       
        f_bits = sign | 0x7F800000 | (mantissa << 13); 			// 无穷大或 NaN
    } 
		else 
		{       
        uint32_t new_exp = exponent + (127 - 15);						// 正规数：调整指数偏移（half偏移15 -> float偏移127）
        f_bits = sign | (new_exp << 23) | (mantissa << 13);
    }
    float result;
    memcpy(&result, &f_bits, sizeof(result));
    return result;
}


/*---------------------------------------------------------------------------------------------------------------floats_to_halfs*/
/*将num个浮点数转换为2*num个字节存储到out数组中*/ 
void floats_to_bytes(const float in[], uint8_t out[], size_t num) 
{
    for (size_t i = 0; i < num; i++) 
		{   
        uint16_t half_value = float_to_half(in[i]);			// 将浮点数转换为16位的半精度浮点表示
																												// 分拆为高8位和低8位，依次存入字节数组中
        out[2 * i]     = (uint8_t)(half_value >> 8);    // 高8位
        out[2 * i + 1] = (uint8_t)(half_value & 0xFF);  // 低8位
    }
}


/*------------------------------------------------------------------------------------------------------------------halfs_to_floats*/
/*将2*num个字节转换为num个浮点数存储到out数组中*/ 
void bytes_to_floats(const uint8_t in[], float out[], size_t num) 
{
    for (size_t i = 0; i < num; i++) 
		{
																												// 合并连续两个字节为16位半精度表示（假设高位在前）
        uint16_t half_value = ((uint16_t)in[2 * i] << 8) | in[2 * i + 1];
																												// 将半精度转换为单精度并存入输出数组
        out[i] = half_to_float(half_value);
    }
}



/*------------------------------------------------------------------------------------------------------------------float型数组拼帧函数*/

/*
 * splice_float_frame 函数说明：
 *
 * 功能：
 *   将多个 float 数组按传入顺序依次拼接到预先分配好内存的输出数组中。
 *
 * 参数：
 *   out       - 输出缓冲区指针，应预先分配好足够空间以容纳所有数据（总元素个数 = 各个数组元素数之和）。
 *   numArrays - 待拼接的 float 数组个数。
 *   ...       - 可变参数部分，对于每个待拼接数组，需要依次传入：
 *                   1) float 数组指针 (float*)
 *                   2) 该数组的元素个数 (size_t)
 *
 * 使用示例：
 *
 *   // 假设有3个 float 型数组
 *   float a[2] = {1.1f, 2.2f};
 *   float b[3] = {3.3f, 4.4f, 5.5f};
 *   float c[1] = {6.6f};
 *
 *   // 输出数组大小需为2+3+1=6个 float 元素
 *   float out[6];
 *
 *   // 调用函数按 a, b, c 顺序拼接，最终 out 数组中依次存放 a, b, c 中的所有数据
 *   splice_float_frame(out, 3, a, (size_t)2, b, (size_t)3, c, (size_t)1);
 */
void splice_float_frame(float *out, int numArrays, ...)
{
    va_list args;
    va_start(args, numArrays);

    size_t offset = 0;   // 以字节为单位的偏移量
    for (int i = 0; i < numArrays; i++) 
		{
        // 依次获取每个 float 数组的指针和元素个数
        float *array_ptr = va_arg(args, float *);
        size_t count     = va_arg(args, size_t);

        // 使用 memcpy 将 count 个 float 数据复制到输出缓冲区中
        memcpy((char *)out + offset, array_ptr, count * sizeof(float));
        offset += count * sizeof(float);
    }
    va_end(args);
}




/*------------------------------------------------------------------------------------------------------------------uint8_t型数组拼帧函数*/
/*
 * splice_uint8_frame 函数说明：
 *
 * 功能：
 *   将多个 uint8_t 数组按传入顺序依次拼接到预先分配好内存的输出数组中。
 *
 * 参数：
 *   out       - 输出缓冲区指针，类型为 uint8_t*，调用前需要分配好足够的空间，空间大小为所有数组字节数之和。
 *   numArrays - 待拼接的数组个数。
 *   ...       - 可变参数部分，对于每个待拼接的数组，需要依次传入：
 *                 1) 数组指针 (uint8_t*)
 *                 2) 数组的长度（单位：字节，size_t 类型）
 *
 * 使用示例：
 *
 *   假设有以下三个 uint8_t 数组：
 *       uint8_t a[2] = {0x01, 0x02};
 *       uint8_t b[4] = {0x03, 0x04, 0x05, 0x06};
 *       uint8_t c[1] = {0x07};
 *
 *   需要将它们依次拼接成一个大数组 out，其总长度为 2+4+1 = 7 字节。
 *
 *   调用方式如下：
 *
 *       uint8_t out[7];
 *       splice_uint8_frame(out, 3,
 *                            a, (size_t)2,
 *                            b, (size_t)4,
 *                            c, (size_t)1);
 */
void splice_uint8_frame(uint8_t *out, int numArrays, ...)
{
    va_list args;
    va_start(args, numArrays);

    size_t offset = 0;
    for (int i = 0; i < numArrays; i++) {
        // 获取每个数组的指针和对应长度
        uint8_t *array_ptr = va_arg(args, uint8_t *);
        size_t count = va_arg(args, size_t);

        // 将数组内容拷贝到输出数组中，并更新偏移量
        memcpy(out + offset, array_ptr, count);
        offset += count;
    }
    va_end(args);
}









