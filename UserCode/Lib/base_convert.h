#ifndef BASE__CONVERT_H
#define BASE__CONVERT_H
#include <string.h>
#include <stdarg.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif




uint16_t float_to_half(float f);
float half_to_float(uint16_t h); 


/*将num个浮点数转换为2*num个字节存储到out数组中*/ 
void floats_to_bytes(const float in[], uint8_t out[], size_t num); 

/*将2*num个字节转换为num个浮点数存储到out数组中*/ 
void bytes_to_floats(const uint8_t in[], float out[], size_t num); 

/*float型数组拼帧*/
void splice_float_frame(float *out, int numArrays, ...);

/*uint8型数组拼帧*/
void splice_uint8_frame(uint8_t *out, int numArrays, ...);

#ifdef __cplusplus
}
#endif

#endif


