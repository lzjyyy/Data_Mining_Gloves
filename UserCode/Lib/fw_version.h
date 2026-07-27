#ifndef FW_VERSION_H
#define FW_VERSION_H

#include <stdint.h>

typedef struct
{
    uint8_t major;       // 主版本
    uint8_t minor;       // 次版本
    uint8_t patch;       // 修订
    uint32_t data;      // 发布日期，格式：YYYYMMDD
}__attribute__((packed)) fw_version_t;

extern const fw_version_t g_fw_version;

#endif // FW_VERSION_H
