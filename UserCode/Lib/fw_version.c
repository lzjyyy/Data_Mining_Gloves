#include "fw_version.h"

// 这里先手动写，后面可以用脚本自动生成
#define FW_VER_MAJOR   2
#define FW_VER_MINOR   0
#define FW_VER_PATCH   5
#define FW_VER_DATE    20260609


const fw_version_t g_fw_version =
{
    .major      = FW_VER_MAJOR,
    .minor      = FW_VER_MINOR,
    .patch      = FW_VER_PATCH,
    .data       = FW_VER_DATE,
};
