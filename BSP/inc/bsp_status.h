#ifndef __BSP_STATUS__H
#define __BSP_STATUS__H
#include "main.h"

extern  CRC_HandleTypeDef hcrc;
//define frame satus 
#define APP_ADDRESS 0x08020000U  // App 的起始地址
#define FRAME_HEADER   0xAA
#define FRAME_TAIL   	 0x55
#define MAX_FRAME_SIZE 140  // 1+1+1+1+134(2+128+4)+2
#define FRAME_MAX_DATA  134
#define FRAME_MIN_DATA    2
#define FLASH_START_ADDR 0x8020000
#define FLASH_WRITE_SIZE 32
#define FRAME_PAYLOAD_SIZE 128
//command data definition
#define CMD_STOP_UPGRADE           0xF0
#define CMD_UPGRADE_REQUEST        0xF1
#define CMD_APP_READY          		 0xF2
#define CMD_BOOT_READY             0xF3
#define CMD_FILE_INFO              0xF4
#define CMD_DATA_FRAME             0xF5
#define CMD_CRC_VERIFY             0xF6
#define CMD_CRC_PASS               0xF7
#define CMD_CRC_FAIL             	 0xF8
#define CMD_FRAME_ERROR            0xF9
#define CMD_FLASH_ERROR            0xFA
#define CMD_STATUS_ERROR           0xFB
#define CMD_FRAMEID_ERROR          0xFC
#define CMD_DEBUG_SHOW             0xFD




#define BOOT_READY_VALUE        0xCD
#define APP_READY_VALUE         0xEE
#define APP_NOT_READY_VALUE     0xBB

//define frame satus 
// STM32H723用户Flash范围：0x08020000 ~ 0x080FFFFF（896KB）
#define FLASH_USER_START_ADDR 0x08020000       
#define FLASH_USER_END_ADDR   0x080FFFFF

//CRC校验相关的
#define CRC32_POLYNOMIAL 0x04C11DB7

typedef enum {
    // ===== Frame 错误 (0x00 段) =====
    FRAME_OK = 0x00,          // 解析成功
    FRAME_ERR_HEADER,         // 0x01: 帧头错误
    FRAME_ERR_CRC,            // 0x02: CRC16 错误
    FRAME_ERR_LEN,            // 0x03: 长度错误
    FRAME_ERR_MAX_LEN,        // 0x04: 超过最大长度
    FRAME_ERR_MIN_LEN,        // 0x05: 小于最小长度
    FRAME_ERR_CRC32,          // 0x06: CRC32 错误
    FRAME_ERR_TAIL,           // 0x07: 帧尾错误
		CMD_FILE_SIZE_ERROR,      // 0x08: 文件的长度错误
		FRAME_ERR_DISCONTINUOUS,	// 0x09: 文件的内容帧编号不连续
		FRAME_ERR_DUPLICATE,      // 0x0A: 文件的内容帧编号重复
		FRAME_ERR_OUT_OF_RANGE,   // 0x0B: 文件的内容帧编号超出范围
    // ===== Flash 错误 (0x10 段) =====
    FLASH_OK = 0x10,          // 成功
    FLASH_ERR_ADDR_RANGE,     // 0x11: 地址超出范围
    FLASH_ERR_ADDR_ALIGN,     // 0x12: 地址未32字节对齐
    FLASH_ERR_DATA_ALIGN,     // 0x13: 数据未32字节对齐
    FLASH_ERR_LEN,            // 0x14: 数据长度错误
    FLASH_ERR_UNLOCK,         // 0x15: 解锁失败
    FLASH_ERR_PROGRAM,        // 0x16: 编程失败
    FLASH_ERR_VOLTAGE,        // 0x17: 电压范围无效
    FLASH_ERR_CRC,            // 0x18: CRC 校验失败
		
		// ===== Flash 错误 (0x10 段) =====
		FLASH_ERASE_OK,           // 0x19: flash擦除成功
		FLASH_ERASE_ERROR,        // 0x1A: 擦除失败
		FLASH_ERASE_ERR_UNLOCK,   // 0x1B: flash解锁失败
		FLASH_ERASE_ERR_PROGRAM,  // 0x1C: falsh编程失败
		
		boot_test=0xFE            //测试字段
} ProtocolStatus_t;

typedef enum {
    UPG_IDLE = 0,             // 空闲
    UPG_WAIT_FILE_INFO,       // 等待文件信息
    UPG_ERASE_FLASH,          // 擦除目标区域
    UPG_WRITE_DATA,           // 接收并写入数据
    UPG_VERIFY_CRC,           // 校验 CRC
    UPG_DONE,                 // 升级完成
    UPG_ERROR                 // 错误状态
} UpgradeState_t;

typedef struct {
    UpgradeState_t   state;          // 当前升级状态
    uint32_t         file_length;    // 升级文件总大小（字节）
    uint32_t         received_len;   // 已接收数据长度（字节）
    uint32_t         pre_crc;        // 上一次参与计算的 CRC32（硬件CRC累加用）
    uint32_t         file_crc;       // 最终计算得到的 CRC32
    uint16_t         last_frame_id;  // 最后处理的帧号（防止重复包）
		uint16_t         max_frame_id;   // 最大的帧号（防止写入长度超过）
    ProtocolStatus_t flash_status;   // Flash 操作状态（统一错误状态枚举）
    ProtocolStatus_t frame_status;   // Frame 解析状态（统一错误状态枚举）
		uint8_t          error_times;    // 错误累计计数（帧错误+1，Flash错误+6）
} UpgradeContext_t;


extern  UpgradeContext_t ctx;

const char* ProtocolStatusToStr(ProtocolStatus_t status);
void UpgradeContext_Init(UpgradeContext_t *ctx);
void PrintUpgradeContext(const UpgradeContext_t* ctx);
const char* UpgradeStateToStr(UpgradeState_t state);
#endif
