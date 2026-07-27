#include "bsp_status.h"
#include "bsp_uart.h"
/**
 * @brief 协议/Flash 错误码转字符串
 */
UpgradeContext_t ctx;
 
const char* ProtocolStatusToStr(ProtocolStatus_t status) {
    switch (status) {
        // ===== Frame 错误描述 =====
        case FRAME_OK:               return "Frame OK";
        case FRAME_ERR_HEADER:       return "Frame Header Error";
        case FRAME_ERR_CRC:          return "Frame CRC16 Error";
        case FRAME_ERR_LEN:          return "Frame Length Error";
        case FRAME_ERR_MAX_LEN:      return "Frame Max Length Error";
        case FRAME_ERR_MIN_LEN:      return "Frame Min Length Error";
        case FRAME_ERR_CRC32:        return "Frame CRC32 Error";
        case FRAME_ERR_TAIL:         return "Frame Tail Error";
        case CMD_FILE_SIZE_ERROR:    return "File Size Error";
        case FRAME_ERR_DISCONTINUOUS:return "Frame ID Not Continuous";
        case FRAME_ERR_DUPLICATE:    return "Frame ID Duplicate";
        case FRAME_ERR_OUT_OF_RANGE: return "Frame ID Out of Range";

        // ===== Flash 错误描述 =====
        case FLASH_OK:               return "Flash OK";
        case FLASH_ERR_ADDR_RANGE:   return "Flash Address Out of Range";
        case FLASH_ERR_ADDR_ALIGN:   return "Flash Address Not 32-Byte Aligned";
        case FLASH_ERR_DATA_ALIGN:   return "Flash Data Not 32-Byte Aligned";
        case FLASH_ERR_LEN:          return "Flash Data Length Error";
        case FLASH_ERR_UNLOCK:       return "Flash Unlock Failed";
        case FLASH_ERR_PROGRAM:      return "Flash Program Failed";
        case FLASH_ERR_VOLTAGE:      return "Flash Invalid Voltage Range";
        case FLASH_ERR_CRC:          return "Flash CRC Check Failed";

        // ===== Flash 擦除状态描述 =====
        case FLASH_ERASE_OK:         return "Flash Erase OK";
        case FLASH_ERASE_ERR_UNLOCK: return "Flash Erase Unlock Failed";
        case FLASH_ERASE_ERR_PROGRAM:return "Flash Erase Program Failed";

        default:                     return "Unknown Error";
    }
}

/**
 * @brief  初始化升级上下文结构体
 * 
 * 该函数会在升级流程开始前或中断后调用，用于将 UpgradeContext_t
 * 中的所有字段重置为默认初始值，确保状态机处于干净的起始状态。
 * 
 * @param ctx 指向 UpgradeContext_t 结构体的指针
 */
void UpgradeContext_Init(UpgradeContext_t *ctx)
{
    if (ctx == NULL) return;

    ctx->state         = UPG_IDLE;     // 回到空闲状态
    ctx->file_length   = 0;                      // 文件大小未知
    ctx->received_len  = 0;                      // 尚未接收数据
    ctx->pre_crc       = 0xFFFFFFFF;             // CRC32 初值（硬件累加用）
    ctx->file_crc      = 0xFFFFFFFF;             // 最终计算 CRC 清零,默认为初始值
    ctx->last_frame_id = 0x00;                 // 无效帧号（防止重复包判定）
		ctx->max_frame_id  = 0;                      //最大帧号，超出文件范围
    ctx->flash_status  = FRAME_OK;        // Flash 状态正常
    ctx->frame_status  = FLASH_OK;        // 帧解析状态正常
		ctx->error_times=0;                   //升级过程中的错误次数
}
// 升级状态转字符串
const char* UpgradeStateToStr(UpgradeState_t state) {
    switch (state) {
        case UPG_IDLE:           return "Idle";
        case UPG_WAIT_FILE_INFO: return "Wait File Info";
        case UPG_ERASE_FLASH:    return "Erase Flash";
        case UPG_WRITE_DATA:     return "Write Data";
        case UPG_VERIFY_CRC:     return "Verify CRC";
        case UPG_DONE:           return "Done";
        case UPG_ERROR:          return "Error";
        default:                 return "Unknown State";
    }
}

void PrintUpgradeContext(const UpgradeContext_t* ctx) {
    if (!ctx) {
        u1_printf("[UpgradeContext] NULL pointer!\n");
        return;
    }

    u1_printf("========== Upgrade Context ==========\n");
    u1_printf("State          : %s\n", UpgradeStateToStr(ctx->state));
    u1_printf("File Length    : %u bytes\n", ctx->file_length);
    u1_printf("Received Length: %u bytes\n", ctx->received_len);
    u1_printf("Prev CRC32     : 0x%08X\n", ctx->pre_crc);
    u1_printf("Final File CRC : 0x%08X\n", ctx->file_crc);
    u1_printf("Last Frame ID  : %u\n", ctx->last_frame_id);
    u1_printf("Max Frame ID   : %u\n", ctx->max_frame_id);
    u1_printf("Flash Status   : %s (0x%02X)\n", 
              ProtocolStatusToStr(ctx->flash_status), ctx->flash_status);
    u1_printf("Frame Status   : %s (0x%02X)\n", 
              ProtocolStatusToStr(ctx->frame_status), ctx->frame_status);
    u1_printf("Error Times    : %u\n", ctx->error_times);
    u1_printf("======================================\n");
}

