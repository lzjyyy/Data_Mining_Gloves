#ifndef RS485_HANDLERS_H
#define RS485_HANDLERS_H

#include "rs485_protocol.h"
#include <stdint.h>
#include <stdbool.h>
#include "Kin_Slover.h"

#ifdef __cplusplus
extern "C" {
#endif

void rs485_handlers_init(rs485_proto_ctx_t *ctx);


extern void DataManager_SetAllMotorPositionsRef(const volatile float *posRef, uint8_t n);
extern void DataManager_Commit(void);

/* 寄存器映射 */
#define REG_SYS_SLAVE_ADDR   0x0000  /* 低8位为新地址（1..247） */

/* ===== 波特率编码表（两字节编码，便于 Modbus 寄存器传输） =====
 *  0x0001 -> 9600
 *  0x0002 -> 19200
 *  0x0003 -> 38400
 *  0x0004 -> 57600
 *  0x0005 -> 115200
 *  0x0006 -> 230400
 *  0x0007 -> 460800
 *  0x0008 -> 921600
 */
#define REG_SYS_BAUD_CODE    0x0001  /* 波特率编码      */
 
#define REG_POS_BASE   0x0002  /* 0x0002..0x0007 : 6 regs -> 6 angles (half) */
#define REG_SPD_BASE   0x0008  /* 0x0008..0x000D : 6 regs -> 6 angular velocities (half) */
#define REG_FB_POS_BASE   0x000E
#define REG_FB_SPD_BASE   0x0014
#define REG_FB_CUR_BASE   0x001A
#define RDG_OTA_BASE      0x0100    //OTA标志寄存器起始地址
#define REG_RESET_BASE    0x0A00    //重启命令寄存器地址
#define REG_FW_VER_BASE   0x0B00  /* 固件版本寄存器起始地址 */
#define REG_POS_COUNT  6
#define REG_SPD_COUNT  6
#define REG_FB_COUNT      6

struct rs485_cfg
{
    uint8_t slave_addr;
    uint32_t baudrate;
};

extern struct rs485_cfg g_rs485_cfg;

void SetUart3Baudrate(uint32_t baudrate); 
uint32_t GetUart3Baudrate(void);

#ifdef __cplusplus
}
#endif
#endif
