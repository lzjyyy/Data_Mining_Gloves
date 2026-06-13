#ifndef __MODBUS_MASTER_H__
#define __MODBUS_MASTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

typedef struct
{
  uint32_t op;
  uint32_t status;
  uint32_t rx_len;
  uint32_t rx_addr;
  uint32_t rx_func;
  uint32_t reg_addr;
} ModbusMaster_DebugTypeDef;

HAL_StatusTypeDef ModbusMaster_WriteU64(uint8_t slave_addr, uint16_t reg_addr, uint64_t value, uint32_t timeout_ms);
HAL_StatusTypeDef ModbusMaster_ReadU64(uint8_t slave_addr, uint16_t reg_addr, uint64_t *value, uint32_t timeout_ms);
void ModbusMaster_GetDebug(ModbusMaster_DebugTypeDef *debug);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_MASTER_H__ */
