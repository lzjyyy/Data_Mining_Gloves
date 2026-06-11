#ifndef __MODBUS_MASTER_H__
#define __MODBUS_MASTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>

HAL_StatusTypeDef ModbusMaster_WriteU64(uint8_t slave_addr, uint16_t reg_addr, uint64_t value, uint32_t timeout_ms);
HAL_StatusTypeDef ModbusMaster_ReadU64(uint8_t slave_addr, uint16_t reg_addr, uint64_t *value, uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_MASTER_H__ */
