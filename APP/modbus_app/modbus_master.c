#include "modbus_master.h"

#include "RS485_uasrt.h"
#include "modbus_frame.h"
#include "modbus_registers.h"
#include "cmsis_os2.h"

#define MODBUS_MASTER_RX_MAX_SIZE 64U
#define MODBUS_MASTER_OP_NONE     0U
#define MODBUS_MASTER_OP_WRITE64  1U
#define MODBUS_MASTER_OP_READ64   2U
#define MODBUS_MASTER_STATUS_OK   0U
#define MODBUS_MASTER_STATUS_WAIT 1U
#define MODBUS_MASTER_STATUS_CRC  2U
#define MODBUS_MASTER_STATUS_FIELD 3U
#define MODBUS_MASTER_STATUS_ARG  4U

static volatile ModbusMaster_DebugTypeDef modbus_master_debug = {0};

static void ModbusMaster_SetDebug(uint32_t op,
                                  uint32_t status,
                                  uint16_t rx_len,
                                  const uint8_t *rx_frame,
                                  uint16_t reg_addr)
{
  modbus_master_debug.op = op;
  modbus_master_debug.status = status;
  modbus_master_debug.rx_len = rx_len;
  modbus_master_debug.rx_addr = (rx_len > 0U) ? rx_frame[0] : 0U;
  modbus_master_debug.rx_func = (rx_len > 1U) ? rx_frame[1] : 0U;
  modbus_master_debug.reg_addr = reg_addr;
}

void ModbusMaster_GetDebug(ModbusMaster_DebugTypeDef *debug)
{
  if (debug == NULL)
  {
    return;
  }

  __disable_irq();
  *debug = (ModbusMaster_DebugTypeDef)modbus_master_debug;
  __enable_irq();
}

static void ModbusMaster_WriteU16(uint8_t *data, uint16_t value)
{
  data[0] = (uint8_t)(value >> 8);
  data[1] = (uint8_t)(value & 0xFFU);
}

static uint16_t ModbusMaster_ReadU16(const uint8_t *data)
{
  return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static void ModbusMaster_WriteU64Regs(uint8_t *data, uint64_t value)
{
  uint8_t index;

  for (index = 0U; index < MODBUS_REGS_U64; index++)
  {
    ModbusMaster_WriteU16(&data[index * 2U], (uint16_t)((value >> (index * 16U)) & 0xFFFFU));
  }
}

static uint64_t ModbusMaster_ReadU64Regs(const uint8_t *data)
{
  uint64_t value = 0U;
  uint8_t index;

  for (index = 0U; index < MODBUS_REGS_U64; index++)
  {
    value |= ((uint64_t)ModbusMaster_ReadU16(&data[index * 2U])) << (index * 16U);
  }

  return value;
}

static void ModbusMaster_AppendCrc(uint8_t *frame, uint16_t len_without_crc)
{
  uint16_t crc = Modbus_Crc16(frame, len_without_crc);

  frame[len_without_crc] = (uint8_t)(crc & 0xFFU);
  frame[len_without_crc + 1U] = (uint8_t)(crc >> 8);
}

static uint8_t ModbusMaster_CheckCrc(const uint8_t *frame, uint16_t len)
{
  uint16_t expected_crc;
  uint16_t frame_crc;

  if ((frame == NULL) || (len < MODBUS_MIN_RTU_FRAME_LEN))
  {
    return 0U;
  }

  expected_crc = Modbus_Crc16(frame, (uint16_t)(len - 2U));
  frame_crc = (uint16_t)(((uint16_t)frame[len - 1U] << 8) | frame[len - 2U]);

  return (expected_crc == frame_crc) ? 1U : 0U;
}

static HAL_StatusTypeDef ModbusMaster_SendAndWait(const uint8_t *tx_frame,
                                                  uint16_t tx_len,
                                                  uint8_t *rx_frame,
                                                  uint16_t *rx_len,
                                                  uint32_t timeout_ms)
{
  uint32_t start_tick;
  uint16_t size = 0U;

  while (RS485_TakeRxFrame(rx_frame, &size, MODBUS_MASTER_RX_MAX_SIZE) != 0U)
  {
    size = 0U;
  }

  start_tick = HAL_GetTick();
  while (RS485_IsTxBusy() != 0U)
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  if (RS485_Send(tx_frame, tx_len) != HAL_OK)
  {
    return HAL_ERROR;
  }

  start_tick = HAL_GetTick();
  while (RS485_IsTxBusy() != 0U)
  {
    if ((uint32_t)(HAL_GetTick() - start_tick) >= timeout_ms)
    {
      return HAL_TIMEOUT;
    }
    osDelay(1);
  }

  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < timeout_ms)
  {
    if (RS485_TakeRxFrame(rx_frame, &size, MODBUS_MASTER_RX_MAX_SIZE) != 0U)
    {
      *rx_len = size;
      return HAL_OK;
    }
    osDelay(1);
  }

  return HAL_TIMEOUT;
}

HAL_StatusTypeDef ModbusMaster_WriteU64(uint8_t slave_addr, uint16_t reg_addr, uint64_t value, uint32_t timeout_ms)
{
  uint8_t tx_frame[17];
  uint8_t rx_frame[MODBUS_MASTER_RX_MAX_SIZE];
  uint16_t rx_len = 0U;

  tx_frame[0] = slave_addr;
  tx_frame[1] = MB_FC_WRITE_MULTIPLE_REGS;
  ModbusMaster_WriteU16(&tx_frame[2], reg_addr);
  ModbusMaster_WriteU16(&tx_frame[4], MODBUS_REGS_U64);
  tx_frame[6] = (uint8_t)(MODBUS_REGS_U64 * 2U);
  ModbusMaster_WriteU64Regs(&tx_frame[7], value);
  ModbusMaster_AppendCrc(tx_frame, 15U);

  if (ModbusMaster_SendAndWait(tx_frame, sizeof(tx_frame), rx_frame, &rx_len, timeout_ms) != HAL_OK)
  {
    ModbusMaster_SetDebug(MODBUS_MASTER_OP_WRITE64, MODBUS_MASTER_STATUS_WAIT, rx_len, rx_frame, reg_addr);
    return HAL_TIMEOUT;
  }

  if ((rx_len != MODBUS_READ_REQ_LEN) || (ModbusMaster_CheckCrc(rx_frame, rx_len) == 0U))
  {
    ModbusMaster_SetDebug(MODBUS_MASTER_OP_WRITE64, MODBUS_MASTER_STATUS_CRC, rx_len, rx_frame, reg_addr);
    return HAL_ERROR;
  }

  if ((rx_frame[0] != slave_addr) ||
      (rx_frame[1] != MB_FC_WRITE_MULTIPLE_REGS) ||
      (ModbusMaster_ReadU16(&rx_frame[2]) != reg_addr) ||
      (ModbusMaster_ReadU16(&rx_frame[4]) != MODBUS_REGS_U64))
  {
    ModbusMaster_SetDebug(MODBUS_MASTER_OP_WRITE64, MODBUS_MASTER_STATUS_FIELD, rx_len, rx_frame, reg_addr);
    return HAL_ERROR;
  }

  ModbusMaster_SetDebug(MODBUS_MASTER_OP_WRITE64, MODBUS_MASTER_STATUS_OK, rx_len, rx_frame, reg_addr);
  return HAL_OK;
}

HAL_StatusTypeDef ModbusMaster_ReadU64(uint8_t slave_addr, uint16_t reg_addr, uint64_t *value, uint32_t timeout_ms)
{
  uint8_t tx_frame[MODBUS_READ_REQ_LEN];
  uint8_t rx_frame[MODBUS_MASTER_RX_MAX_SIZE];
  uint16_t rx_len = 0U;

  if (value == NULL)
  {
    ModbusMaster_SetDebug(MODBUS_MASTER_OP_READ64, MODBUS_MASTER_STATUS_ARG, 0U, rx_frame, reg_addr);
    return HAL_ERROR;
  }

  tx_frame[0] = slave_addr;
  tx_frame[1] = MB_FC_READ_HOLDING_REGS;
  ModbusMaster_WriteU16(&tx_frame[2], reg_addr);
  ModbusMaster_WriteU16(&tx_frame[4], MODBUS_REGS_U64);
  ModbusMaster_AppendCrc(tx_frame, 6U);

  if (ModbusMaster_SendAndWait(tx_frame, sizeof(tx_frame), rx_frame, &rx_len, timeout_ms) != HAL_OK)
  {
    ModbusMaster_SetDebug(MODBUS_MASTER_OP_READ64, MODBUS_MASTER_STATUS_WAIT, rx_len, rx_frame, reg_addr);
    return HAL_TIMEOUT;
  }

  if ((rx_len != 13U) || (ModbusMaster_CheckCrc(rx_frame, rx_len) == 0U))
  {
    ModbusMaster_SetDebug(MODBUS_MASTER_OP_READ64, MODBUS_MASTER_STATUS_CRC, rx_len, rx_frame, reg_addr);
    return HAL_ERROR;
  }

  if ((rx_frame[0] != slave_addr) ||
      (rx_frame[1] != MB_FC_READ_HOLDING_REGS) ||
      (rx_frame[2] != (uint8_t)(MODBUS_REGS_U64 * 2U)))
  {
    ModbusMaster_SetDebug(MODBUS_MASTER_OP_READ64, MODBUS_MASTER_STATUS_FIELD, rx_len, rx_frame, reg_addr);
    return HAL_ERROR;
  }

  *value = ModbusMaster_ReadU64Regs(&rx_frame[3]);
  ModbusMaster_SetDebug(MODBUS_MASTER_OP_READ64, MODBUS_MASTER_STATUS_OK, rx_len, rx_frame, reg_addr);
  return HAL_OK;
}
