#include "gripper.h"
#include <string.h>

#define GRIPPER_FRAME_SYNC_TRIES 3U
#define GRIPPER_DMA_RX_CHUNK_SIZE 64U
#define GRIPPER_DMA_RX_RING_SIZE 512U

//计算一段数据的 Modbus CRC16 校验值
static uint16_t Gripper_Crc16Modbus(const uint8_t* data, uint16_t len)
{
  uint16_t crc = 0xFFFFU;

  while (len-- > 0U) {
    crc ^= (uint16_t)(*data++);
    for (uint8_t i = 0; i < 8U; i++) {
      if ((crc & 0x0001U) != 0U) {
        crc = (uint16_t)((crc >> 1) ^ 0xA001U);
      } else {
        crc >>= 1;
      }
    }
  }
  return crc;
}

//把 32 位无符号整数按小端格式写入字节缓冲区
static void Gripper_PutU32LE(uint8_t* buf, uint32_t val)
{
  buf[0] = (uint8_t)(val & 0xFFU);
  buf[1] = (uint8_t)((val >> 8) & 0xFFU);
  buf[2] = (uint8_t)((val >> 16) & 0xFFU);
  buf[3] = (uint8_t)((val >> 24) & 0xFFU);
}

//把 float 浮点数按小端格式写入字节缓冲区
static void Gripper_PutFloatLE(uint8_t* buf, float val)
{
  uint32_t raw = 0;
  memcpy(&raw, &val, sizeof(raw));
  Gripper_PutU32LE(buf, raw);
}

//从小端字节缓冲区读取 16 位无符号整数
static uint16_t Gripper_GetU16LE(const uint8_t* buf)
{
  return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

//从小端字节缓冲区读取 32 位有符号整数
static int32_t Gripper_GetI32LE(const uint8_t* buf)
{
  uint32_t raw = (uint32_t)buf[0] |
                 ((uint32_t)buf[1] << 8) |
                 ((uint32_t)buf[2] << 16) |
                 ((uint32_t)buf[3] << 24);
  return (int32_t)raw;
}

//从小端字节缓冲区读取 32 位无符号整数
static uint32_t Gripper_GetU32LE(const uint8_t* buf)
{
  return (uint32_t)buf[0] |
         ((uint32_t)buf[1] << 8) |
         ((uint32_t)buf[2] << 16) |
         ((uint32_t)buf[3] << 24);
}

//从小端字节缓冲区读取 float 浮点数
static float Gripper_GetFloatLE(const uint8_t* buf)
{
  uint32_t raw = Gripper_GetU32LE(buf);
  float val = 0.0f;
  memcpy(&val, &raw, sizeof(val));
  return val;
}

//计算 32 位有符号整数的绝对值
static int32_t Gripper_AbsI32(int32_t val)
{
  return (val < 0) ? -val : val;
}

//计算 float 浮点数的绝对值
static float Gripper_AbsF(float val)
{
  return (val < 0.0f) ? -val : val;
}


//解析回包结构实时状态， 并填充到 GripperRealtime_t 结构体中
static GripperResult_t Gripper_ParseRealtime(const uint8_t* payload,
                                             uint8_t payload_len,
                                             GripperRealtime_t* status)
{
  if ((payload == NULL) || (status == NULL)) {
    return GRIPPER_INVALID_ARG;
  }
  if (payload_len != 0x16U) {
    return GRIPPER_BAD_FRAME;
  }

  status->single_turn_raw = Gripper_GetU16LE(&payload[0]);
  status->multi_turn_count = Gripper_GetI32LE(&payload[2]);
  status->speed_raw = Gripper_GetI32LE(&payload[6]);
  status->q_current_raw = Gripper_GetI32LE(&payload[10]);
  status->bus_voltage_raw = Gripper_GetU16LE(&payload[14]);
  status->bus_current_raw = Gripper_GetU16LE(&payload[16]);
  status->temperature_c = payload[18];
  status->run_state = payload[19];
  status->motor_enabled = payload[20];
  status->fault_code = payload[21];

  return GRIPPER_OK;
}

//把夹爪回包里的 24 字节运动参数 payload位置环速度环系数，解析成 GripperMotionParams_t 结构体
static GripperResult_t Gripper_ParseMotionParams(const uint8_t* payload,
                                                 uint8_t payload_len,
                                                 GripperMotionParams_t* params)
{
  if ((payload == NULL) || (params == NULL)) {
    return GRIPPER_INVALID_ARG;
  }
  if (payload_len != 0x18U) {
    return GRIPPER_BAD_FRAME;
  }

  params->position_kp = Gripper_GetFloatLE(&payload[0]);
  params->position_ki = Gripper_GetFloatLE(&payload[4]);
  params->position_output_limit_raw = Gripper_GetU32LE(&payload[8]);
  params->velocity_kp = Gripper_GetFloatLE(&payload[12]);
  params->velocity_ki = Gripper_GetFloatLE(&payload[16]);
  params->velocity_output_limit_raw = Gripper_GetU32LE(&payload[20]);

  return GRIPPER_OK;
}

//把结构体里的运动参数变成 24 字节 payload，用来发送给夹爪
static void Gripper_BuildMotionParamsPayload(const GripperMotionParams_t* params,
                                             uint8_t* payload)
{
  Gripper_PutFloatLE(&payload[0], params->position_kp);
  Gripper_PutFloatLE(&payload[4], params->position_ki);
  Gripper_PutU32LE(&payload[8], params->position_output_limit_raw);
  Gripper_PutFloatLE(&payload[12], params->velocity_kp);
  Gripper_PutFloatLE(&payload[16], params->velocity_ki);
  Gripper_PutU32LE(&payload[20], params->velocity_output_limit_raw);
}

static UART_HandleTypeDef* Gripper_GetUart(uint8_t rs485_ch)
{
  if (rs485_ch == RS485A_CH) {
    return &huart1;
  }
  if (rs485_ch == RS485B_CH) {
    return &huart3;
  }
  if (rs485_ch == RS485C_CH) {
    return &huart4;
  }
  return NULL;
}

typedef struct {
  uint8_t rs485_ch;
  UART_HandleTypeDef* huart;
  uint8_t dma_buf[GRIPPER_DMA_RX_CHUNK_SIZE];
  uint8_t ring_buf[GRIPPER_DMA_RX_RING_SIZE];
  volatile uint16_t head;
  volatile uint16_t tail;
  volatile uint8_t active;
} GripperDmaRx_t;

static GripperDmaRx_t gripper_dma_rx_a = { RS485A_CH, &huart1, {0}, {0}, 0U, 0U, 0U };
static GripperDmaRx_t gripper_dma_rx_b = { RS485B_CH, &huart3, {0}, {0}, 0U, 0U, 0U };

static GripperDmaRx_t* Gripper_GetDmaRxByChannel(uint8_t rs485_ch)
{
  if (rs485_ch == RS485A_CH) {
    return &gripper_dma_rx_a;
  }
  if (rs485_ch == RS485B_CH) {
    return &gripper_dma_rx_b;
  }
  return NULL;
}

static GripperDmaRx_t* Gripper_GetDmaRxByUart(UART_HandleTypeDef* huart)
{
  if (huart == &huart1) {
    return &gripper_dma_rx_a;
  }
  if (huart == &huart3) {
    return &gripper_dma_rx_b;
  }
  return NULL;
}

static void Gripper_DmaRxReset(GripperDmaRx_t* rx)
{
  uint32_t primask;

  if (rx == NULL) {
    return;
  }
  primask = __get_PRIMASK();
  __disable_irq();
  rx->head = 0U;
  rx->tail = 0U;
  if (primask == 0U) {
    __enable_irq();
  }
}

static void Gripper_DmaRxPush(GripperDmaRx_t* rx, const uint8_t* data, uint16_t len)
{
  uint16_t i;
  uint16_t next;

  if ((rx == NULL) || (data == NULL)) {
    return;
  }

  for (i = 0U; i < len; i++) {
    next = (uint16_t)((rx->head + 1U) % GRIPPER_DMA_RX_RING_SIZE);
    if (next == rx->tail) {
      rx->tail = (uint16_t)((rx->tail + 1U) % GRIPPER_DMA_RX_RING_SIZE);
    }
    rx->ring_buf[rx->head] = data[i];
    rx->head = next;
  }
}

static HAL_StatusTypeDef Gripper_DmaRxReadByte(uint8_t rs485_ch,
                                               uint8_t* data,
                                               uint32_t timeout_ms)
{
  GripperDmaRx_t* rx = Gripper_GetDmaRxByChannel(rs485_ch);
  uint32_t start_tick = HAL_GetTick();

  if ((rx == NULL) || (data == NULL) || (rx->active == 0U)) {
    return HAL_ERROR;
  }

  while ((HAL_GetTick() - start_tick) < timeout_ms) {
    if (rx->tail != rx->head) {
      *data = rx->ring_buf[rx->tail];
      rx->tail = (uint16_t)((rx->tail + 1U) % GRIPPER_DMA_RX_RING_SIZE);
      return HAL_OK;
    }
  }

  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef Gripper_ReadByte(uint8_t rs485_ch,
                                          uint8_t* data,
                                          uint32_t timeout_ms)
{
  GripperDmaRx_t* rx = Gripper_GetDmaRxByChannel(rs485_ch);

  if ((rx != NULL) && (rx->active != 0U)) {
    return Gripper_DmaRxReadByte(rs485_ch, data, timeout_ms);
  }

  return RS485_Receive(rs485_ch, data, 1U, timeout_ms);
}

static HAL_StatusTypeDef Gripper_DmaRxStart(uint8_t rs485_ch)
{
  GripperDmaRx_t* rx = Gripper_GetDmaRxByChannel(rs485_ch);
  HAL_StatusTypeDef ret;

  if ((rx == NULL) || (rx->huart == NULL)) {
    return HAL_ERROR;
  }

  (void)HAL_UART_DMAStop(rx->huart);
  (void)HAL_UART_AbortReceive(rx->huart);
  Gripper_DmaRxReset(rx);

  ret = HAL_UARTEx_ReceiveToIdle_DMA(rx->huart,
                                     rx->dma_buf,
                                     GRIPPER_DMA_RX_CHUNK_SIZE);
  if (ret == HAL_OK) {
    rx->active = 1U;
    if (rx->huart->hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(rx->huart->hdmarx, DMA_IT_HT);
    }
  } else {
    rx->active = 0U;
  }

  return ret;
}

uint8_t Gripper_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  GripperDmaRx_t* rx = Gripper_GetDmaRxByUart(huart);

  if (rx == NULL) {
    return 0U;
  }

  if (rx->active == 0U) {
    return 0U;
  }

  if (Size > GRIPPER_DMA_RX_CHUNK_SIZE) {
    Size = GRIPPER_DMA_RX_CHUNK_SIZE;
  }

  Gripper_DmaRxPush(rx, rx->dma_buf, Size);

  if (HAL_UARTEx_ReceiveToIdle_DMA(rx->huart,
                                   rx->dma_buf,
                                   GRIPPER_DMA_RX_CHUNK_SIZE) == HAL_OK) {
    rx->active = 1U;
    if (rx->huart->hdmarx != NULL) {
      __HAL_DMA_DISABLE_IT(rx->huart->hdmarx, DMA_IT_HT);
    }
  } else {
    rx->active = 0U;
  }

  return 1U;
}

static void Gripper_FlushRx(uint8_t rs485_ch)
{
  UART_HandleTypeDef* huart = Gripper_GetUart(rs485_ch);
  GripperDmaRx_t* rx = Gripper_GetDmaRxByChannel(rs485_ch);

  if (huart == NULL) {
    return;
  }

  if ((rx != NULL) && (rx->active != 0U)) {
    Gripper_DmaRxReset(rx);
  } else {
    (void)HAL_UART_AbortReceive(huart);
  }
  __HAL_UART_CLEAR_IDLEFLAG(huart);
  __HAL_UART_CLEAR_OREFLAG(huart);
  __HAL_UART_CLEAR_FEFLAG(huart);
  __HAL_UART_CLEAR_NEFLAG(huart);
  __HAL_UART_FLUSH_DRREGISTER(huart);
}

static GripperResult_t Gripper_ReadFrame(GripperHandle_t* handle,
                                         GripperCommand_t cmd,
                                         uint8_t seq,
                                         uint8_t* resp_payload,
                                         uint8_t* resp_len)
{
  uint8_t rx[GRIPPER_MAX_FRAME_LEN];
  uint8_t b = 0U;
  uint32_t start_tick;
  uint16_t rx_len;
  uint8_t payload_len;
  uint16_t frame_len;
  uint16_t crc_calc;
  uint16_t crc_recv;
  uint8_t saw_bad_frame = 0U;

  if (resp_len != NULL) {
    *resp_len = 0U;
  }

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }

  start_tick = HAL_GetTick();

  while ((HAL_GetTick() - start_tick) < handle->timeout_ms) {
    if (Gripper_ReadByte(handle->rs485_ch, &b, 1U) != HAL_OK) {
      continue;
    }

    if (b != GRIPPER_FRAME_RX_HEAD) {
      continue;
    }

    rx[0] = b;
    rx_len = 1U;

    while ((rx_len < 5U) &&
           ((HAL_GetTick() - start_tick) < handle->timeout_ms)) {
      if (Gripper_ReadByte(handle->rs485_ch, &rx[rx_len], 1U) == HAL_OK) {
        rx_len++;
      }
    }

    if (rx_len < 5U) {
      continue;
    }

    payload_len = rx[4];
    if (payload_len > GRIPPER_MAX_PAYLOAD_LEN) {
      saw_bad_frame = 1U;
      continue;
    }

    frame_len = (uint16_t)(5U + payload_len + 2U);

    while ((rx_len < frame_len) &&
           ((HAL_GetTick() - start_tick) < handle->timeout_ms)) {
      if (Gripper_ReadByte(handle->rs485_ch, &rx[rx_len], 1U) == HAL_OK) {
        rx_len++;
      }
    }

    if (rx_len < frame_len) {
      continue;
    }

    crc_calc = Gripper_Crc16Modbus(rx, (uint16_t)(5U + payload_len));

    crc_recv = (uint16_t)rx[5U + payload_len] |
               ((uint16_t)rx[6U + payload_len] << 8);

    if (crc_calc != crc_recv) {
      saw_bad_frame = 1U;
      continue;
    }

    if (rx[1] != seq) {
      saw_bad_frame = 1U;
      continue;
    }

    if (rx[3] != (uint8_t)cmd) {
      saw_bad_frame = 1U;
      continue;
    }

    if ((handle->device_addr != GRIPPER_PUBLIC_ADDR) &&
        (rx[2] != handle->device_addr)) {
      saw_bad_frame = 1U;
      continue;
    }

    if ((resp_payload != NULL) && (payload_len > 0U)) {
      memcpy(resp_payload, &rx[5], payload_len);
    }

    if (resp_len != NULL) {
      *resp_len = payload_len;
    }

    handle->last_error = GRIPPER_OK;
    return GRIPPER_OK;
  }

  handle->last_error = (saw_bad_frame != 0U) ? GRIPPER_BAD_FRAME : GRIPPER_TIMEOUT;
  return handle->last_error;
}


//负责组帧、加 CRC、发送、接收回包、校验回包、取出 payload
static GripperResult_t Gripper_Transact(GripperHandle_t* handle,
                                        GripperCommand_t cmd,
                                        const uint8_t* payload,
                                        uint8_t payload_len,
                                        uint8_t* resp_payload,
                                        uint8_t* resp_len,
                                        bool expect_response)
{
  uint8_t tx[GRIPPER_MAX_FRAME_LEN];
  uint16_t tx_len;
  uint16_t crc;
  uint8_t seq;
  bool read_response;

  if (resp_len != NULL) {
    *resp_len = 0U;
  }

  if ((handle == NULL) || ((payload_len > 0U) && (payload == NULL))) {
    return GRIPPER_INVALID_ARG;
  }

  if (payload_len > GRIPPER_MAX_PAYLOAD_LEN) {
    return GRIPPER_INVALID_ARG;
  }

  read_response = expect_response &&
                  (handle->device_addr != GRIPPER_BROADCAST_ADDR);

  seq = handle->next_seq++;

  tx[0] = GRIPPER_FRAME_TX_HEAD;
  tx[1] = seq;
  tx[2] = handle->device_addr;
  tx[3] = (uint8_t)cmd;
  tx[4] = payload_len;

  if (payload_len > 0U) {
    memcpy(&tx[5], payload, payload_len);
  }

  tx_len = (uint16_t)(5U + payload_len);
  crc = Gripper_Crc16Modbus(tx, tx_len);

  tx[tx_len++] = (uint8_t)(crc & 0xFFU);
  tx[tx_len++] = (uint8_t)((crc >> 8) & 0xFFU);

  /*
   * 发送前清空旧 RX，防止读到上一帧残留。
   * 即使不读回复，也可以考虑清一下，尤其是 RS485 总线调试阶段。
   */
  Gripper_FlushRx(handle->rs485_ch);

  if (RS485_Send(handle->rs485_ch, tx, tx_len, handle->timeout_ms) != HAL_OK) {
    handle->last_error = GRIPPER_ERROR;
    return GRIPPER_ERROR;
  }

  if (!read_response) {
    /*
     * 如果是广播地址，并且设备可能回包，建议短延时后再清一次 RX。
     * 如果协议明确广播不回包，可以删掉这段。
     */
    if (handle->device_addr == GRIPPER_BROADCAST_ADDR) {
      HAL_Delay(5);
      Gripper_FlushRx(handle->rs485_ch);
    }

    handle->last_error = GRIPPER_OK;
    return GRIPPER_OK;
  }

  return Gripper_ReadFrame(handle, cmd, seq, resp_payload, resp_len);
}

//初始化
void Gripper_Init(GripperHandle_t* handle,
                  uint8_t rs485_ch,
                  uint8_t device_addr,
                  uint16_t timeout_ms)
{
  if (handle == NULL) {
    return;
  }

  memset(handle, 0, sizeof(*handle));
  handle->rs485_ch = rs485_ch;
  handle->device_addr = device_addr;
  handle->timeout_ms = (timeout_ms == 0U) ? GRIPPER_DEFAULT_TIMEOUT_MS : timeout_ms;
  handle->last_error = GRIPPER_OK;
  handle->open_position_count = GRIPPER_DEFAULT_OPEN_COUNT;
  handle->close_position_count = GRIPPER_DEFAULT_CLOSE_COUNT;
  handle->deadband_percent = GRIPPER_DEFAULT_DEADBAND_PERCENT;
  handle->calibrated = 0U;

  (void)Gripper_DmaRxStart(rs485_ch);
}

//读取夹爪实时状态的上层接口
GripperResult_t Gripper_ReadRealtime(GripperHandle_t* handle,
                                      GripperRealtime_t* status)
{
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  ret = Gripper_Transact(handle, GRIPPER_CMD_READ_REALTIME, NULL, 0U,
                         payload, &len, true);
  if (ret != GRIPPER_OK) {
    return ret;
  }

  ret = Gripper_ParseRealtime(payload, len, status);
  if ((ret == GRIPPER_OK) && (handle != NULL) && (status != NULL)) {
    handle->last_status = *status;
  }
  if (handle != NULL) {
    handle->last_error = ret;
  }
  return ret;
}

//读的是夹爪版本信息，不是实时状态
GripperResult_t Gripper_ReadVersion(GripperHandle_t* handle,
                                    GripperVersion_t* version)
{
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  if ((handle == NULL) || (version == NULL)) {
    return GRIPPER_INVALID_ARG;
  }

  ret = Gripper_Transact(handle, GRIPPER_CMD_READ_VERSION, NULL, 0U,
                         payload, &len, true);
  if (ret != GRIPPER_OK) {
    return ret;
  }
  if (len != 0x16U) {
    handle->last_error = GRIPPER_BAD_FRAME;
    return GRIPPER_BAD_FRAME;
  }

  version->boot_ver = Gripper_GetU16LE(&payload[0]);
  version->software_ver = Gripper_GetU16LE(&payload[2]);
  version->hardware_ver = Gripper_GetU16LE(&payload[4]);
  version->aux_rs485_ver = payload[6];
  version->modbus_rs485_ver = payload[7];
  version->aux_can_ver = payload[8];
  version->canopen_ver = payload[9];
  memcpy(version->uid, &payload[10], sizeof(version->uid));

  return GRIPPER_OK;
}

//清除夹爪故障的接口函数
GripperResult_t Gripper_ClearFault(GripperHandle_t* handle, uint8_t* fault_code)
{
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  ret = Gripper_Transact(handle, GRIPPER_CMD_CLEAR_FAULT, NULL, 0U,
                         payload, &len, (fault_code != NULL));
  if (ret != GRIPPER_OK) {
    return ret;
  }
  if ((handle != NULL) && (handle->device_addr == GRIPPER_BROADCAST_ADDR)) {
    if (fault_code != NULL) {
      *fault_code = 0U;
    }
    return GRIPPER_OK;
  }
  if (fault_code == NULL) {
    return GRIPPER_OK;
  }
  if (len != 1U) {
    handle->last_error = GRIPPER_BAD_FRAME;
    return GRIPPER_BAD_FRAME;
  }
  if (fault_code != NULL) {
    *fault_code = payload[0];
  }
  return GRIPPER_OK;
}

//夹爪绝对位置运动命令
GripperResult_t Gripper_MoveAbsolute(GripperHandle_t* handle,
                                      int32_t target_count,
                                      GripperRealtime_t* status)
{
  uint8_t req[4];
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  Gripper_PutU32LE(req, (uint32_t)target_count);
  ret = Gripper_Transact(handle, GRIPPER_CMD_MOVE_ABSOLUTE, req, sizeof(req),
                         payload, &len, (status != NULL));
  if (ret != GRIPPER_OK) {
    return ret;
  }
  if ((handle != NULL) && (handle->device_addr == GRIPPER_BROADCAST_ADDR)) {
    if (status != NULL) {
      memset(status, 0, sizeof(*status));
      status->multi_turn_count = target_count;
    }
    return GRIPPER_OK;
  }
  if (status == NULL) {
    return GRIPPER_OK;
  }

  ret = Gripper_ParseRealtime(payload, len, status);
  if ((ret == GRIPPER_OK) && (handle != NULL) && (status != NULL)) {
    handle->last_status = *status;
  }
  return ret;
}

//按 rpm 和电流单位设置临时限速限流后，再执行绝对位置运动
GripperResult_t Gripper_MoveAbsoluteWithLimits(GripperHandle_t* handle,
                                               int32_t target_count,
                                               float max_speed_rpm,
                                               float max_current_amp,
                                               GripperRealtime_t* status)
{
  uint32_t speed_limit_raw = 0;
  uint32_t current_limit_raw = 0;

  if (max_speed_rpm > 0.0f) {
    speed_limit_raw = (uint32_t)Gripper_RpmToRaw(max_speed_rpm);
  }
  if (max_current_amp > 0.0f) {
    current_limit_raw = (uint32_t)Gripper_AmpToRaw(max_current_amp);
  }

  return Gripper_MoveAbsoluteWithLimitsRaw(handle, target_count,
                                           speed_limit_raw,
                                           current_limit_raw,
                                           status);
}

//按 raw 原始值设置临时限速限流后，再执行绝对位置运动
GripperResult_t Gripper_MoveAbsoluteWithLimitsRaw(GripperHandle_t* handle,
                                                  int32_t target_count,
                                                  uint32_t speed_limit_raw,
                                                  uint32_t current_limit_raw,
                                                  GripperRealtime_t* status)
{
  uint8_t motion_payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  if ((speed_limit_raw > 0U) || (current_limit_raw > 0U)) {
    ret = Gripper_Transact(handle, GRIPPER_CMD_READ_MOTION_PARAMS, NULL, 0U,
                           motion_payload, &len, true);
    if (ret != GRIPPER_OK) {
      return ret;
    }
    if (len != 0x18U) {
      if (handle != NULL) {
        handle->last_error = GRIPPER_BAD_FRAME;
      }
      return GRIPPER_BAD_FRAME;
    }

    if (speed_limit_raw > 0U) {
      Gripper_PutU32LE(&motion_payload[8], speed_limit_raw);
    }
    if (current_limit_raw > 0U) {
      Gripper_PutU32LE(&motion_payload[20], current_limit_raw);
    }

    ret = Gripper_Transact(handle, GRIPPER_CMD_WRITE_MOTION_PARAMS_TEMP,
                           motion_payload, 0x18U,
                           motion_payload, &len, true);
    if (ret != GRIPPER_OK) {
      return ret;
    }
    if (len != 0x18U) {
      if (handle != NULL) {
        handle->last_error = GRIPPER_BAD_FRAME;
      }
      return GRIPPER_BAD_FRAME;
    }
  }

  return Gripper_MoveAbsolute(handle, target_count, status);
}

//发送夹爪相对位置运动命令，可选读取返回的实时状态
GripperResult_t Gripper_MoveRelative(GripperHandle_t* handle,
                                      int32_t delta_count,
                                      GripperRealtime_t* status)
{
  uint8_t req[4];
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  Gripper_PutU32LE(req, (uint32_t)delta_count);
  ret = Gripper_Transact(handle, GRIPPER_CMD_MOVE_RELATIVE, req, sizeof(req),
                         payload, &len, (status != NULL));
  if (ret != GRIPPER_OK) {
    return ret;
  }
  if ((handle != NULL) && (handle->device_addr == GRIPPER_BROADCAST_ADDR)) {
    if (status != NULL) {
      memset(status, 0, sizeof(*status));
      status->multi_turn_count = delta_count;
    }
    return GRIPPER_OK;
  }
  if (status == NULL) {
    return GRIPPER_OK;
  }

  return Gripper_ParseRealtime(payload, len, status);
}

//发送夹爪最短路径回零命令，可选读取返回的实时状态
GripperResult_t Gripper_GoHomeShortest(GripperHandle_t* handle,
                                       GripperRealtime_t* status)
{
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  ret = Gripper_Transact(handle, GRIPPER_CMD_HOME_SHORTEST, NULL, 0U,
                         payload, &len, (status != NULL));
  if (ret != GRIPPER_OK) {
    return ret;
  }
  if ((handle != NULL) && (handle->device_addr == GRIPPER_BROADCAST_ADDR)) {
    if (status != NULL) {
      memset(status, 0, sizeof(*status));
    }
    return GRIPPER_OK;
  }
  if (status == NULL) {
    return GRIPPER_OK;
  }

  return Gripper_ParseRealtime(payload, len, status);
}

//发送速度控制命令，设置目标转速和加速度
GripperResult_t Gripper_SetSpeed(GripperHandle_t* handle,
                                  float rpm,
                                  uint32_t accel_0p01rpm_per_sec,
                                  GripperRealtime_t* status)
{
  uint8_t req[8];
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;
  int32_t speed_raw;

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }

  speed_raw = Gripper_RpmToRaw(rpm);

  Gripper_PutU32LE(&req[0], (uint32_t)speed_raw);
  Gripper_PutU32LE(&req[4], accel_0p01rpm_per_sec);

  ret = Gripper_Transact(handle, GRIPPER_CMD_SPEED, req, sizeof(req),
                         payload, &len, (status != NULL));
  if (ret != GRIPPER_OK) {
    return ret;
  }

  /*
   * 广播地址不会等待回复。
   * 命令已经发出，不能再解析 payload。
   */
  if (handle->device_addr == GRIPPER_BROADCAST_ADDR) {
    if (status != NULL) {
      memset(status, 0, sizeof(*status));
      status->speed_raw = speed_raw;
    }
    return GRIPPER_OK;
  }
  if (status == NULL) {
    return GRIPPER_OK;
  }

  return Gripper_ParseRealtime(payload, len, status);
}


// 速度控制 + 临时电流限制
GripperResult_t Gripper_SetSpeedWithCurrentLimit(GripperHandle_t* handle,
                                                 float rpm,
                                                 uint32_t accel_0p01rpm_per_sec,
                                                 float current_limit_a,
                                                 GripperRealtime_t* status)
{
  GripperResult_t ret;
  GripperMotionParams_t params;
  GripperMotionParams_t echo;
  uint32_t current_limit_raw = 0;

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }

  if (current_limit_a <= 0.0f) {
    return Gripper_SetSpeed(handle, rpm, accel_0p01rpm_per_sec, status);
  }

  memset(&params, 0, sizeof(params));
  memset(&echo, 0, sizeof(echo));

  ret = Gripper_ReadMotionParams(handle, &params);
  if (ret != GRIPPER_OK) {
    return ret;
  }

  /*
   * 2. 修改电流限制
   *
   * 你的驱动里电流换算是：
   * Gripper_AmpToRaw(amp) = amp * 1000
   *
   * 0.5A -> 500
   * 0.8A -> 800
   * 1.0A -> 1000
   * 2.0A -> 2000
   */
  current_limit_raw = (uint32_t)Gripper_AmpToRaw(current_limit_a);
  params.velocity_output_limit_raw = current_limit_raw;

  ret = Gripper_WriteMotionParamsTemp(handle, &params, &echo);
  if (ret != GRIPPER_OK) {
    return ret;
  }
  return Gripper_SetSpeed(handle, rpm, accel_0p01rpm_per_sec, status);
}






//发送 q 轴电流控制命令，设置目标电流和电流斜率
GripperResult_t Gripper_SetQCurrent(GripperHandle_t* handle,
                                     float amp,
                                     uint32_t slope_milliamp_per_sec,
                                     GripperRealtime_t* status)
{
  uint8_t req[8];
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;
  int32_t current_raw;

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }

  current_raw = Gripper_AmpToRaw(amp);

  Gripper_PutU32LE(&req[0], (uint32_t)current_raw);
  Gripper_PutU32LE(&req[4], slope_milliamp_per_sec);
  ret = Gripper_Transact(handle, GRIPPER_CMD_Q_CURRENT, req, sizeof(req),
                         payload, &len, (status != NULL));
  if (ret != GRIPPER_OK) {
    return ret;
  }
  if (handle->device_addr == GRIPPER_BROADCAST_ADDR) {
    if (status != NULL) {
      memset(status, 0, sizeof(*status));
      status->q_current_raw = current_raw;
    }
    return GRIPPER_OK;
  }
  if (status == NULL) {
    return GRIPPER_OK;
  }

  return Gripper_ParseRealtime(payload, len, status);
}

//发送失能命令关闭夹爪电机输出
GripperResult_t Gripper_Disable(GripperHandle_t* handle,
                                 GripperRealtime_t* status)
{
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }

  ret = Gripper_Transact(handle, GRIPPER_CMD_DISABLE, NULL, 0U,
                         payload, &len, (status != NULL));
  if (ret != GRIPPER_OK) {
    return ret;
  }

  /*
   * 广播地址不会有回复，不能解析 payload。
   */
  if (handle->device_addr == GRIPPER_BROADCAST_ADDR) {
    if (status != NULL) {
      memset(status, 0, sizeof(*status));
    }
    return GRIPPER_OK;
  }
  if (status == NULL) {
    return GRIPPER_OK;
  }

  return Gripper_ParseRealtime(payload, len, status);
}

//发送当前位置置零命令，可选读取机械偏移量
GripperResult_t Gripper_SetZero(GripperHandle_t* handle, uint16_t* mechanical_offset)
{
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  ret = Gripper_Transact(handle, GRIPPER_CMD_SET_ZERO, NULL, 0U,
                         payload, &len, (mechanical_offset != NULL));
  if (ret != GRIPPER_OK) {
    return ret;
  }
  if ((handle != NULL) && (handle->device_addr == GRIPPER_BROADCAST_ADDR)) {
    if (mechanical_offset != NULL) {
      *mechanical_offset = 0U;
    }
    return GRIPPER_OK;
  }
  if (mechanical_offset == NULL) {
    return GRIPPER_OK;
  }
  if (len != 2U) {
    handle->last_error = GRIPPER_BAD_FRAME;
    return GRIPPER_BAD_FRAME;
  }
  if (mechanical_offset != NULL) {
    *mechanical_offset = Gripper_GetU16LE(payload);
  }
  return GRIPPER_OK;
}

//读取夹爪内部运动控制参数
GripperResult_t Gripper_ReadMotionParams(GripperHandle_t* handle,
                                         GripperMotionParams_t* params)
{
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  ret = Gripper_Transact(handle, GRIPPER_CMD_READ_MOTION_PARAMS, NULL, 0U,
                         payload, &len, true);
  if (ret != GRIPPER_OK) {
    return ret;
  }

  ret = Gripper_ParseMotionParams(payload, len, params);
  if (handle != NULL) {
    handle->last_error = ret;
  }
  return ret;
}

//写入夹爪运动控制参数，可选读取回显参数
static GripperResult_t Gripper_WriteMotionParams(GripperHandle_t* handle,
                                                 GripperCommand_t cmd,
                                                 const GripperMotionParams_t* params,
                                                 GripperMotionParams_t* echo)
{
  uint8_t req[0x18];
  uint8_t payload[GRIPPER_MAX_PAYLOAD_LEN];
  uint8_t len = 0;
  GripperResult_t ret;

  if ((handle == NULL) || (params == NULL)) {
    return GRIPPER_INVALID_ARG;
  }

  Gripper_BuildMotionParamsPayload(params, req);
  ret = Gripper_Transact(handle, cmd, req, sizeof(req), payload, &len, true);
  if (ret != GRIPPER_OK) {
    return ret;
  }
  if (handle->device_addr == GRIPPER_BROADCAST_ADDR) {
    if (echo != NULL) {
      *echo = *params;
    }
    return GRIPPER_OK;
  }
  if (echo != NULL) {
    return Gripper_ParseMotionParams(payload, len, echo);
  }
  return GRIPPER_OK;
}

//临时写入夹爪运动控制参数，掉电后不保存
GripperResult_t Gripper_WriteMotionParamsTemp(GripperHandle_t* handle,
                                              const GripperMotionParams_t* params,
                                              GripperMotionParams_t* echo)
{
  return Gripper_WriteMotionParams(handle, GRIPPER_CMD_WRITE_MOTION_PARAMS_TEMP,
                                   params, echo);
}

//写入并保存夹爪运动控制参数，掉电后保留
GripperResult_t Gripper_WriteMotionParamsSave(GripperHandle_t* handle,
                                              const GripperMotionParams_t* params,
                                              GripperMotionParams_t* echo)
{
  return Gripper_WriteMotionParams(handle, GRIPPER_CMD_WRITE_MOTION_PARAMS_SAVE,
                                   params, echo);
}

//生成夹爪限位标定的默认配置参数
void Gripper_CalibrateConfigDefault(GripperCalibrateConfig_t* config)
{
  if (config == NULL) {
    return;
  }

  config->search_speed_rpm = 100.0f;
  config->search_direction = 1;
  config->poll_interval_ms = 20;
  config->timeout_ms = 5000;
  config->speed_epsilon_rpm = 0.5f;
  config->current_threshold_a = 0.6f;
  config->position_epsilon_count = 5;
  config->detect_consecutive_samples = 4;
  config->clear_fault_before_start = 1;
  config->set_zero_after_detect = 1;
  config->backoff_count_after_zero = -15000;
}

//生成夹爪上电启动标定的默认配置参数
void Gripper_StartupConfigDefault(GripperStartupConfig_t* config)
{
  if (config == NULL) {
    return;
  }

  Gripper_CalibrateConfigDefault(&config->calibrate);
  config->close_position_count = GRIPPER_DEFAULT_CLOSE_COUNT;
  config->close_speed_limit_rpm = 100.0f;
  config->close_current_limit_a = 0.8f;
  config->close_after_calibrate = 1U;
}

//执行夹爪限位搜索标定，检测堵转限位并可置零回退
GripperResult_t Gripper_CalibrateLimit(GripperHandle_t* handle,
                                       const GripperCalibrateConfig_t* config,
                                       GripperCalibrateResult_t* result)
{
  GripperCalibrateConfig_t cfg;
  GripperRealtime_t latest;
  uint32_t start_tick;
  uint8_t current_fault = 0;
  uint8_t has_prev = 0;
  uint8_t hits = 0;
  int32_t prev_count = 0;
  GripperResult_t ret;

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }
  if (handle->device_addr == GRIPPER_BROADCAST_ADDR) {
    return GRIPPER_INVALID_ARG;
  }

  if (config == NULL) {
    Gripper_CalibrateConfigDefault(&cfg);
  } else {
    cfg = *config;
  }

  if ((cfg.search_direction != 1) && (cfg.search_direction != -1)) {
    return GRIPPER_INVALID_ARG;
  }
  if ((cfg.search_speed_rpm <= 0.0f) ||
      (cfg.poll_interval_ms == 0U) ||
      (cfg.timeout_ms == 0U) ||
      (cfg.detect_consecutive_samples == 0U) ||
      (cfg.position_epsilon_count < 0)) {
    return GRIPPER_INVALID_ARG;
  }

  if (result != NULL) {
    memset(result, 0, sizeof(*result));
  }

  if (cfg.clear_fault_before_start != 0U) {
    ret = Gripper_ClearFault(handle, &current_fault);
    if (ret != GRIPPER_OK) {
      return ret;
    }
  }

  ret = Gripper_SetSpeed(handle,
                         (float)cfg.search_direction * cfg.search_speed_rpm,
                         0U,
                         NULL);
  if (ret != GRIPPER_OK) {
    return ret;
  }

  start_tick = HAL_GetTick();
  while ((uint32_t)(HAL_GetTick() - start_tick) < cfg.timeout_ms) {
    int32_t delta_count = 0;
    uint8_t had_prev;
    uint8_t speed_small;
    uint8_t current_high;
    uint8_t position_locked;

    HAL_Delay(cfg.poll_interval_ms);

    ret = Gripper_ReadRealtime(handle, &latest);
    if (ret != GRIPPER_OK) {
      (void)Gripper_Disable(handle, NULL);
      return ret;
    }

    if (latest.fault_code != 0U) {
      (void)Gripper_Disable(handle, NULL);
      handle->last_error = GRIPPER_ERROR;
      return GRIPPER_ERROR;
    }

    had_prev = has_prev;
    if (had_prev != 0U) {
      delta_count = Gripper_AbsI32(latest.multi_turn_count - prev_count);
    }
    prev_count = latest.multi_turn_count;
    has_prev = 1U;

    speed_small = (Gripper_AbsF(Gripper_RawToRpm(latest.speed_raw)) <= cfg.speed_epsilon_rpm) ? 1U : 0U;
    current_high = (Gripper_AbsF(Gripper_RawToAmp(latest.q_current_raw)) >= cfg.current_threshold_a) ? 1U : 0U;
    position_locked = ((had_prev != 0U) && (delta_count <= cfg.position_epsilon_count)) ? 1U : 0U;

    if ((speed_small != 0U) && (current_high != 0U) && (position_locked != 0U)) {
      hits++;
    } else {
      hits = 0U;
    }

    if (hits >= cfg.detect_consecutive_samples) {
      uint16_t mechanical_offset = 0;

      (void)Gripper_SetSpeed(handle, 0.0f, 0U, NULL);
      (void)Gripper_Disable(handle, NULL);

      if (result != NULL) {
        result->limit_detected = 1U;
        result->detect_samples = hits;
        result->limit_count_before_zero = latest.multi_turn_count;
      }

      if (cfg.set_zero_after_detect != 0U) {
        ret = Gripper_SetZero(handle, &mechanical_offset);
        if (ret != GRIPPER_OK) {
          return ret;
        }
        if (result != NULL) {
          result->zero_set = 1U;
          result->mechanical_offset = mechanical_offset;
        }
      }

      if (cfg.backoff_count_after_zero != 0) {
        ret = Gripper_MoveRelative(handle, cfg.backoff_count_after_zero, &latest);
        if (ret != GRIPPER_OK) {
          return ret;
        }
        if (result != NULL) {
          result->backoff_done = 1U;
        }
      }

      ret = Gripper_ReadRealtime(handle, &latest);
      if (ret != GRIPPER_OK) {
        return ret;
      }
      if (result != NULL) {
        result->final_status = latest;
      }
      if (cfg.set_zero_after_detect != 0U) {
        handle->open_position_count = 0;
      } else {
        handle->open_position_count = latest.multi_turn_count;
      }
      handle->calibrated = 1U;

      return GRIPPER_OK;
    }
  }

  (void)Gripper_SetSpeed(handle, 0.0f, 0U, NULL);
  (void)Gripper_Disable(handle, NULL);
  handle->last_error = GRIPPER_TIMEOUT;
  return GRIPPER_TIMEOUT;
}

//执行上电开闭标定流程，标定后可自动移动到闭合位置
GripperResult_t Gripper_StartupCalibrateOpenClose(GripperHandle_t* handle,
                                                  const GripperStartupConfig_t* config,
                                                  GripperCalibrateResult_t* result)
{
  GripperStartupConfig_t cfg;
  GripperResult_t ret;

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }

  if (config == NULL) {
    Gripper_StartupConfigDefault(&cfg);
  } else {
    cfg = *config;
  }

  ret = Gripper_CalibrateLimit(handle, &cfg.calibrate, result);
  if (ret != GRIPPER_OK) {
    return ret;
  }

  handle->close_position_count = cfg.close_position_count;

  if (cfg.close_after_calibrate != 0U) {
    ret = Gripper_MoveAbsoluteWithLimits(handle,
                                         handle->close_position_count,
                                         cfg.close_speed_limit_rpm,
                                         cfg.close_current_limit_a,
                                         NULL);
    if (ret != GRIPPER_OK) {
      return ret;
    }
  }

  return GRIPPER_OK;
}

//发送夹爪重启命令，不等待回包
GripperResult_t Gripper_Reboot(GripperHandle_t* handle)
{
  return Gripper_Transact(handle, GRIPPER_CMD_REBOOT, NULL, 0U,
                          NULL, NULL, false);
}

//将 rpm 转换为协议使用的速度原始值
int32_t Gripper_RpmToRaw(float rpm)
{
  return (int32_t)(rpm * 100.0f);
}

//将安培转换为协议使用的电流原始值
int32_t Gripper_AmpToRaw(float amp)
{
  return (int32_t)(amp * 1000.0f);
}

//将速度原始值转换为 rpm
float Gripper_RawToRpm(int32_t raw)
{
  return (float)raw * 0.01f;
}

//将电流原始值转换为安培
float Gripper_RawToAmp(int32_t raw)
{
  return (float)raw * 0.001f;
}

//将编码器计数转换为角度值
float Gripper_CountToDeg(int32_t count)
{
  return (float)count * (360.0f / 16384.0f);
}

//设置夹爪张开和闭合位置计数范围
void Gripper_SetPositionProfile(GripperHandle_t* handle,
                                int32_t open_count,
                                int32_t close_count)
{
  if (handle == NULL) {
    return;
  }

  handle->open_position_count = open_count;
  handle->close_position_count = close_count;
}

void Gripper_SetDeadbandPercent(GripperHandle_t* handle, float deadband_percent)
{
  if (handle == NULL) {
    return;
  }
  if (deadband_percent < 0.0f) {
    deadband_percent = 0.0f;
  }
  if (deadband_percent > 100.0f) {
    deadband_percent = 100.0f;
  }
  handle->deadband_percent = deadband_percent;
}

//将开合百分比转换为目标位置计数
int32_t Gripper_PercentToCount(const GripperHandle_t* handle, float percent)
{
  float clamped = percent;
  float ratio;
  int32_t open_count;
  int32_t close_count;

  if (clamped < 0.0f) {
    clamped = 0.0f;
  }
  if (clamped > 100.0f) {
    clamped = 100.0f;
  }

  open_count = (handle == NULL) ? GRIPPER_DEFAULT_OPEN_COUNT : handle->open_position_count;
  close_count = (handle == NULL) ? GRIPPER_DEFAULT_CLOSE_COUNT : handle->close_position_count;

  ratio = clamped / 100.0f;
  return close_count + (int32_t)((float)(open_count - close_count) * ratio);
}

//将当前位置计数转换为开合百分比
float Gripper_CountToPercent(const GripperHandle_t* handle, int32_t count)
{
  int32_t open_count;
  int32_t close_count;
  int32_t span;
  float percent;

  open_count = (handle == NULL) ? GRIPPER_DEFAULT_OPEN_COUNT : handle->open_position_count;
  close_count = (handle == NULL) ? GRIPPER_DEFAULT_CLOSE_COUNT : handle->close_position_count;
  span = open_count - close_count;
  if (span == 0) {
    return 0.0f;
  }

  percent = ((float)(count - close_count) / (float)span) * 100.0f;
  if (percent < 0.0f) {
    percent = 0.0f;
  }
  if (percent > 100.0f) {
    percent = 100.0f;
  }
  return percent;
}

//按开合百分比执行位置运动
uint8_t Gripper_IsPercentInDeadband(const GripperHandle_t* handle,
                                    float target_percent,
                                    int32_t current_count)
{
  float deadband;
  float half_deadband;
  float current_percent;
  float diff;

  if (handle == NULL) {
    return 0U;
  }

  deadband = handle->deadband_percent;
  if (deadband <= 0.0f) {
    return 0U;
  }

  if (target_percent < 0.0f) {
    target_percent = 0.0f;
  }
  if (target_percent > 100.0f) {
    target_percent = 100.0f;
  }

  current_percent = Gripper_CountToPercent(handle, current_count);
  diff = current_percent - target_percent;
  if (diff < 0.0f) {
    diff = -diff;
  }

  half_deadband = deadband * 0.5f;
  return (diff <= half_deadband) ? 1U : 0U;
}

GripperResult_t Gripper_MoveToPercent(GripperHandle_t* handle,
                                      float percent,
                                      GripperRealtime_t* status)
{
  GripperRealtime_t current;
  GripperResult_t ret;

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }

  if (handle->deadband_percent > 0.0f) {
    ret = Gripper_ReadRealtime(handle, &current);
    if (ret != GRIPPER_OK) {
      return ret;
    }
    if (Gripper_IsPercentInDeadband(handle, percent, current.multi_turn_count) != 0U) {
      if (status != NULL) {
        *status = current;
      }
      handle->last_status = current;
      handle->last_error = GRIPPER_OK;
      return GRIPPER_OK;
    }
  }

  return Gripper_MoveAbsolute(handle, Gripper_PercentToCount(handle, percent), status);
}

//按开合百分比并带限速限流执行位置运动
GripperResult_t Gripper_MoveToPercentWithLimits(GripperHandle_t* handle,
                                                float percent,
                                                float max_speed_rpm,
                                                float max_current_amp,
                                                GripperRealtime_t* status)
{
  GripperRealtime_t current;
  GripperResult_t ret;

  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }

  if (handle->deadband_percent > 0.0f) {
    ret = Gripper_ReadRealtime(handle, &current);
    if (ret != GRIPPER_OK) {
      return ret;
    }
    if (Gripper_IsPercentInDeadband(handle, percent, current.multi_turn_count) != 0U) {
      if (status != NULL) {
        *status = current;
      }
      handle->last_status = current;
      handle->last_error = GRIPPER_OK;
      return GRIPPER_OK;
    }
  }

  return Gripper_MoveAbsoluteWithLimits(handle,
                                        Gripper_PercentToCount(handle, percent),
                                        max_speed_rpm,
                                        max_current_amp,
                                        status);
}

//控制夹爪移动到已配置的张开位置
GripperResult_t Gripper_Open(GripperHandle_t* handle,
                             GripperRealtime_t* status)
{
  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }
  return Gripper_MoveAbsolute(handle, handle->open_position_count, status);
}

//控制夹爪移动到已配置的闭合位置
GripperResult_t Gripper_Close(GripperHandle_t* handle,
                              GripperRealtime_t* status)
{
  if (handle == NULL) {
    return GRIPPER_INVALID_ARG;
  }
  return Gripper_MoveAbsolute(handle, handle->close_position_count, status);
}
