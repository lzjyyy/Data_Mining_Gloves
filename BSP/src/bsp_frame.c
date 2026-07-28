#include "bsp_frame.h"
#include "string.h"
#include "bsp_eeprom.h"
#include "bsp_flash.h"
#include "bsp_uart.h"
// 升级状态变量
uint32_t file_crc_accumulated = 0;
uint32_t file_length = 0;
uint32_t received_length = 0;
uint8_t frame_status=0;
#define MODBUS_FC_READ_HOLDING_REGISTERS  0x03U
#define MODBUS_REG_SLAVE_ADDR              0x0000U
#define MODBUS_REG_BAUD_CODE               0x0001U

#define MODBUS_EX_ILLEGAL_DATA_ADDRESS     0x02U
#define MODBUS_EX_ILLEGAL_DATA_VALUE       0x03U
#define MODBUS_EX_SLAVE_DEVICE_FAILURE     0x04U
// ======================= 公共变量 =======================
uint8_t error_data_ack[4]={0xA5,0xA5,0x00,0x00};
uint8_t error_frameID_ack[4]={0x00,0x00,0x00,0x00};
// ======================= 公共函数 =======================
// 发送 ACK 帧
void send_ack(uint8_t cmd, const uint8_t *data, uint8_t len, const char *tag) {
    if (PackFrame(FILE_ACK_FRAME, cmd, data, len) ==10) {
        print_hex_with_tag(tag, FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
    }
}
// -------------------- 公共错误应答封装 --------------------
void send_error_ack(uint8_t cmd_code, uint8_t state, uint8_t cmd, const char *tag)
{
		//默认是原始命令，错误代码
		error_data_ack[2]=cmd;error_data_ack[3]=state;
    if (PackFrame(FILE_ACK_FRAME,cmd_code, error_data_ack, sizeof(error_data_ack)) == 10) {
        print_hex_with_tag(tag, FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
			rs485_uart3_tx(FILE_ACK_FRAME,10);
    }
}
void send_frameID_error_ack(uint8_t cmd_code, uint8_t state, uint8_t cmd,uint16_t* frameID, const char *tag)
{
	memcpy(error_frameID_ack,(uint8_t *)&frameID,2);
	error_data_ack[2]=cmd;error_data_ack[3]=state;
	if (PackFrame(FILE_ACK_FRAME,cmd_code, error_frameID_ack, sizeof(error_frameID_ack)) == 10) {
			print_hex_with_tag(tag, FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
			rs485_uart3_tx(FILE_ACK_FRAME,10);
	}
}

void bsp_frame_init(void){
	//
}
static void Build_Modbus_Upgrade_Frame(uint8_t *frame, const uint8_t *payload)
{
	frame[0] = bsp_uart_get_slave_addr();
	frame[1] = 0x10;
	frame[2] = 0x01;
	frame[3] = 0x00;
	frame[4] = 0x00;
	frame[5] = 0x02;
	frame[6] = 0x04;
	memcpy(&frame[7], payload, 4);
	uint16_t crc = Modbus_CRC16(frame, 11);
	frame[11] = (uint8_t)(crc & 0xFF);
	frame[12] = (uint8_t)((crc >> 8) & 0xFF);
}

static uint8_t Is_Modbus_Upgrade_Frame(const uint8_t *frame_buf, uint16_t frame_len, const uint8_t *payload)
{
	if (frame_len < 13) return 0;
	for (uint16_t offset = 0; offset <= frame_len - 13; offset++) {
		const uint8_t *frame = &frame_buf[offset];
		if (frame[0] != bsp_uart_get_slave_addr()) continue;
		if (frame[1] != 0x10 || frame[2] != 0x01 || frame[3] != 0x00 ||
			frame[4] != 0x00 || frame[5] != 0x02 || frame[6] != 0x04) {
			continue;
		}
		if (memcmp(&frame[7], payload, 4) != 0) continue;
		uint16_t crc_calc = Modbus_CRC16(frame, 11);
		uint16_t crc_recv = frame[11] | (frame[12] << 8);
		if (crc_calc == crc_recv) return 1;
	}
	return 0;
}
// 校验码低位在前，高位在后 (Little Endian CRC16)
/*
—— 应答帧或错误帧格式说明 ——————————————————————————————
字段       	| 说明
-----------	|-----------------------------------------------------
帧头       	| 固定帧头，标识帧开始， 0xAA
错误指令   	| 应答码/错误命令码，区分错误类型，如 0xF9 表示错误帧指令， 0xFA 表示错误FLASH指令
长度       	| 数据区长度，固定为 0x04（4 字节数据）
保留（帧号）	| 保留字段，或用作帧号，12字节
原始命令   	| 原始帧命令码，指示出错的命令
错误代码   	| 具体错误码，范围 0x00~0x1F，表示不同错误类型
CRC16校验  	| CRC16 校验码，低字节在前，高字节在后（2 字节）
帧尾       	| 固定帧头，标识帧开始， 0x55

帧格式示例（十六进制）：
| AA | F9 | 04 | 01 | 20 | 05 | 12 34 |  55
  ^    ^    ^    ^    ^    ^    ^        ^
  |    |    |    |    |    |    |        +-- 帧尾 0X55
  |    |    |    |    |    |    +-------- CRC16 校验码 (0x3412 表示低字节 0x12，高字节 0x34)
  |    |    |    |    |    +------------- 错误代码 0x05
  |    |    |    |    +----------------- 原始命令码 0x20/保留
  |    |    |    +---------------------- 保留/帧号 0x01
  |    |    +--------------------------- 长度 0x04
  |    +------------------------------- 错误指令 0xF9,帧错误
  +------------------------------------ 帧头 0xAA
*/

//回应帧数据包
static uint8_t Is_Modbus_Read_Comm_Config_Frame(const uint8_t *frame_buf,
                                                  uint16_t frame_len,
                                                  Frame_t *out_frame)
{
    uint16_t crc_calc;
    uint16_t crc_recv;

    if (frame_len != 8U || frame_buf[0] != bsp_uart_get_slave_addr() ||
        frame_buf[1] != MODBUS_FC_READ_HOLDING_REGISTERS) {
        return 0;
    }

    crc_calc = Modbus_CRC16(frame_buf, 6);
    crc_recv = (uint16_t)frame_buf[6] | ((uint16_t)frame_buf[7] << 8);
    if (crc_calc != crc_recv) {
        return 0;
    }

    out_frame->cmd = frame_buf[1];
    out_frame->len = 4;
    memcpy(out_frame->data, &frame_buf[2], out_frame->len);
    return 1;
}

static uint8_t Modbus_BaudrateToCode(uint32_t baudrate, uint16_t *baud_code)
{
    switch (baudrate) {
        case 9600U:   *baud_code = BAUD_CODE_9600; return 1;
        case 19200U:  *baud_code = BAUD_CODE_19200; return 1;
        case 38400U:  *baud_code = BAUD_CODE_38400; return 1;
        case 57600U:  *baud_code = BAUD_CODE_57600; return 1;
        case 115200U: *baud_code = BAUD_CODE_115200; return 1;
        case 230400U: *baud_code = BAUD_CODE_230400; return 1;
        case 460800U: *baud_code = BAUD_CODE_460800; return 1;
        case 921600U: *baud_code = BAUD_CODE_921600; return 1;
        default: return 0;
    }
}

static uint8_t Modbus_LegacyBaudCodeToCurrentCode(uint8_t legacy_baud_code,
                                                   uint16_t *baud_code)
{
    switch (legacy_baud_code) {
        case 0: *baud_code = BAUD_CODE_9600; return 1;
        case 1: *baud_code = BAUD_CODE_19200; return 1;
        case 2: *baud_code = BAUD_CODE_38400; return 1;
        case 3: *baud_code = BAUD_CODE_57600; return 1;
        case 4: *baud_code = BAUD_CODE_115200; return 1;
        case 5: *baud_code = BAUD_CODE_230400; return 1;
        case 6: *baud_code = BAUD_CODE_460800; return 1;
        case 7: *baud_code = BAUD_CODE_921600; return 1;
        default: return 0;
    }
}
static uint8_t Modbus_ReadCommRegister(uint16_t reg_addr, uint16_t *value)
{
    uint8_t slave_addr;
    uint8_t legacy_baud_code;
    uint32_t baudrate = 0;

    if (reg_addr == MODBUS_REG_SLAVE_ADDR) {
        if (EEPROM_ReadByte(EEPROM_SLAVE_ADDR, &slave_addr) != HAL_OK ||
            slave_addr == 0x00U || slave_addr == 0xFFU) {
            return 0;
        }
        *value = slave_addr;
        return 1;
    }

    if (reg_addr == MODBUS_REG_BAUD_CODE) {
        if (EEPROM_ReadBytes(EEPROM_SLAVE_BAUD, (uint8_t *)&baudrate,
                             sizeof(baudrate)) != HAL_OK) {
            return 0;
        }
        if (Modbus_BaudrateToCode(baudrate, value)) {
            return 1;
        }
        if (EEPROM_ReadByte(EEPROM_SLAVE_BAUD, &legacy_baud_code) == HAL_OK) {
            return Modbus_LegacyBaudCodeToCurrentCode(legacy_baud_code, value);
        }
    }

    return 0;
}

static void Modbus_SendException(uint8_t exception_code)
{
    uint8_t response[5];
    uint16_t crc;

    response[0] = bsp_uart_get_slave_addr();
    response[1] = MODBUS_FC_READ_HOLDING_REGISTERS | 0x80U;
    response[2] = exception_code;
    crc = Modbus_CRC16(response, 3);
    response[3] = (uint8_t)(crc & 0xFFU);
    response[4] = (uint8_t)(crc >> 8);
    rs485_uart3_tx(response, sizeof(response));
}

void Modbus_ProcessReadCommConfig(const Frame_t *frame)
{
    uint16_t start_addr;
    uint16_t quantity;
    uint16_t value;
    uint16_t crc;
    uint16_t i;
    uint8_t response[9];
    uint8_t response_len;

    if (frame == NULL || frame->cmd != MODBUS_FC_READ_HOLDING_REGISTERS ||
        frame->len != 4U) {
        return;
    }

    start_addr = ((uint16_t)frame->data[0] << 8) | frame->data[1];
    quantity = ((uint16_t)frame->data[2] << 8) | frame->data[3];
    if (quantity == 0U || quantity > 2U) {
        Modbus_SendException(MODBUS_EX_ILLEGAL_DATA_VALUE);
        return;
    }
    if (start_addr > MODBUS_REG_BAUD_CODE ||
        (uint32_t)start_addr + quantity > 2U) {
        Modbus_SendException(MODBUS_EX_ILLEGAL_DATA_ADDRESS);
        return;
    }

    response[0] = bsp_uart_get_slave_addr();
    response[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    response[2] = (uint8_t)(quantity * 2U);
    for (i = 0; i < quantity; i++) {
        if (!Modbus_ReadCommRegister(start_addr + i, &value)) {
            Modbus_SendException(MODBUS_EX_SLAVE_DEVICE_FAILURE);
            return;
        }
        response[3U + i * 2U] = (uint8_t)(value >> 8);
        response[4U + i * 2U] = (uint8_t)value;
    }

    response_len = (uint8_t)(3U + quantity * 2U);
    crc = Modbus_CRC16(response, response_len);
    response[response_len++] = (uint8_t)(crc & 0xFFU);
    response[response_len++] = (uint8_t)(crc >> 8);
    rs485_uart3_tx(response, response_len);
}
uint8_t FILE_ACK_FRAME[10]  ={0x0FC,0x0E1,0x00,0x04,0xA5,0xA5,0x00,0x00,0x0a5,0x059};
uint8_t FRAME_ACK_DATA[4]   ={0x00,0x00,0x00,0x00};
Frame_t ft;

/**
 * @brief  计算 MODBUS CRC16 校验码（低字节在前，高字节在后）
 * @param  data    待计算数据指针
 * @param  length  数据长度（单位：字节）
 * @retval 计算得到的 CRC16 值（Little Endian 格式，低字节在前）
 * @note   使用标准 MODBUS 多项式 0xA001，初始值为 0xFFFF
 */
uint16_t Modbus_CRC16(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;

    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;  // 0xA001 是 MODBUS CRC16 的多项式
            else
                crc >>= 1;
        }
    }

    return crc;  // 返回的是低字节在前，高字节在后（Little Endian 格式）
}

/**
 * @brief  将一帧内容解析并封装成 Frame_t 结构体
 * @param  frame_buf   输入的帧数据缓存指针
 * @param  frame_len   输入的帧数据长度（单位：字节）
 * @param  out_frame   输出的已解析帧结构体指针
 * @retval FrameParseStatus_t
 *         - FRAME_OK          		：解析成功
 *         - FRAME_ERR_MIN_LEN  	：帧长度小于最小值
 *         - FRAME_ERR_MAX_LEN  	：帧长度超过最大值
 *         - FRAME_ERR_HEADER   	：帧头错误
 *         - FRAME_ERR_ADDR     	：地址错误
 *         - FRAME_ERR_LEN     		：长度字段不匹配
 *         - FRAME_ERR_CRC      	：CRC16 校验失败
 *         - FRAME_ERR_CRC32    	：CRC32 校验失败（文件数据）
 * @note
 *   1. 帧格式： [Header][Addr][Cmd][Len]{[帧编号(2B,仅数据帧)][Payload...][CRC32(4B,仅数据帧)]len个数据长度}[CRC16(2B)]
 *   2. 普通命令帧只做 Modbus CRC16 校验；数据帧（CMD_DATA_FRAME）额外做 CRC32 校验。
 *   3. CRC32 校验同时支持软件计算和硬件计算（需 STM32 CRC 外设支持）。
 */
ProtocolStatus_t ParseFrame(const uint8_t* frame_buf,
                             uint16_t frame_len, Frame_t* out_frame) 
{
	uint8_t boot_app_payload[] = {0x12, 0x34, 0x56, 0x78};
	uint8_t boot_boot_payload[] = {0x87, 0x65, 0x43, 0x21};
	uint8_t boot_app_ck[13];
	uint8_t boot_boot_ck[13];
	if (Is_Modbus_Read_Comm_Config_Frame(frame_buf, frame_len, out_frame)) {
		return FRAME_MODBUS_NEED_RESPONSE;
	}
	Build_Modbus_Upgrade_Frame(boot_app_ck, boot_app_payload);
	Build_Modbus_Upgrade_Frame(boot_boot_ck, boot_boot_payload);
	if(Is_Modbus_Upgrade_Frame(frame_buf, frame_len, boot_app_payload)){
		rs485_uart3_tx(boot_app_ck,13);
		HAL_Delay(10);
		rs485_uart3_tx(boot_boot_ck,13);
		u1_printf("收到升级标志，开始升级");
		return (ProtocolStatus_t)0xfE;
	}
//	print_hex_with_tag("传入的指令是：",frame_buf,frame_len);
    if (frame_len < FRAME_MIN_DATA)  return FRAME_ERR_MIN_LEN;
    if (frame_len > MAX_FRAME_SIZE)  return FRAME_ERR_MAX_LEN;
    if (frame_buf[0] != FRAME_HEADER) return FRAME_ERR_HEADER;

    uint8_t cmd  = frame_buf[1];
    uint8_t len  = frame_buf[2];
	if (ctx.state == UPG_WRITE_DATA && cmd == CMD_DATA_FRAME && ctx.last_frame_id < 3) {
		u1_printf("rx data raw len=%u data_len=%u\r\n", frame_len, len);
	}
    if ((cmd == CMD_DATA_FRAME && len > 134) || frame_len != (6 + len)) 
        return FRAME_ERR_LEN;
    
    if (frame_buf[frame_len - 1] != FRAME_TAIL) 
        return FRAME_ERR_TAIL;

    uint16_t crc_calc = Modbus_CRC16(&frame_buf[0], 3 + len);
    uint16_t crc_recv = frame_buf[3 + len] | (frame_buf[4 + len] << 8);
    if (crc_calc != crc_recv) return FRAME_ERR_CRC;

    if (cmd == CMD_DATA_FRAME) {
        /* ===== 先帧编号检查 ===== */
        uint16_t frame_id = (frame_buf[3] << 8) | frame_buf[4];
        uint8_t  error_frameID_ack[4];

        if (frame_id > ctx.max_frame_id) {
            ctx.frame_status = FRAME_ERR_OUT_OF_RANGE;
        } 
        else if (frame_id <= ctx.last_frame_id) {
            ctx.frame_status = FRAME_ERR_DUPLICATE;
        } 
        else if (frame_id != ctx.last_frame_id + 1) {
            ctx.frame_status = FRAME_ERR_DISCONTINUOUS;
        } 
        else {
            ctx.frame_status = FRAME_OK;
        }

        if (ctx.frame_status != FRAME_OK) {
					u1_printf("接收到的帧编号：%d\r\n",frame_id);
            update_error_count(ctx.frame_status);
            error_frameID_ack[0] = (uint8_t)(ctx.last_frame_id >> 8);
            error_frameID_ack[1] = ctx.last_frame_id & 0xff;
            error_frameID_ack[2] = cmd;
            error_frameID_ack[3] = ctx.frame_status;
            if (PackFrame(FILE_ACK_FRAME, CMD_FRAMEID_ERROR, error_frameID_ack, 4) == 10) {
                switch (ctx.frame_status) {
                    case FRAME_ERR_OUT_OF_RANGE:
                        print_hex_with_tag("帧编号超过最大范围", FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
                        break;
                    case FRAME_ERR_DUPLICATE:
                        print_hex_with_tag("帧编号重复", FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
                        break;
                    case FRAME_ERR_DISCONTINUOUS:
                        print_hex_with_tag("帧编号不连续", FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
                        break;
										default:break;
                }
                rs485_uart3_tx(FILE_ACK_FRAME, 10);
            }
            return ctx.frame_status; // ? 帧编号错误直接返回
        }

        /* ===== 再 CRC32 累加校验 ===== */
        uint32_t receive_crc = ((uint32_t)frame_buf[len - 1] << 24)
                             | ((uint32_t)frame_buf[len] << 16)
                             | ((uint32_t)frame_buf[len + 1] << 8)
                             | ((uint32_t)frame_buf[len + 2]);
        //u1_printf("接收到的CRC32的值是：%08x\r\n", receive_crc);

        uint32_t calc_crc = BSP_HW_CalculateCRC32((uint32_t *)&frame_buf[5], len - 6, ctx.file_crc); 
       // u1_printf("计算得到的CRC32的值是：%08x\r\n", calc_crc);

        if (calc_crc != receive_crc) {
            u1_printf("CRC32校验失败：%s recv=0x%08X calc=0x%08X\r\n", ProtocolStatusToStr(FRAME_ERR_CRC32), receive_crc, calc_crc);
            return FRAME_ERR_CRC32;
        } else {
            ctx.pre_crc = receive_crc;
            //ctx.last_frame_id = frame_id; // ? 成功才更新 last_frame_id
        }
    }
		out_frame->cmd=cmd;
		memcpy(out_frame->data,&frame_buf[3],len);
		out_frame->len=len;
    return FRAME_OK;
}


// 封装数据帧，返回帧长度，失败返回0
/**
 * @brief  封装 Modbus CRC16 格式的回应帧
 * @param  out_frame  输出帧缓冲区指针（调用者需保证空间足够）
 * @param  addr       设备地址
 * @param  cmd        命令码
 * @param  data       数据区指针
 * @param  len        数据区长度（字节数，不能超过 FRAME_MAX_DATA）
 * @retval uint16_t   成功：帧总长度；失败：0
 * @note
 *   帧格式：
 *     [Header][Addr][Cmd][Len][Payload...][CRC16_L][CRC16_H]
 *   CRC16 计算规则：
 *     - 算法：Modbus CRC16（多项式 0xA001，初始值 0xFFFF）
 *     - 计算范围：从 Header 开始，连续 (4 + len) 个字节
 *   返回的帧总长度 = len + 6
 */
uint16_t PackFrame(uint8_t* out_frame,  uint8_t cmd, const uint8_t* data, uint8_t len) {
    if (len > FRAME_MAX_DATA) return 0;

    out_frame[0] = FRAME_HEADER;
    out_frame[1] = cmd;
    out_frame[2] = len;

    memcpy(&out_frame[3], data, len);

    uint16_t crc = Modbus_CRC16(&out_frame[0], 3 + len);  // 从帧头开始算CRC
    out_frame[3 + len] = (uint8_t)(crc & 0xFF);           // CRC低字节
    out_frame[4 + len] = (uint8_t)((crc >> 8) & 0xFF);    // CRC高字节
		out_frame[5+len]=FRAME_TAIL;
    return 6 + len;
}
/**
 * @brief  OTA 协议帧处理函数
 * @param  frame  指向已解析完成的协议帧结构体指针（Frame_t）
 * @retval None
 * @note
 *  根据接收到的地址和命令类型执行不同的 OTA 相关操作：
 *   - CMD_UPGRADE_REQUEST：响应升级请求
 *   - CMD_FILE_INFO：解析文件总长度、擦除 Flash、返回确认
 *   - CMD_DATA_FRAME：接收数据帧、写入 Flash、回传确认
 *   - CMD_CRC_VERIFY：接收并校验 CRC32，返回校验结果
 *
 *  注意事项：
 *   1. 本函数只处理地址为 CMD_ADDR_BOOT 的帧
 *   2. Flash 擦写和 CRC 计算过程需与上层逻辑配合
 *   3. 回应帧使用 PackFrame 打包，CRC 为 Modbus CRC16
 *   4. 函数内部直接调用 print_hex_with_tag 进行调试输出
 */
void frame_process(Frame_t *frame){
	//首先读取地址和命令
		switch (frame->cmd)
    { 
			 // ----------------- 升级请求（握手） -----------------
			case CMD_UPGRADE_REQUEST:
			{	
				frame_update_request_ack(frame);
				break;
			}
			// ----------------- 文件信息（总长度、元信息） -----------------
      case CMD_FILE_INFO:
      {
            frame_FILE_INFO_ack(frame);
            break;
        }
				// ----------------- 数据帧（分片写入） -----------------
        case CMD_DATA_FRAME:
        {
          frame_data_ack(frame);
          break;
        }
				// 5) 计算该 chunk 的 CRC（优先硬件，如果有） —— 注意 BSP_CalculateCRC32 的第三个参数为初始值或之前的 CRC
        //    这里我们单块验证：传入初值 0xFFFFFFFF 来计算单块 CRC（与主机发送的 CRC 约定一致）
        case CMD_CRC_VERIFY:
        {
            
						frame_VERIFY_CRC32_ack(frame);
            break;
        }
				case CMD_DEBUG_SHOW:
				{
					if(frame->data[0]==0x12&&frame->data[1]==0x34&&
						frame->data[2]==0x56&&frame->data[3]==0x78){
							PrintUpgradeContext(&ctx);
						}
					break;
				}

        default:
            // 未知命令
						 send_error_ack(CMD_FRAME_ERROR, ctx.state, frame->cmd, "Unknown CMD");
            break;
    }
}



//保留，后面应该不用了
void Append_CRC16_To_Frame(uint8_t *frame)
{
    uint8_t data_len = frame[2]; // 假设第3个字节表示数据长度
    uint16_t crc_len = 3+ data_len; // 从头开始，到数据尾部
    uint16_t crc = Modbus_CRC16(frame, crc_len);

    // 写入CRC：低位在前，高位在后（Modbus格式）
    frame[crc_len-1]     = crc & 0xFF;       // CRC低位
    frame[crc_len ] = (crc >> 8) & 0xFF; // CRC高位
}

// -------------------- 错误统计处理 --------------------
void update_error_count(ProtocolStatus_t error_code) {
    if (error_code > FRAME_OK && error_code <= FRAME_ERR_OUT_OF_RANGE) {
        // 帧错误，增加1次
        ctx.error_times += 1;
    } else if (error_code > FLASH_OK && error_code <= FLASH_ERASE_ERR_PROGRAM) {
        // Flash错误，增加6次
        ctx.error_times += 6;
    } else {
        // 其它未知错误，也增加1次
        ctx.error_times += 1;
    }

    if (ctx.error_times > 10) {
        // 超过阈值，触发异常处理
        ctx.state=UPG_ERROR;
        //ctx.error_times = 0;  // 重置计数
    }
}


// -------------------- OTA 升级函数实现 --------------------

// 接收升级请求
void frame_update_request_ack(Frame_t *frame)
{
    if (ctx.state == UPG_IDLE) {
			//代替APP程序发送
			uint8_t APP_ack[]={0xAA, 0xF2, 0x04, 0x12, 0x34, 0x56, 0x78, 0x8E, 0xEC, 0x55};
			//BOOTLOADER程序发送
			rs485_uart3_tx(APP_ack,10);
      uint8_t buf[4] = { 0x87, 0x65, 0x43, 0x21 };
      if (PackFrame(FILE_ACK_FRAME, CMD_BOOT_READY, buf, sizeof(buf)) == 10) {
         print_hex_with_tag("have receive OTA request:", FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
				rs485_uart3_tx(FILE_ACK_FRAME,10);
      }
        ctx.state = UPG_WAIT_FILE_INFO;
    } 
		else 
		{
       send_error_ack(CMD_STATUS_ERROR, ctx.state, frame->cmd, "UPG_IDLE status ERROR:");
    }
}

// 接收文件信息
void frame_FILE_INFO_ack(Frame_t *frame)
{
    if (ctx.state == UPG_WAIT_FILE_INFO) {
        file_length = (frame->data[0] << 24) | (frame->data[1] << 16) |
                      (frame->data[2] << 8)  | frame->data[3];
		
        // 文件大小安全检查
        if (file_length > 1024 * 896) { // APP 程序超过最大长度
					ctx.frame_status=CMD_FILE_SIZE_ERROR;
            send_error_ack(CMD_FRAME_ERROR, ctx.frame_status, frame->cmd, "File length exceeds max size!");
          update_error_count(CMD_FILE_SIZE_ERROR);
					return;
        }
				//写入最大帧编号
				ctx.max_frame_id=(uint16_t)((file_length+127)/128);
				ctx.pre_crc=0xffffffff;
				ctx.file_crc=ctx.pre_crc;
				//写入文件长度
				ctx.file_length=file_length;
        // 擦除 APP 区域
        ctx.flash_status= (ProtocolStatus_t)Flash_EraseSectors1To7();
        if (ctx.flash_status != FLASH_OK) {
						update_error_count(ctx.flash_status);
            send_error_ack(CMD_FLASH_ERROR, frame->cmd, ctx.flash_status, "Flash erase failed:");
						Stop_UpgradeAndReset();
					return;
        }
				else 
				{
						ctx.state = UPG_WRITE_DATA;
						// 写 EEPROM 放在 ACK 前，避免上位机收到 ACK 后立刻发数据时 Boot 还没准备好
						if(EEPROM_WriteUint32_BigEndian(EEPROM_FILE_LENGTH_ADDR,ctx.file_length)==HAL_OK){
							u1_printf("have write flie_length success!\r\n");
						}
						else{
							u1_printf("!!have write flie_length failed!\r\n");
						}
            // 回传确认
            uint8_t buf[4] = { frame->data[0], frame->data[1], frame->data[2], frame->data[3] };
            if (PackFrame(FILE_ACK_FRAME, frame->cmd, buf, sizeof(buf)) > 4) {
                print_hex_with_tag("接收到的文件长度指令：", FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
							rs485_uart3_tx(FILE_ACK_FRAME,10);
            }
        }
    } 
		else
		{
       send_error_ack(CMD_STATUS_ERROR, ctx.state, frame->cmd, "UPG_WAIT_FILE_INFO status ERROR:");
    }
}

// 接收文件数据
void frame_data_ack(Frame_t *frame)
{
    if (ctx.state == UPG_WRITE_DATA) {
        uint16_t frame_id = (frame->data[0] << 8) | frame->data[1];
        uint8_t *payload = &frame->data[2];
        uint16_t payload_len = frame->len - 6;
        // 写入 Flash
				 ctx.flash_status = (ProtocolStatus_t)Flash_Write128BytesWithCheck(
														FLASH_START_ADDR+ctx.received_len,payload,payload_len);
        if (ctx.flash_status != FLASH_OK) {
					//擦除失败，就重传
						update_error_count(ctx.frame_status);
            send_error_ack(CMD_FLASH_ERROR, frame->cmd, ctx.flash_status, "Flash write failed:");
						Stop_UpgradeAndReset();
					return ;
        } 
				else 
				{
            uint8_t buf[4] = { 0xA5, 0xA5, frame->data[0], frame->data[1] };
            if (PackFrame(FILE_ACK_FRAME,  frame->cmd, buf, sizeof(buf))==10) {
              // print_hex_with_tag("接收到的文件数据帧：", FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
							ctx.last_frame_id = frame_id;
							ctx.file_crc=ctx.pre_crc;//更新crc
							ctx.received_len += payload_len;//这个是实际的写入字节
							if (frame_id <= 3 || (frame_id % 50) == 0 || frame_id == ctx.max_frame_id) {
								u1_printf("data frame ok id=%u/%u payload=%u received=%lu\r\n", frame_id, ctx.max_frame_id, payload_len, ctx.received_len);
							}
							if(frame_id==ctx.max_frame_id&& ctx.received_len>=ctx.file_length)
							{
								ctx.state = UPG_VERIFY_CRC; // TODO: 判断是否最后一帧
							}
								rs485_uart3_tx(FILE_ACK_FRAME,10);
            }
        }
        
    } 
		else {
        send_error_ack(CMD_STATUS_ERROR, ctx.state, frame->cmd, "UPG_WRITE_DATA status ERROR:");
    }
}

// 接收 CRC 校验
void frame_VERIFY_CRC32_ack(Frame_t *frame)
{
		PrintUpgradeContext(&ctx);
    if (ctx.state == UPG_VERIFY_CRC) {
        // uint32_t received_crc = (frame->data[0] << 24) | (frame->data[1] << 16) |
        //                         (frame->data[2] << 8)  | frame->data[3];
        uint8_t buf[4] = { 0x12, 0x34, 0x56, 0x78 };
				//需要将写入的文件数据进行CRC32检验
				//起始地址，APP_ADDRESS,长度：ctx.file_length,使用硬件完成
				u1_printf("接收到的文件的CRC32是0x%08X\r\n",ctx.pre_crc );
				uint32_t calc_crc= BSP_HW_CalculateCRC32((uint32_t*)FLASH_START_ADDR,ctx.received_len,0xffffffff);
				u1_printf("MCU硬件计算得到的CRC32是%08x\r\n",calc_crc );
				if(ctx.file_crc!=calc_crc){
					ctx.flash_status=FLASH_ERR_CRC;
					send_error_ack(CMD_FLASH_ERROR, frame->cmd, ctx.flash_status, "Flash CRC32 check Error");
					update_error_count(ctx.flash_status);
					Stop_UpgradeAndReset();
					return;
				}
				
        if (PackFrame(FILE_ACK_FRAME,  frame->cmd, buf, sizeof(buf)) > 4) {
            print_hex_with_tag("have receive OTA VERIFY_CRC :", FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
						rs485_uart3_tx(FILE_ACK_FRAME,10);
        }
        ctx.state = UPG_DONE;
				//写入文件写入长度
				if(EEPROM_WriteUint32_BigEndian(EEPROM_WRITTEN_LEN_ADDR,ctx.received_len)==HAL_OK){
					u1_printf("have write flie_write_length success!\r\n");
				}
				else{
					u1_printf("!!have write flie_write_length failed!\r\n");
				}
				//写入CRC
				if(EEPROM_WriteUint32_BigEndian(EEPROM_CRC32_ADDR,ctx.file_crc)==HAL_OK){
					u1_printf("have write file_crc32 success!\r\n");
				}
				else{
					u1_printf("!!have write file_crc32 failed!\r\n");
				}
				//修改升级标志
				uint8_t up_flag_not[4]={0xFF, 0xFF, 0xFF, 0xFF};
				if(EEPROM_WriteBytes(EEPROM_UPGRADEFLAGE_ADDR,up_flag_not,4)==HAL_OK){
					u1_printf("have write up_flag_not success!\r\n");
				}
				else{
					u1_printf("!!have write up_flag_not failed!\r\n");
				}
				//修改APP标志，确认APP准备就绪
				if(EEPROM_WriteByte(EEPROM_APP_READY, APP_READY_VALUE)==HAL_OK){
					u1_printf("have write APP_READY success!\r\n");
				}
				else{
					u1_printf("!!have write APP_READY failed!\r\n");
				}

				//进行相关的复位操作
				__disable_irq();       // 可选：禁止中断，确保复位顺利
				NVIC_SystemReset();    // 触发软件复位
				while(1);              //永远不会执行
				
    } 
		else 
		{
       send_error_ack(CMD_STATUS_ERROR, ctx.state, frame->cmd, "UPG_VERIFY_CRC status ERROR:");
    }
}


/**
 * @brief  停止当前固件升级并重置所有升级状态
 *
 * 当 Bootloader 检测到重大升级异常时（例如 Flash 写入失败、CRC 校验不通过、
 * 上位机主动下发停止指令、通信中断等），调用本函数可以：
 *
 * 1. 通知上位机升级中止（发送 CMD_STOP_UPGRADE 命令帧）。
 * 2. 重置 EEPROM 升级相关信息：
 *    - 将升级标志重新置为初始值（0x12345656），表示系统处于可重新升级状态；
 *    - 清空已写入长度（EEPROM_WRITTEN_LEN_ADDR）；
 *    - 清空目标文件长度（EEPROM_FILE_LENGTH_ADDR）；
 *    - 清空目标 CRC 校验值（EEPROM_CRC32_ADDR）。
 * 3. 将 Bootloader 升级状态机恢复到 UPG_IDLE 状态。
 * 4. 将APP状态设置未准备好
 *
 * @note 此函数不会触发复位或跳转，仅清除升级数据和标志。
 *       重新升级时，上位机需要再次下发 CMD_FILE_INFO 并重传固件。
 */
void Stop_UpgradeAndReset(void)
{
    uint8_t ack[10] = {0};
    uint8_t stop_flag[4] = {0xFF, 0xFF, 0xFF, 0xFF};  // 用于 STOP_UPGRADE 响应
    uint8_t up_flag[4]   = {0x12, 0x34, 0x56, 0x78};  // EEPROM 初始升级标志

    // Step 1: 发送 CMD_STOP_UPGRADE 帧给上位机，通知升级异常
    PackFrame(ack, CMD_STOP_UPGRADE, stop_flag, sizeof(stop_flag));
    rs485_uart3_tx(ack, sizeof(ack));
    u1_printf("重大升级异常，发送 STOP_UPGRADE 指令，等待重新上传...\r\n");
			
    // Step 2: 重置 EEPROM 升级信息
    EEPROM_WriteBytes(EEPROM_UPGRADEFLAGE_ADDR, up_flag, sizeof(up_flag));   // 升级标志回到初始值
    EEPROM_WriteUint32_BigEndian(EEPROM_WRITTEN_LEN_ADDR, 0);                // 已写长度清零
    EEPROM_WriteUint32_BigEndian(EEPROM_FILE_LENGTH_ADDR, 0);                // 文件长度清零
    EEPROM_WriteUint32_BigEndian(EEPROM_CRC32_ADDR, 0);                      // 目标 CRC 清零

    // Step 3: 状态机恢复空闲，准备新一轮升级
    ctx.state = UPG_IDLE;

    // Step 4: 设置APP程序未准备
		EEPROM_WriteByte(EEPROM_APP_READY, APP_NOT_READY_VALUE);
		
}


