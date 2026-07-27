#include "bsp.h"
#include "string.h"
ProtocolStatus_t ft_status;
void bsp_init(void){
	bsp_eeprom_init();
	UpgradeContext_Init(&ctx);
	bsp_time_init();
	bsp_uart_init();
	//是否升级判断
	bsp_boot_init();
}

/**
 * @brief  处理一帧 UART 接收到的消息
 *         - 尝试解析帧
 *         - 成功则进入业务处理
 *         - 失败则返回错误应答帧
 */
void frame_message_process(uint8_t flag) {
    // 1. 调用 ParseFrame() 尝试解析 UART 接收到的原始数据
    //    - uart1_buffer_data: 串口接收缓冲区
    //    - uart1_data_lenth : 当前缓冲区内有效数据长度
    //    - &ft              : 解析后存储到的 Frame_t 结构体
		if(ctx.state==UPG_ERROR){
			//输出错误升级信息
			u1_printf("出现重大问题，请检查设备状态或者设备连接是否可靠\r\n");
			
		}
		//解析新的帧时，需要重置帧状态，避免发生错误
		ctx.frame_status = FRAME_OK;
		if(flag==1){
			 ft_status = ParseFrame(uart1_buffer_data, uart1_data_lenth, &ft);
		}
		else if(flag==3){
			 ft_status = ParseFrame(uart3_buffer_data, uart3_data_lenth, &ft);
		}
		if(ft_status==(ProtocolStatus_t)0xFE){
			 ctx.state = UPG_WAIT_FILE_INFO;
			return ;
		}
    // 2. 判断解析是否成功
    if (ft_status == FRAME_OK) {
        // 解析成功，进入业务处理逻辑
        frame_process(&ft);
				
    }
    else 
		{
				if (flag == 3 && uart3_buffer_data[0] != FRAME_HEADER) {
					return;
				}
				update_error_count(ft_status);
        // 解析失败，打印错误类型（如 Header Error、CRC Error 等）
				if (flag == 3) {
					u1_printf("uart3 parse err=%s len=%u head=0x%02X cmd=0x%02X\r\n",
						ProtocolStatusToStr(ft_status), uart3_data_lenth, uart3_buffer_data[0], uart3_buffer_data[1]);
				} else {
					u1_printf(ProtocolStatusToStr(ft_status));
				}
				if(ft_status<FRAME_ERR_DISCONTINUOUS){
					        // ===== 进行错误应答帧打包 =====
        // 错误应答帧固定前 3 个字节为 0xA5 0xA5 0xA5
        FRAME_ACK_DATA[0] = 0xA5;
        FRAME_ACK_DATA[1] = 0xA5;
        FRAME_ACK_DATA[2] = 0xA5;
        // 第 4 个字节为错误码（枚举值）
        FRAME_ACK_DATA[3] = ft_status;

        // 封装成帧：
        // - 命令码  : CMD_FRAME_ERROR (表示解析错误)
        // - 数据区  : FRAME_ACK_DATA（长度为 4）
        if (PackFrame(FILE_ACK_FRAME, CMD_FRAME_ERROR, FRAME_ACK_DATA, 4)==10) {
            // 打印打包后的应答帧内容（HEX 格式）
           // print_hex_with_tag("错误应答帧: ", FILE_ACK_FRAME, sizeof(FILE_ACK_FRAME));
					
						rs485_uart3_tx(FILE_ACK_FRAME,10);
        }
				
		}		
    }


}

