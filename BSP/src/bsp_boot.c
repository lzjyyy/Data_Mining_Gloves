#include "bsp_boot.h"
#include "bsp_eeprom.h"
#include "bsp_status.h"
#include "bsp_uart.h"
#include "bsp_frame.h"
#include "bsp_flash.h"
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef hi2c1;
extern CRC_HandleTypeDef hcrc;
extern TIM_HandleTypeDef htim6;
typedef void (*pFunction)(void);

void bootloader_clear(void){
	HAL_GPIO_DeInit(LED1_GPIO_Port,LED1_Pin);
	HAL_GPIO_DeInit(LED2_GPIO_Port,LED2_Pin);
	//针对串口自动释放资源
	HAL_UART_DeInit(&huart1);
	//针对IIC外设
	HAL_I2C_MspDeInit(&hi2c1);
	//针对时钟
	HAL_TIM_Base_MspDeInit(&htim6);
	//针对CRC
	HAL_CRC_DeInit(&hcrc);
	
	
////	//清除HAL的设计
	HAL_DeInit();
}
void bootToUserAPP(void){
	__disable_irq();  // 禁用所有中断

    // 停止系统滴答
    SysTick->CTRL = 0;
    SysTick->LOAD = 0;
    SysTick->VAL  = 0;

    // 清除 NVIC 所有中断
    for (int i = 0; i < 8; i++) {
        NVIC->ICER[i] = 0xFFFFFFFF;
        NVIC->ICPR[i] = 0xFFFFFFFF;
    }

    // 关闭缓存（STM32H7 必须,打开这个容易导致程序卡死）
////    SCB_DisableICache();
////    SCB_DisableDCache();
		    // 2. 判断是否启用了 Cache，再做清理
    if ((SCB->CCR & SCB_CCR_DC_Msk) != 0)  // D-Cache enabled?
    {
        SCB_CleanInvalidateDCache();
        SCB_DisableDCache();
    }

    if ((SCB->CCR & SCB_CCR_IC_Msk) != 0)  // I-Cache enabled?
    {
        SCB_InvalidateICache();
        SCB_DisableICache();
    }

   // 清除 ITCM 映射（H7 特殊，H723，没有这个功能）
		
	//清除所有的外设配置
    bootloader_clear();
    // 设置 MSP 指针（栈顶地址）
    __set_MSP(*((__IO uint32_t*)APP_ADDRESS));
		//设置中断向量表
		SCB->VTOR = APP_ADDRESS;
    // 跳转到复位地址（即复位中断处理函数）
    uint32_t jump_address = *((__IO uint32_t*)(APP_ADDRESS + 4));
		pFunction jump_to_app = (pFunction)jump_address;
		/* optional short delay */
    for (volatile int i = 0; i < 10000; ++i)   __NOP();
    //进行跳转
    jump_to_app();  // 永远不会返回
}


//升级判断并决定跳转

void bsp_boot_init(){
	if(APP_REAY()&&Boot_CheckUpgradeFlag()){
		u1_printf("now goto the userAPP!\r\n");
		bootToUserAPP();
	}
	else{
		u1_printf("now stay bootloader!\r\n");
		u1_printf("uart3 slave addr: 0x%02X, baud code: %u, baudrate: %lu\r\n",
			bsp_uart_get_slave_addr(),
			bsp_uart_get_uart3_baud_code(),
			bsp_uart_get_uart3_baudrate());
	}

}


/**
 * @brief  检查 Boot 与 App 的准备状态
 *
 * 该函数通过读取 EEPROM 中的两个关键标志位：
 * - EEPROM_APP_FLAG：标记 Boot 是否已经完成第一次初始化
 * - EEPROM_APP_READY：标记 App 是否已经准备好运行
 *
 * 逻辑说明：
 * 1. 如果 boot_flag 不等于 BOOT_READY_VALUE（说明是第一次进入 Boot）：
 *    - 写入 BOOT_READY_VALUE，表示 Boot 已初始化。
 *    - 如果 APP 还没有准备好（app_flag != APP_READY_VALUE），
 *      则将 APP 标志位置为 APP_NOT_READY_VALUE（0xBB）。
 *    - 返回 0，表示暂不跳转到 APP。
 *
 * 2. 如果 Boot 已初始化完成（boot_flag == BOOT_READY_VALUE）：
 *    - 如果 APP 未准备好（app_flag != APP_READY_VALUE），
 *      则返回 0，继续停留在 Boot。
 *
 * 3. 如果 APP 已准备好（app_flag == APP_READY_VALUE），
 *    返回 1，表示可以跳转到 APP。
 *
 * @return uint8_t
 *         - 0：Boot 状态，APP 未准备好，停留在 Boot
 *         - 1：APP 已准备好，可以跳转到 APP
 */
uint8_t APP_REAY(void){
	//EEPROM_WriteByte(EEPROM_APP_READY, APP_NOT_READY_VALUE);
	uint8_t boot_flag = 0;
	uint8_t app_flag = 0;
	
	EEPROM_ReadByte(EEPROM_APP_FLAG, &boot_flag);
	EEPROM_ReadByte(EEPROM_APP_READY, &app_flag);

	if (boot_flag != BOOT_READY_VALUE)
	{
			// 第一次烧写Boot
			EEPROM_WriteByte(EEPROM_APP_FLAG, BOOT_READY_VALUE);
			
			// ?? 如果 APP 还没准备好，才写 0xBB
			if (app_flag != APP_READY_VALUE)
			{
					EEPROM_WriteByte(EEPROM_APP_READY, APP_NOT_READY_VALUE);
			}
			//--------------//这里增加打印标识
			
			
			return 0;
	}

	// Boot 已经初始化完成
	if (app_flag != APP_READY_VALUE)
	{
			// APP 还没准备好，停留在Boot
		
		//--------------------------//这里增加打印标识
			return 0;
	}
	return 1;
}



/**
 * @brief  检查 Boot 是否有升级标志，并进行相应处理
 *
 * 功能流程：
 * 1. 从 EEPROM 读取升级信息（文件长度、已写长度、目标 CRC、升级标志）。
 * 2. 解析各个字段，检查是否有效。
 * 3. 计算 Flash 中实际写入数据的 CRC（软/硬件两种方式）。
 * 4. 如果存在有效升级标志 (0x12345678)：
 *    - 回复 Boot 就绪帧
 *    - 清除 EEPROM 中的升级标志
 *    - 状态机进入 UPG_WAIT_FILE_INFO
 *    - 保持在 Boot 状态等待后续升级数据
 ***
 * @retval 1 跳转到APP
 * @retval 0 bootloder不跳转
 
 * @note  该函数应在 Boot 启动时调用。
 */
uint8_t Boot_CheckUpgradeFlag(void)
{
    uint32_t upgrade_flag = 0;
    uint32_t length_file = 0;
    uint32_t crc_receive = 0;
    uint32_t write_len = 0;

    uint8_t buf[16] = {0};

    // Step 1：从 EEPROM 读取 16 字节升级信息
    HAL_StatusTypeDef eeprom_status = EEPROM_ReadBytes(EEPROM_FILE_LENGTH_ADDR, buf, sizeof(buf));
    if (eeprom_status != HAL_OK)
    {
        u1_printf("? EEPROM 读取升级信息失败\r\n");
        return 0;
    }

    // Step 2：按大端解析 4 个 uint32_t
    length_file =  ((uint32_t)buf[0]  << 24) |
                   ((uint32_t)buf[1]  << 16) |
                   ((uint32_t)buf[2]  << 8)  |
                   ((uint32_t)buf[3]);
    crc_receive =  ((uint32_t)buf[4]  << 24) |
                   ((uint32_t)buf[5]  << 16) |
                   ((uint32_t)buf[6]  << 8)  |
                   ((uint32_t)buf[7]);
    write_len   =  ((uint32_t)buf[8]  << 24) |
                   ((uint32_t)buf[9]  << 16) |
                   ((uint32_t)buf[10] << 8)  |
                   ((uint32_t)buf[11]);
    upgrade_flag=  ((uint32_t)buf[12] << 24) |
                   ((uint32_t)buf[13] << 16) |
                   ((uint32_t)buf[14] << 8)  |
                   ((uint32_t)buf[15]);

    // Step 3：输出调试信息
    u1_printf("读取升级信息:\r\n");
    u1_printf("  length_file   = %lu (0x%08lX)\r\n", length_file, length_file);
    u1_printf("  crc_receive   = 0x%08lX\r\n", crc_receive);
    u1_printf("  write_len     = %lu (0x%08lX)\r\n", write_len, write_len);
    u1_printf("  upgrade_flag  = 0x%08lX\r\n", upgrade_flag);

    // Step 4：基础有效性检查
    if (length_file == 0 || length_file > 0x100000)
    {
        u1_printf("length_file 无效，退出检查\r\n");
        return 0;
    }

    if (write_len == 0 || write_len > length_file||write_len!=length_file)
    {
        u1_printf("write_len 无效，退出检查\r\n");
        return 0;
    }

    // Step 5：计算 Flash 区域的 CRC

    uint32_t calc_hardwarecrc = BSP_HW_CalculateCRC32((uint32_t*)FLASH_START_ADDR, write_len, 0xFFFFFFFF);
    u1_printf("硬件计算 CRC32: 0x%08X\r\n", calc_hardwarecrc);
	  // 检查 CRC 是否匹配
		if (calc_hardwarecrc != crc_receive)
		{
			u1_printf(" CRC 校验不通过，仍然进入升级模式\r\n");
			return 0;
		}
		else
		{
			//	u1_printf("CRC 校验通过\r\n");
		}
    // Step 6：读取并判断升级标志
    if (upgrade_flag == 0x12345678)
    {
        u1_printf(" 检测到有效升级标志\r\n");
        // Boot 回复上位机“准备好升级”
//        uint8_t update_flag[4]     = {0x87, 0x65, 0x43, 0x21};
//        uint8_t ack[10] = {0};

//        PackFrame(ack, CMD_BOOT_READY, update_flag, 4);
//       // HAL_StatusTypeDef eeprom_status2 = EEPROM_WriteBytes(EEPROM_UPGRADEFLAGE_ADDR, not_update_flag, 4);
//        rs485_uart3_tx(ack, sizeof(ack));
        // Step 7：状态机进入等待文件信息状态
        ctx.state = UPG_WAIT_FILE_INFO;
        u1_printf("Boot 停留，等待 APP 文件信息...\r\n");
				return 0;
    }
    else
    {
        u1_printf("未检测到升级标志，正常启动...\r\n");
			return 1;
    }
		return 1;
}


