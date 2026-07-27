#include "cmsis_os2.h" 
#include "FreeRTOS.h"
#include "queue.h"

#include "usart.h"
#include "dma.h"

#include "StoreTask.h"
#include "AT24C256.h"
#include "data_manager.h"
#include "base_convert.h"
#include "rs485_handlers.h"

#include <stdint.h>
#include <string.h>

#include <SysCtrlTask.h>

//osEventFlagsId_t StoreEventsHandle;

static float storeCurrDataPre[6]={0.f};     //电流存储数据(Pre)
static float storeSpeedDataPre[6]={0.f};    //速度存储数据
static float storePositionDataPre[6]={0.f}; //位置存储数据
static float storeAllDataPreFloat[18]={0.f};//存储电流、速度、位置数据

static float storeCurrDataNow[6]={0.f};     //电流存储数据(Now)
static float storeSpeedDataNow[6]={0.f};    //速度存储数据
static float storePositionDataNow[6]={0.f}; //位置存储数据
static float storeAllDataNowFloat[18]={0.f};//存储电流、速度、位置数据

static uint8_t storeCurrData2Byte[12]={0};    //电流存储数据(拆分成2个字节存储)
static uint8_t storeSpeedData2Byte[12]={0};   //速度存储数据(拆分成2个字节存储)
static uint8_t storePositionData2Byte[12]={0};//位置存储数据(拆分成2个字节存储)
static uint8_t storeAllDataByte[36]={0};      //所有需要存储的数据转换成字节，拼成一个大的数组

static volatile uint8_t ChangFlag = 0;

#define CurrDataThreshold       (221.2f)   	//用于判断两次数据是否一致的阈值
#define SpeedDataThreshold      (2.2f)
#define PositionDataThreshold   (1.2f)


/**
 * @brief 检查EEPROM中是否存在有效的485配置初始化标识符，若无则写入默认配置
 * 			标识符存储在EEPROM地址69-72处，内容为0x05 0x04 0x03 0x02
 * @note 该函数应在首次启动时调用，以确保EEPROM中有有效的485配置参数
 * @return 无
 */
void Rs485Config_InitFlagCheck(void)
{
	uint8_t initFlag[4]={0};

	HAL_StatusTypeDef result;
	result = AT24Cxx_Read_Amount_Byte(69,initFlag,4);
	while(result != HAL_OK){
		osDelay(200);
		result = AT24Cxx_Read_Amount_Byte(69,initFlag,4);
	} //读取EEPROM中存储的初始化标识符
	printf("%d\r\n", result);
	printf("%x, %x, %x, %x\r\n", initFlag[0], initFlag[1], initFlag[2], initFlag[3]);
	if(result != HAL_OK || initFlag[0]!=0x05 || initFlag[1]!=0x04 || initFlag[2]!=0x03 || initFlag[3]!=0x02)
	{
		//写入默认参数
		g_rs485_cfg.slave_addr = 0xC8;    //默认地址200
		g_rs485_cfg.baudrate   = DEFAULT_SYS_BAUDRATE;  //默认波特率115200
		result = AT24Cxx_Write_Amount_Byte(64,(uint8_t *)&(g_rs485_cfg.slave_addr),1);
		while(result != HAL_OK){
			osDelay(200);
			result = AT24Cxx_Write_Amount_Byte(64,(uint8_t *)&(g_rs485_cfg.slave_addr),1);
		}	//485地址存储在eeprom地址64处
		printf("%d\r\n", result);
		result = AT24Cxx_Write_Amount_Byte(65,(uint8_t *)&(g_rs485_cfg.baudrate),4);
		while(result != HAL_OK){
			osDelay(200);
			result = AT24Cxx_Write_Amount_Byte(65,(uint8_t *)&(g_rs485_cfg.baudrate),4);
		}		//485波特率存储在eeprom地址65处，4字节
		printf("%d\r\n", result);
		//写入初始化标识符
		initFlag[0]=0x05;
		initFlag[1]=0x04;
		initFlag[2]=0x03;
		initFlag[3]=0x02;
		
		result = AT24Cxx_Write_Amount_Byte(69,initFlag,4); 
		while(result != HAL_OK){
			osDelay(200);
			result = AT24Cxx_Write_Amount_Byte(69,initFlag,4); 
		}
		printf("%d\r\n", result);
	}
}


//读取存储在EEPROM中的485通信参数
void StoreTask_Read485ConfigFromEEPROM(void)
{
	HAL_StatusTypeDef result;
	//读取485地址
	result = AT24Cxx_Read_Amount_Byte(64,(uint8_t *)&(g_rs485_cfg.slave_addr),1); //485地址存储在eeprom地址64处
	//读取失败或者地址不合法，设置地址为默认0xC8
	if(result != HAL_OK || g_rs485_cfg.slave_addr < 1 || g_rs485_cfg.slave_addr > 247)
	{
		//默认地址200
		g_rs485_cfg.slave_addr = 0xC8; 
	}
	//读取485波特率
	result = AT24Cxx_Read_Amount_Byte(65,(uint8_t *)&(g_rs485_cfg.baudrate),4);   //485波特率存储在eeprom地址65处，4字节
	//读取失败，设置为默认115200
	if(result != HAL_OK)
	{
		g_rs485_cfg.baudrate = DEFAULT_SYS_BAUDRATE;
	} else if(g_rs485_cfg.baudrate != 9600 &&
			  g_rs485_cfg.baudrate != 19200 &&
			  g_rs485_cfg.baudrate != 38400 &&
			  g_rs485_cfg.baudrate != 57600 &&
			  g_rs485_cfg.baudrate != 115200 &&
			  g_rs485_cfg.baudrate != 230400 &&
			  g_rs485_cfg.baudrate != 460800 &&
			  g_rs485_cfg.baudrate != 921600) {
		//波特率不合法，设置为默认115200
		g_rs485_cfg.baudrate = DEFAULT_SYS_BAUDRATE;
	}
}

/*---------------------------------------------------------------------------------------------------------------------20hz线程处理*/
void StartStoreTask(void *argument)/*掉电存储线程*/
{
	
	TickType_t xLastWakeTime;
	const TickType_t xFrequency = 10;//线程周期，这里表示10个tick执行该线程一次
	xLastWakeTime = xTaskGetTickCount();
	for(;;)
	{
		
		/*① 获取所有需要存储的数据*/
		DataManager_GetAllMotorCurrentsMea(storeCurrDataNow,6);
		DataManager_GetAllMotorSpeedsMea(storeSpeedDataNow,6);
		DataManager_GetAllMotorPositionsMea(storePositionDataNow,6);
		
		/*② 判断所获取的数据存储的必要性（主要判断前后数据帧中是否有变化）*/
		for(uint8_t i=0;i<6;i++)
		{
			if(-CurrDataThreshold<storeCurrDataNow[i]-storeCurrDataPre[i] && storeCurrDataNow[i]-storeCurrDataPre[i]<CurrDataThreshold) ChangFlag=1;
			if(-SpeedDataThreshold<storeSpeedDataNow[i]-storeSpeedDataPre[i] && storeSpeedDataNow[i]-storeSpeedDataPre[i]<SpeedDataThreshold) ChangFlag=1;
			if(-PositionDataThreshold<storePositionDataNow[i]-storePositionDataPre[i] && storePositionDataNow[i]-storePositionDataPre[i]<PositionDataThreshold) ChangFlag=1;
			storeCurrDataPre[i]=storeCurrDataNow[i];
			storeSpeedDataPre[i]=storeSpeedDataNow[i];
			storePositionDataPre[i]=storePositionDataNow[i];

		}
				
		if(ChangFlag==1||ChangFlag==0)                            //存在误差范围外的变化，所以要存储
		{
		/*③ 将需要存储的数据转为两个字节的浮点数*/
    	splice_float_frame(storeAllDataNowFloat,3,
							storeCurrDataNow,(size_t)6,
							storeSpeedDataNow,(size_t)6,
							storePositionDataNow,(size_t)6);			//拼帧
		
		floats_to_bytes(storeAllDataNowFloat,storeAllDataByte,18);//将拼好帧的浮点数转成16位的二进制数，两个字式存储
			
		/*④ 将转换好格式的数据存储到EEPROM中*/					
		AT24Cxx_Write_Amount_Byte(0,storeAllDataByte,36);		
			
		}

		//485通讯存储事件处理
		uint32_t ev = osEventFlagsWait(StoreEventsHandle, EVT_ALL_BITS, 0, 0);
		//成功返回才处理
		if ((int32_t)ev >= 0) {  
			
			if (ev & EVT_485ADDR_BIT) {
				HAL_StatusTypeDef result;
				result = AT24Cxx_Write_Amount_Byte(64, (uint8_t *)&(g_rs485_cfg.slave_addr), 1);
				while(result != HAL_OK){
					osDelay(200);
					result = AT24Cxx_Write_Amount_Byte(64, (uint8_t *)&(g_rs485_cfg.slave_addr), 1);
				} //485地址存储在eeprom地址64处
				osEventFlagsClear(StoreEventsHandle, EVT_485ADDR_BIT);
			}
			if (ev & EVT_485BAUD_BIT) {
				HAL_StatusTypeDef result;
				result = AT24Cxx_Write_Amount_Byte(65, (uint8_t*)&(g_rs485_cfg.baudrate), 4);
				while(result != HAL_OK){
					osDelay(200);
					result = AT24Cxx_Write_Amount_Byte(65, (uint8_t*)&(g_rs485_cfg.baudrate), 4);
				} //485波特率存储在eeprom地址65处，4字节
				osEventFlagsClear(StoreEventsHandle, EVT_485BAUD_BIT);
			}
			if( ev & EVT_OTA_FLAG_BIT) {
				uint8_t ota_flag[4] = {0x12, 0x34, 0x56, 0x78};
				HAL_StatusTypeDef result;
				result = AT24Cxx_Write_Amount_Byte(EEPROM_UPGRADEFLAGE_ADDR, ota_flag, 4); //OTA标志存储在EEPROM_UPGRADEFLAGE_ADDR处，4字节

				while(result != HAL_OK){
					osDelay(200);
					result = AT24Cxx_Write_Amount_Byte(EEPROM_UPGRADEFLAGE_ADDR, ota_flag, 4); //OTA标志存储在EEPROM_UPGRADEFLAGE_ADDR处，4字节

				}
				printf("OTA Flag Stored\r\n");
				osEventFlagsClear(StoreEventsHandle, EVT_OTA_FLAG_BIT);
				ota_write_eeprom_complete_flag = 1; //设置OTA写EEPROM完成标志
				
			}
		}
		
		/*⑤ 线程周期控制*/
		vTaskDelayUntil( &xLastWakeTime, xFrequency );//任务周期100Hz
	}
}

