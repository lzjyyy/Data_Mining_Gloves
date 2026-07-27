#include "UartDbgTask.h"
#include "cmsis_os2.h" 
#include "Rs485RecvTask.h"
#include "data_manager.h"
#include "AT24C256.h"
#include "base_convert.h"
#include "LaunchTask.h"
#include "EncSenseTask.h"


static float current_motors[6]={0};
static float speed_motors[6]={0};

static float Uart_posRef[6]={0.0f};
static float Uart_posMea[6]={0.0f};
static float Uart_spdRef[6]={0.0f};
static float Uart_spdMea[6]={0.0f};


static float position_motors[6]={0};
static float allData[18]={0};
uint8_t test_receive[36]={0};





void StartUartDbgTask(void *argument){
	for(;;){

		DataManager_GetAllMotorSpeedsMea(Uart_spdMea,6);
		DataManager_GetAllMotorSpeedsRef(Uart_spdRef,6);
		DataManager_GetAllMotorPositionsMea(Uart_posMea,6);
		DataManager_GetAllMotorPositionsRef(Uart_posRef,6);
		DataManager_GetAllMotorCurrentsMea(current_motors,6);

		float posEro[MOTOR_COUNT]={0};
		for(uint8_t i = 0; i<MOTOR_COUNT; i++){
			posEro[i] = Uart_posMea[i]-Uart_posRef[i];
		}

		// printf("posref[5] = %.4f, posmea[5] = %.4f\r\n",Uart_posRef[5],Uart_posMea[5]);
						
//  	AT24Cxx_Read_Amount_Byte(0,test_receive,36);		
//		bytes_to_floats(test_receive,allData,18);
		
//printf("position_1:%.4f\n",position_motors[5]);
//printf("position_2:%.4f\n",allData[17]);	
	
//printf("读出的数据为 %d\n",test_receive[2]);
//printf("spdRef_spdMea_posRef_posMea:%.4f,%.4f,%.4f,%.4f\n",Uart_spdRef[1],Uart_spdMea[1],Uart_posRef[1],Uart_posMea[1]);
//printf("position_1:%.4f\n",(float)(position_motors[5]));
//printf("position_2:%.4f\n",(float)(allData[17]));
// printf("currdata: %4f, %4f, %4f, %4f, %4f, %4f\r\n",current_motors[0],current_motors[1],current_motors[2],current_motors[3],current_motors[4],current_motors[5]);
		// printf("spdRef_spdMea_posRef_posMea:%.4f,%.4f,%.4f\n",Uart_spdMea[3],Uart_posRef[3],Uart_posMea[3]);
		

		// printf("uartposmeatest: %.4f, %.4f\r\n",Uart_posMea[2],Uart_posRef[2]);
		// printf("posdata: %4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f\r\n",Uart_posMea[0],Uart_posMea[1],Uart_posMea[4],Uart_posMea[5],Uart_posRef[0],Uart_posRef[1],Uart_posRef[4],Uart_posRef[5]);
		// printf("posdata: %4f, %4f, %4f, %4f, %4f, %4f, %4f, %4f\r\n",Uart_spdMea[2],Uart_spdMea[3],Uart_spdMea[4],Uart_spdMea[5],Uart_spdRef[2],Uart_spdRef[3],Uart_spdRef[4],Uart_spdRef[5]);
		
		// printf("pos: %4f, %4f, %4f, %4f, %4f, %4f\r\n",Uart_posMea[0],Uart_posMea[1],Uart_posMea[2],Uart_posMea[3],Uart_posMea[4],Uart_posMea[5]);
		
		// printf("currdata: %4f, %4f, %4f, %4f, %4f, %4f\r\n",current_motors[0],current_motors[1],current_motors[2],current_motors[3],current_motors[4],current_motors[5]);


		// printf("posEro: %4f, %4f, %4f, %4f, %4f, %4f\r\n",posEro[0],posEro[1],posEro[2],posEro[3],posEro[4],posEro[5]);

		// printf("totol_count: %d, %d, %d, %d, %d, %d\r\n",total_cnt[0],total_cnt[1],total_cnt[2],total_cnt[3],total_cnt[4],total_cnt[5]);

		osDelay(10);
	}
}


