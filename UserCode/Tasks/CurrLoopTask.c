#include "CurrLoopTask.h"
#include "cmsis_os2.h" 
#include "tim.h"
#include "data_manager.h"
	static float posMea[6]={0};


void StartCurrLoopTask(void *argument){

	for(;;)
	{
//		DataManager_GetAllMotorPositionsMea(posMea,6);
//		for(uint8_t i=0;i<6;i++)
//		{
//		//	printf("posMea[%d]:%.4f\n",i,posMea[i]);
//		}
		
//printf("is ok");
	osDelay(100);
	}

}
