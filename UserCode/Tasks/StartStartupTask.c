#include "StartStartupTask.h"
#include "cmsis_os2.h" 




void StartStartupTask(void *argument){
	for(;;){
//printf("currdata:%f\n",(float)(current_motors[5]));
		osDelay(1000);
	}
}


