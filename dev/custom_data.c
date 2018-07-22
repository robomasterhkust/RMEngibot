#include "custom_data.h"
#include "judge.h"
#include "island.h"
#include <stdbool.h>

static Custom_Data_t customData;
static size_t sizeout;
// static projectile_fb_t projectile;
static judge_fb_t* pJudge;



 #define  CUSTOM_DATA_UPDATE_PERIOD      90U // the update frequency is 100ms
 static THD_WORKING_AREA(custom_data_thread_wa, 1024);

 static THD_FUNCTION(custom_data_thread, p)
 {
     Custom_Data_t *d = (Custom_Data_t *) p;
     chRegSetThreadName("Update Custom Data");
     // msg_t rxmsg;
     // systime_t timeout = MS2ST(CUSTOM_DATA_UPDATE_PERIOD);
    while (!chThdShouldTerminateX()) {
	    	d->data1 = (float)(island_getRobotState());
	    	d->data2 = (float)(island_getState()); // edit
	    	d->data3 = (float)(-1.0); // edit
	    	d->lights8 = 0b00000000;  // edit
    	
    	sizeout = judgeDataWrite(d->data1, d->data2, d->data3, d->lights8); 
    	chThdSleepMilliseconds(CUSTOM_DATA_UPDATE_PERIOD);
    }
}


void customData_init(void){

    pJudge = judgeDataGet();


	customData.data1 = 0.0f;
	customData.data2 = 0.0f;
	customData.data3 = 0.0f;
	customData.lights8 = 0;

     chThdCreateStatic(custom_data_thread_wa, sizeof(custom_data_thread_wa),
                   NORMALPRIO + 7,
                   custom_data_thread, &customData);


}