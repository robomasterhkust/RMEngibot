#include "gimbal.h"

static RC_Ctl_t* pRC;


static THD_WORKING_AREA(gimbal_control_wa, 512);
static THD_FUNCTION(gimbal_control, p)
{
  (void)p;
  chRegSetThreadName("gimbal controller");

  
  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {

  	int16_t RX_Y2 = (pRC->rc.channel0 - RC_CH_VALUE_OFFSET);
    can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,
        0,  5000  , 0, 0);
  }
}

void gimbal_init(void){
	chThdCreateStatic(gimbal_control_wa, sizeof(gimbal_control_wa),
                          NORMALPRIO, gimbal_control, NULL);
}