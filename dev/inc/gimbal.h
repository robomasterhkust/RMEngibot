
#ifndef INC_GIMBAL_H_
#define INC_GIMBAL_H_

#include "canBusProcess.h"
#include "chassis.h"


#define GIMBAL_CAN  &CAND1         // Later should be CAND2
#define GIMBAL_CAN_EID  0x200
#define GIMBAL_GEAR_RATIO    36U
#define GIMBAL_MAX_OUTPUT    15384
#define GIMBAL_CONTROL_FREQ 500U

#define GIMBAL_IN_POSITION_COUNT      50U
#define GIMBAL_CONNECTION_ERROR_COUNT 10U

#define ABS(x)     ( ((x) > 0) ? (x) : (-(x)) ) //return abs value of x
typedef enum {
  GIMBAL_UNINIT         = 0,
  GIMBAL_INITING        = 1,
  GIMBAL_STABLE			= 2,
  GIMBAL_ROTATION 		= 4
}gimbal_state_t;


motorPosStruct* gimbal_get(void);
void gimbal_kill(void);
void gimbal_changePos(const float pos_sp);
void gimbal_calibrate(void);
void gimbal_init(void);


#endif