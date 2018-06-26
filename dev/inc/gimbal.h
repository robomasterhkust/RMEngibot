
#ifndef INC_GIMBAL_H_
#define INC_GIMBAL_H_

#include "chassis.h"

#define GIMBAL_CAN          &CAND1         // Later should be CAND2
#define GIMBAL_CAN_EID       0x1ff
#define GIMBAL_GEAR_RATIO      36U
#define GIMBAL_MAX_OUTPUT    15384
#define GIMBAL_CONTROL_FREQ  1000U

#define GIMBAL_IN_POSITION_COUNT      50U
#define GIMBAL_CONNECTION_ERROR_COUNT 10U

#define GIMBAL_MAX_SPEED  2.0f

#define ABS(x)     ( ((x) > 0) ? (x) : (-(x)) ) //return abs value of x
typedef enum {
  GIMBAL_UNINIT             = 0,
  GIMBAL_INITING            = 1,
  GIMBAL_STABLE			        = 2,
  GIMBAL_SCREEN   		      = 3,
  GIMBAL_SCREEN_LOCK   		  = 4,
  GIMBAL_TRANSITION         = 5,
  GIMBAL_ERROR  		    = -1
}gimbal_state_t;

motorPosStruct* gimbal_get(void);
bool gimbal_getError(void);
void gimbal_kill(void);

void gimbal_calibrate(void);
void gimbal_init(void);
void gimbal_ToScreen();
void gimbal_ToStable();

#endif
