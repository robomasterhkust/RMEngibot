#ifndef _LIFT_H_
#define _LIFT_H_

#include "chassis.h"

#define LIFT_CAN   &CAND2
#define LIFT_GEAR_RATIO 27U
#define LIFT_CONTROL_FREQ 500U

motorPosStruct* lift_get(void);
void lift_init(void);

#endif
