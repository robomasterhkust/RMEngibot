#ifndef _LIFT_H_
#define _LIFT_H_

#include "chassis.h"

#define LIFT_CAN   &CAND2
#define LIFT_GEAR_RATIO 27U
#define LIFT_CONTROL_FREQ 500U

#define   LIFT_CONNECTION_ERROR_COUNT 20U
#define   LIFT_IN_POSITION_COUNT 50U

void lift_changePos(motorPosStruct* motor, const float pos_sp);
motorPosStruct* lift_get(void);
void lift_init(void);

#endif
