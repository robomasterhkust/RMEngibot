#ifndef _LIFT_H_
#define _LIFT_H_

#include "chassis.h"

#define LIFT_CAN   &CAND2
#define LIFT_GEAR_RATIO 27U
#define LIFT_CONTROL_FREQ 500U

#define   LIFT_CONNECTION_ERROR_COUNT 10U
#define   LIFT_IN_POSITION_COUNT 50U

typedef enum {
  LIFT0_NOT_CONNECTED = 1 << 0,
  LIFT1_NOT_CONNECTED = 1 << 1,
  LIFT2_NOT_CONNECTED = 1 << 2,
  LIFT3_NOT_CONNECTED = 1 << 3,
  LIFT_NOT_CONNECTED = 0x0000ffff
} lift_error_t;

void lift_changePos(motorPosStruct* motor, const float pos_sp);
motorPosStruct* lift_get(void);
lift_error_t lift_getError(void);
void lift_init(void);

#endif
