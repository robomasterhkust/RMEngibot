#ifndef _LIFT_H_
#define _LIFT_H_

#include "chassis.h"

#define LIFT_CAN   &CAND2
#define LIFT_CAN_EID  0x1FF
#define LIFT_GEAR_RATIO 27U
#define LIFT_CONTROL_FREQ 500U
#define LIFT_MOTOR_NUM 4U

#define LIFT_CONNECTION_ERROR_COUNT 10U

typedef enum {
  LIFT_UNINIT = 0,
  LIFT_INITING,
  LIFT_RUNNING,
  LIFT_ERROR
} lift_state_t;

typedef enum {
  LIFT0_NOT_CONNECTED = 1 << 0,
  LIFT1_NOT_CONNECTED = 1 << 1,
  LIFT2_NOT_CONNECTED = 1 << 2,
  LIFT3_NOT_CONNECTED = 1 << 3,
  LIFT_NOT_CONNECTED = 0x0000ffff
} lift_error_t;

void lift_changePos(const float pos_sp1, const float pos_sp2,
                    const float pos_sp3, const float pos_sp4);

motorPosStruct* lift_get(void);
lift_error_t lift_getError(void);
bool lift_inPosition(void);
void lift_init(void);
void lift_kill(void);
void lift_calibrate(void);

#endif
