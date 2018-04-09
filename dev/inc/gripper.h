#ifndef _GRIPPER_H_
#define _GRIPPER_H_

#define GRIPPER_CAN        &CAND2
#define GRIPPER_CAN_EID     0X200

#define GRIPPER_CONTROL_FREQ 500U
#define GRIPPER_MOTOR_NUM 2U

#define GRIPPER_ARM   0U
#define GRIPPER_HAND  1U

#define GRIPPER_IN_POSITION_COUNT      50U
#define GRIPPER_CONNECTION_ERROR_COUNT 10U

typedef enum {
  GRIPPER_UNINIT = 0,
  GRIPPER_INITING,
  GRIPPER_RUNNING,
  GRIPPER_ERROR
} gripper_state_t;

static const float gripper_gear_ratio[GRIPPER_MOTOR_NUM] = {27.0f, 36.0f};
static const int16_t gripper_output_max[GRIPPER_MOTOR_NUM] = {15000, 16384};
#include "chassis.h"

motorPosStruct* gripper_get(void);
void gripper_init(void);
void gripper_kill(void);
bool gripper_inPosition(const uint8_t id);
void gripper_changePos(const float pos_sp1, const float pos_sp2);
void gripper_calibrate(void);

#endif
