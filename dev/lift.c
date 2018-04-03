#include "ch.h"
#include "hal.h"
#include "lift.h"

#include "params.h"
#include "canBusProcess.h"
#include "math_misc.h"

#define LIFT_IN_POSITION_COUNT 50U
#define LIFT_UPDATE_PERIOD_US  1000000/LIFT_CONTROL_FREQ

static lift_state_t lift_state = 0;
static lift_error_t lift_error = 0;

static ChassisEncoder_canStruct* encoders;
static volatile motorPosStruct motors[EXTRA_MOTOR_NUM];
static float offset[EXTRA_MOTOR_NUM];
static lpfilterStruct lp_speed[EXTRA_MOTOR_NUM];

static param_t weight;
static uint8_t transition[EXTRA_MOTOR_NUM] = {0,0,0,0};
static uint8_t on_foot[EXTRA_MOTOR_NUM] = {0,0,0,0};

static pid_controller_t controllers[EXTRA_MOTOR_NUM];

motorPosStruct* lift_get(void)
{
  return motors;
}

lift_error_t lift_getError(void)
{
  uint32_t errorFlag = lift_error;
  lift_error = 0;
  return errorFlag;
}

bool lift_inPosition(void)
{
  return motors[0].in_position == LIFT_IN_POSITION_COUNT
    && motors[1].in_position == LIFT_IN_POSITION_COUNT
    && motors[2].in_position == LIFT_IN_POSITION_COUNT
    && motors[3].in_position == LIFT_IN_POSITION_COUNT;
}

#define   LIFT_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define   LIFT_SPEED_PSC 1.0f/((float)LIFT_GEAR_RATIO)

static void lift_kill(void)
{
  LEDY_ON();

  if(lift_state == LIFT_RUNNING)
  {
    lift_state = LIFT_ERROR;
    can_motorSetCurrent(LIFT_CAN, CHASSIS_CAN_EID, 0, 0, 0, 0);
  }
}

static void lift_encoderUpdate(void)
{
  uint8_t i;
  for (i = 0; i < EXTRA_MOTOR_NUM; i++)
  {
    if(encoders[i].updated)
    {
      //Check validiaty of can connection
      encoders[i].updated = false;

      float pos_input = encoders[i].raw_angle*LIFT_ANGLE_PSC;
      float speed_input = encoders[i].raw_speed*LIFT_SPEED_PSC;

      if((motors[i]._prev < 0.6f && pos_input > 5.68f) ||
        (speed_input < -80.0f && pos_input > motors[i]._prev))
        motors[i].rev--;
      if((motors[i]._prev > 5.68f && pos_input < 0.6f) ||
        (speed_input > 80.0f && pos_input < motors[i]._prev))
        motors[i].rev++;

      motors[i]._prev = pos_input;
      pos_input += motors[i].rev * 2*M_PI;

      motors[i]._pos =  pos_input/LIFT_GEAR_RATIO;
      motors[i]._speed = lpfilter_apply(&lp_speed[i], speed_input);
      motors[i]._wait_count = 1;
    }
    else
    {
      motors[i]._wait_count++;
      if(motors[i]._wait_count > LIFT_CONNECTION_ERROR_COUNT)
      {
        motors[i]._wait_count = 1;
        lift_error |= LIFT0_NOT_CONNECTED << i;

        lift_kill();
      }
    }

    if((motors[i].pos_sp - motors[i]._pos) < 1.0f && (motors[i].pos_sp - motors[i]._pos) > -1.0f)
      motors[i].in_position++;
    else
      motors[i].in_position = 0;

    if(motors[i].in_position > LIFT_IN_POSITION_COUNT)
      motors[i].in_position = LIFT_IN_POSITION_COUNT;
  }
}

void lift_changePos(const float pos_sp1, const float pos_sp2,
                    const float pos_sp3, const float pos_sp4)
{
  if(offset[0] - pos_sp1 != motors[0].pos_sp)
  {
    motors[0].in_position = 0;
    motors[0].pos_sp = offset[0] - pos_sp1;
  }
  if(offset[1] - pos_sp2 != motors[1].pos_sp)
  {
    motors[1].in_position = 0;
    motors[1].pos_sp = offset[1] - pos_sp2;
  }
  if(offset[2] - pos_sp3 != motors[2].pos_sp)
  {
    motors[2].in_position = 0;
    motors[2].pos_sp = offset[2] - pos_sp3;
  }
  if(offset[3] - pos_sp4 != motors[3].pos_sp)
  {
    motors[3].in_position = 0;
    motors[3].pos_sp = offset[3] - pos_sp4;
  }
}

void lift_calibrate(void)
{
  //To initialize the lift wheel, a calibration is needed
  //CAUTION!! Moving lift wheel may cause injury, stay back during power up!!

  uint8_t init_state = 0;

  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 5000));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 5000));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 5000));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  chThdSleepMilliseconds(500);

  while(init_state < 0x0f)
  {
    if(!LS2_DOWN())
      motors[0].pos_sp+= 0.02f;
    else
    {
      init_state |= 0x01;
      offset[0] = motors[0]._pos;
    }
    if(!LS1_DOWN())
      motors[1].pos_sp+= 0.02f;
    else
    {
      init_state |= 0x02;
      offset[1] = motors[1]._pos;
    }
    if(!LS0_DOWN())
      motors[2].pos_sp+= 0.02f;
    else
    {
      init_state |= 0x04;
      offset[2] = motors[2]._pos;
    }
    if(!LS3_DOWN())
      motors[3].pos_sp+= 0.02f;
    else
    {
      init_state |= 0x08;
      offset[3] = motors[3]._pos;
    }

    chThdSleepMilliseconds(2);
  }
}

#define OUTPUT_MAX  16384

#define ONFOOT_TRANSITION_TIME_MS   100
#define ONFOOT_TRANSITION_TH        20.0f
#define ONFOOT_TRANSITION_PERIOD    50

#define LIFT_IDLE       0
#define LIFT_GOING_UP   1
#define LIFT_GOING_DOWN 2

static int16_t lift_controlPos
  (const motorPosStruct* const motor, pid_controller_t* const controller, const uint8_t on_foot)
{
  float error = motor->pos_sp - motor->_pos;
  controller->error_int += error * controller->ki;
  controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
  float output =
    error*controller->kp + controller->error_int - motor->_speed * controller->kd + on_foot * weight;

  return (int16_t)(boundOutput(output,OUTPUT_MAX));
}

static THD_WORKING_AREA(lift_control_wa, 512);
static THD_FUNCTION(lift_control, p)
{
  (void)p;
  chRegSetThreadName("lift wheel controller");

  float output[4];
  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += US2ST(LIFT_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    lift_encoderUpdate();

    uint8_t i;

    for (i = 0; i < EXTRA_MOTOR_NUM; i++)
    {
      if(fabsf(motors[i].pos_sp) > ONFOOT_TRANSITION_TH && on_foot[i] < ONFOOT_TRANSITION_PERIOD)
        transition[i] = LIFT_GOING_UP;
      if(fabsf(motors[i].pos_sp < ONFOOT_TRANSITION_TH && on_foot[i] > 0))
        transition[i] = LIFT_GOING_DOWN;

      if(transition[i] == LIFT_GOING_UP)
        on_foot[i]++;
      else if(transition[i] == LIFT_GOING_DOWN)
        on_foot[i]--;

      if(on_foot[i] == 0 || on_foot[i] == ONFOOT_TRANSITION_PERIOD)
        transition[i] = LIFT_IDLE;
    }

    for(i = 0; i < 4; i++)
      output[i] = lift_controlPos(&motors[i], &controllers[i], on_foot[i]);

    if(lift_state == LIFT_RUNNING)
      can_motorSetCurrent(LIFT_CAN, CHASSIS_CAN_EID,
        	output[3], output[2], output[1], output[0]);
  }
}

static const FRLName = "FR_lift";
static const FLLName = "FL_lift";
static const BLLName = "BL_lift";
static const BRLName = "BR_lift";

#define LIFT_ERROR_INT_MAX  30000
void lift_init(void)
{
  memset(&motors, 0, sizeof(motorPosStruct)*EXTRA_MOTOR_NUM);
  encoders = can_getExtraMotor();
  uint8_t i;
  for (i = 0; i < EXTRA_MOTOR_NUM; i++) {
    lpfilter_init(lp_speed + i, LIFT_CONTROL_FREQ, 24);
    motors[i].speed_sp = 0.0f;
    motors[i]._speed = 0.0f;
    motors[i].pos_sp = 0.0f;
    motors[i]._pos = 0.0f;
    motors[i]._prev = 0.0f;

    controllers[i].error_int = 0.0f;
    controllers[i].error_int_max = LIFT_ERROR_INT_MAX;
  }
  params_set(&controllers[0], 13,3,BLLName,subname_PID,PARAM_PUBLIC);
  params_set(&controllers[1], 14,3,BRLName,subname_PID,PARAM_PUBLIC);
  params_set(&controllers[2], 15,3,FRLName,subname_PID,PARAM_PUBLIC);
  params_set(&controllers[3], 16,3,FLLName,subname_PID,PARAM_PUBLIC);

  chThdCreateStatic(lift_control_wa, sizeof(lift_control_wa),
                          NORMALPRIO, lift_control, NULL);

  lift_state = LIFT_RUNNING;
}
