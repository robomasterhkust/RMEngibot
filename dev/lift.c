#include "ch.h"
#include "hal.h"
#include "lift.h"

#include "params.h"
#include "canBusProcess.h"
#include "math_misc.h"

#define LIFT_UPDATE_PERIOD_US  1000000/LIFT_CONTROL_FREQ

static ChassisEncoder_canStruct* encoders;
static volatile motorPosStruct motors[EXTRA_MOTOR_NUM];
static lpfilterStruct lp_speed[EXTRA_MOTOR_NUM];
static param_t weight;

static uint8_t transition[EXTRA_MOTOR_NUM] = {0,0,0,0};
static uint8_t on_foot[EXTRA_MOTOR_NUM] = {0,0,0,0};

static pid_controller_t controllers[EXTRA_MOTOR_NUM];

motorPosStruct* lift_get(void)
{
  return motors;
}

#define   LIFT_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define   LIFT_SPEED_PSC 1.0f/((float)LIFT_GEAR_RATIO)
#define   LIFT_CONNECTION_ERROR_COUNT 20U
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

      if(motors[i]._prev < 1.9f && pos_input > 4.38f)
        motors[i].rev--;
      if(motors[i]._prev > 4.38f && pos_input < 1.9f)
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
      }
    }
  }
}

#define OUTPUT_MAX  30000

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
  while(!chThdShouldTerminateX())
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

    can_motorSetCurrent(LIFT_CAN, CHASSIS_CAN_EID,
        	output[3], output[2], output[1], output[0]);
  }
}

static const FRLName = "FR_lift";
static const FLLName = "FL_lift";
static const BLLName = "BL_lift";
static const BRLName = "BR_lift";
static const weightName = "lift weight";
static const weightSubName = "1";

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
  params_set(&weight, 17,1,weightName,weightSubName,PARAM_PUBLIC);

  chThdCreateStatic(lift_control_wa, sizeof(lift_control_wa),
                          NORMALPRIO, lift_control, NULL);

}
