#include "gimbal.h"
#include "canBusProcess.h"

#include <math.h>
#include "math_misc.h"
#include "system_error.h"

#include "ch.h"
#include "hal.h"
#include "params.h"


static ChassisEncoder_canStruct* gimbal_encoders;
static volatile motorPosStruct motors;
static float offset;
static lpfilterStruct lp_speed;
static bool in_position;
static pid_controller_t controllers;
static gimbal_state_t gimbal_state = GIMBAL_UNINIT;
static param_t gimbal_pos_sp[2];

motorPosStruct* gimbal_get(void)
{
  return motors;
}



void gimbal_kill(void){
	system_setWarningFlag();
  can_motorSetCurrent(GRIPPER_CAN, GRIPPER_CAN_EID, 0, 0, 0, 0);

}


#define  GIMBAL_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF

static void gimbal_encoderUpdate(void)
{

	if(gimbal_encoders.updated)
	{
	  //Check validiaty of can connection
   gimbal_encoders.updated = false;

   float pos_input = gimbal_encoders.raw_angle*GIMBAL_ANGLE_PSC;
   float speed_input = gimbal_encoders.raw_speed/ gimbal_gear_ratio;


   motors.rev = gimbal_encoders.round_count;

   motors._prev = pos_input;
   pos_input += motors[i].rev * 2*M_PI;

   motors._pos =  pos_input / gimbal_gear_ratio;
   motors._speed = lpfilter_apply(&lp_speed, speed_input);
   motors._wait_count = 1;
 }
 else
 {
   motors._wait_count++;
   if(motors._wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
   {
     motors._wait_count = 1;
     gimbal_kill();
   }
 }

 if((motors.pos_sp - motors._pos) < 2 * M_PI &&
  (motors.pos_sp - motors._pos) > -2 * M_PI)
   motors.in_position++;
 else
   motors.in_position = 0;

 if(motors.in_position > GIMBAL_IN_POSITION_COUNT)
 {
   motors.in_position = GIMBAL_IN_POSITION_COUNT;
	  in_position = true; //Motor is regarded as moved to postion
	}

}


void gimbal_changePos(const float pos_sp)
{
  if(offset - pos_sp != motors.pos_sp)
  {
    in_position = false;
    motors.pos_sp = offset - pos_sp;
  }
}

static int16_t gimbal_controlPos
(const motorPosStruct* const motor, pid_controller_t* const controller,
  const int16_t output_max)
{
  float error = motor->pos_sp - motor->_pos;
  controller->error_int += error * controller->ki;
  controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
  float output =
  error*controller->kp + controller->error_int - motor->_speed * controller->kd;

  return (int16_t)(boundOutput(output,output_max));
}


#define GIMBAL_UPDATE_PERIOD_US  1000000/GIMBAL_CONTROL_FREQ
static THD_WORKING_AREA(gimbal_control_wa, 512);
static THD_FUNCTION(gimbal_control, p)
{
  (void)p;
  chRegSetThreadName("gimbal controller");

  float output;
  float output_max;
  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += US2ST(GRIPPER_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    gimbal_encoderUpdate();


    if(gimbal_state == GIMBAL_INITING)
    {
    	output_max = 3000;
    }
    else
    	output_max = gimbal_output_max;

    output =
    gimbal_controlPos(&motors, &controllers, output_max);
    

    can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,
      output, 0, 0, 0);
  }
}


#define STALL_COUNT_MAX 100U
void gimbal_calibrate(void){
  bool init_state = 0;

  float prev_pos;

  uint8_t init_count = 0;
  uint8_t stall_count = 0;

  const float motor_step = 0.02f;

  while(init_count < 1)
  {
    uint8_t i;
    init_count = 0;  //finish initialization only if all motor calibration finishes
    
    if(stall_count[i] < STALL_COUNT_MAX)
    {
    	motors.pos_sp += motor_step;
    	if(motors._pos - prev_pos < motor_step * 0.2f)
    		stall_count++;
    	else if(stall_count > 10)
    		stall_count= 10;
    	else
    		stall_count = 0;

    	prev_pos = motors._pos;
    }
    else
    {
    	init_state = true;
    	offset = motors._pos;
    }

    init_count = init_state ;
  }

  chThdSleepMilliseconds(2);
  
  gripper_state = GRIPPER_STABLE;
}

static const GimbalName = "Gimbal";

#define GRIPPER_ERROR_INT_MAX  30000

static const char PosName[] = "gimbal Pos";
static const char PosSubName[] = "up down";

void gimbal_init(void)
{
  memset(&motors, 0, sizeof(motorPosStruct) * GRIPPER_MOTOR_NUM);
  gimbal_encoders = can_getExtraMotor() + 6;

  
  lpfilter_init(lp_speed , GIMBAL_CONTROL_FREQ, 24);
  motors.speed_sp = 0.0f;
  motors._speed = 0.0f;
  motors.pos_sp = 0.0f;
  motors._pos = 0.0f;
  motors._prev = 0.0f;

  controllers.error_int = 0.0f;
  controllers.error_int_max = GRIPPER_ERROR_INT_MAX;
  
  params_set(&controllers, 24,3,GimbalName, subname_PID,PARAM_PUBLIC);
  params_set(&pos_sp, 25, 2, PosName, PosSubName , PARAM_PUBLIC);

  gimbal_state = GIMBAL_INITING;

  chThdCreateStatic(gimbal_control_wa, sizeof(gimbal_control_wa),
    NORMALPRIO, gimbal_control, NULL);
}
