#include "gimbal.h"
#include "canBusProcess.h"

#include <math.h>
#include "math_misc.h"
#include "system_error.h"
#include "shell.h"
#include "ch.h"
#include "hal.h"
#include "params.h"
#include "dbus.h"
#include "attitude.h"
static ChassisEncoder_canStruct* gimbal_encoders;
static volatile motorPosStruct motors;
static float offset;
static lpfilterStruct lp_speed;
static bool in_position;
static pid_controller_t controllers;
static gimbal_state_t gimbal_state = GIMBAL_UNINIT;
static const float gimbal_gear_ratio = 36.0f;
static const int16_t gimbal_output_max = 16384;
static float pos_cmd = 0.0f;
static RC_Ctl_t* rc;
static int error = 0;
static float angel_offset;
static param_t gimbal_pos_sp[2];


static gimbal_position_t gimbal_position = GIMBAL_UP;


//parameters for the quaternion
float  current_angle ;
float result[4];
float EulerAngel[3];
float q[4];


void gimbal_up(){
  gimbal_position = GIMBAL_UP;
   gimbal_changePos(gimbal_pos_sp[0]- pos_cmd);
}
void gimbal_down(){
  gimbal_position = GIMBAL_DOWN;
   gimbal_changePos(gimbal_pos_sp[1]);
}

float*get_Euler(){
  return EulerAngel;
}
motorPosStruct* gimbal_get(void)
{
  return &motors;
}

gimbal_state_t gimbal_getError(void)
{
  int error_flag = error ;
  error = 0;
  return error_flag;
}

void gimbal_kill(void){
  gimbal_state = GIMBAL_ERROR;
  error = 1 ;
  system_setWarningFlag();
  can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID, 0, 0, 0, 0);
}


#define  GIMBAL_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF

static void gimbal_encoderUpdate(void)
{

	if(gimbal_encoders->updated)
	{
	  //Check validiaty of can connection
   gimbal_encoders->updated = false;

   float pos_input = gimbal_encoders->raw_angle*GIMBAL_ANGLE_PSC;
   float speed_input = gimbal_encoders->raw_speed/ gimbal_gear_ratio;


   motors.rev = gimbal_encoders->round_count;

   motors._prev = pos_input;
   pos_input += motors.rev * 2*M_PI;

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


    tick += US2ST(GIMBAL_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }
    int16_t input = rc->rc.channel3 - RC_CH_VALUE_OFFSET +
    ((rc->keyboard.key_code & KEY_Q) - (rc->keyboard.key_code & KEY_E)) * 200;

if(input > 400)
      pos_cmd += 0.01f;
    else if(input > 100)
      pos_cmd += 0.0025f;
    else if(input < -400)
      pos_cmd -= 0.01f;
    else if(input < -100)
      pos_cmd -= 0.0025f;


    //we can use this pos_cmd to control the gimabl 
    gimbal_encoderUpdate();


    if(gimbal_state == GIMBAL_INITING)
      output_max = 2000;
    else
      output_max = gimbal_output_max;

      // current_angle = (gimbal_encoders->radian_angle - angel_offset)/gimbal_gear_ratio ;

      // PIMUStruct pIMU = imu_get();
      // float Euler[3] = {0,current_angle,0};

      // euler2quarternion(Euler,q);

      // quarternionXquarternion(q,pIMU->qIMU,result);

      // quarternion2euler(result,EulerAngel);

    output = gimbal_controlPos(&motors, &controllers, output_max);


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
  gimbal_position = GIMBAL_INITING;
  const float motor_step = 0.002f;
  BUZZER(500);
  chThdSleepMilliseconds(500);
  BUZZER(0);
  while(init_count < 1)
  {
    uint8_t i;
    init_count = 0;  //finish initialization only if all motor calibration finishes
    
    if(stall_count < STALL_COUNT_MAX)
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


    chThdSleepMilliseconds(2);
  }

  gimbal_changePos(gimbal_pos_sp[0]);

  gimbal_state = GIMBAL_STABLE;

  do{
    chThdSleepMilliseconds(2000);

  }while(in_position == false);

  angel_offset = gimbal_encoders->radian_angle;// 

}

static const GimbalName = "Gimbal";

static const char GimbalPosName[] = "gimbal Pos";
static const char GimbalPosSubName[] = "up down";
#define GIMBAL_ERROR_INT_MAX  30000

void gimbal_init(void)
{
  memset(&motors, 0, sizeof(motorPosStruct));
  gimbal_encoders = can_getExtraMotor() + 6;
  rc = RC_get();
  
  lpfilter_init(&lp_speed , GIMBAL_CONTROL_FREQ, 24);
  motors.speed_sp = 0.0f;
  motors._speed = 0.0f;
  motors.pos_sp = 0.0f;
  motors._pos = 0.0f;
  motors._prev = 0.0f;

  controllers.error_int = 0.0f;
  controllers.error_int_max = GIMBAL_ERROR_INT_MAX;
  
  params_set(&controllers, 24,3,GimbalName, subname_PID,PARAM_PUBLIC);
  params_set(gimbal_pos_sp, 25, 2, GimbalPosName, GimbalPosSubName , PARAM_PUBLIC);

  gimbal_state = GIMBAL_INITING;

  chThdCreateStatic(gimbal_control_wa, sizeof(gimbal_control_wa),
    NORMALPRIO, gimbal_control, NULL);
}
