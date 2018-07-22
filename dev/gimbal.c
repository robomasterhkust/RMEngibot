#include "ch.h"
#include "hal.h"

#include "gimbal.h"
#include "canBusProcess.h"

#include "math_misc.h"
#include "system_error.h"
#include "params.h"
#include "dbus.h"
#include "attitude.h"

static ChassisEncoder_canStruct* gimbal_encoders;
static volatile motorPosStruct motors;
static float offset;

static bool in_position;

static pid_controller_t gimbal_pos;
static pi_controller_t  gimbal_vel;
static pid_controller_t gimbal_atti;

static gimbal_state_t gimbal_state = GIMBAL_UNINIT;
static float atti_cmd = 0.0f;
static RC_Ctl_t* rc;

static param_t gimbal_pos_sp[2];
static param_t gimbal_pos_limit[2];
static uint8_t gimbal_limit_status; //gimbal limit position status

static gimbal_error_t gimbal_error;

typedef enum{
  GIMBAL_AT_LOW_LIMIT = 1,
  GIMBAL_AT_UP_LIMIT = 2
}gimbal_pos_limit_t;

static float EulerAngle[3];
static lpfilterStruct lp_speed;

static PIMUStruct pIMU;

static thread_t* gimbal_thd;
static float gimbal_init_pos;

static void gimbal_changePos(const float pos_sp)
{
  if(offset - pos_sp != motors.pos_sp)
  {
    in_position = false;
    motors.pos_sp = offset - pos_sp;
  }
}

motorPosStruct* gimbal_get(void)
{
  return &motors;
}

gimbal_error_t gimbal_getError(void)
{
  return gimbal_error;
}

float* gimbal_getEulerAngle(void)
{
  return EulerAngle;
}

void gimbal_kill(void){
  gimbal_state = GIMBAL_ERROR;
  system_setErrorFlag();
  can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID, 0, 0, 0, 0);

  if(gimbal_thd != NULL)
  {
    chThdTerminate(gimbal_thd);
    gimbal_thd = NULL;
  }
}

static void gimbal_getAtti(float* EulerAngle)
{
  float  current_angle = -(motors._pos - gimbal_init_pos);
  //parameters for the quaternion
  float q_result[4];
  float q[4];
  float Euler[3] = {0,current_angle,0};

  euler2quarternion(Euler,q);
  quarternionXquarternion(pIMU->qIMU,  q, q_result);
  quarternion2euler(q_result,EulerAngle);

  if(current_angle < -gimbal_pos_limit[0] || pIMU->euler_angle[Pitch] < -M_PI_2_F + 0.3f)
    gimbal_limit_status = GIMBAL_AT_UP_LIMIT;
  else if(current_angle > gimbal_pos_limit[1] || pIMU->euler_angle[Pitch] > M_PI_2_F - 0.3f)
    gimbal_limit_status = GIMBAL_AT_LOW_LIMIT;
  else if(current_angle > -gimbal_pos_limit[0] + 0.05f && current_angle < gimbal_pos_limit[1] - 0.05f)
    gimbal_limit_status = 0;

}

static void gimbal_attiCmd(void)
{
  float rc_input = -mapInput((float)rc->rc.channel3, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX,
                              -GIMBAL_MAX_SPEED, GIMBAL_MAX_SPEED)
               -  mapInput((float)rc->mouse.y, -25, 25, -GIMBAL_MAX_SPEED, GIMBAL_MAX_SPEED);

  rc_input *= cosf(pIMU->euler_angle[Roll]);

  float limit_speed = pIMU->gyroData[Y];
  if(gimbal_limit_status == GIMBAL_AT_UP_LIMIT && rc_input < limit_speed)
    rc_input = limit_speed;
  else if(gimbal_limit_status == GIMBAL_AT_LOW_LIMIT && rc_input > limit_speed)
    rc_input = limit_speed;

  atti_cmd += rc_input * (1.0f / GIMBAL_CONTROL_FREQ);
}

static void gimbal_encoderUpdate(void)
{

	if(gimbal_encoders->updated)
	{
	  //Check validiaty of can connection
   gimbal_encoders->updated = false;

   float speed_input = (float)gimbal_encoders->raw_speed/ GIMBAL_GEAR_RATIO;
   motors._pos = gimbal_encoders->radian_angle / GIMBAL_GEAR_RATIO;
   motors._speed = lpfilter_apply(&lp_speed, speed_input) * 2*M_PI / 60;

   motors._wait_count = 1;
 }
 else
 {
   motors._wait_count++;
   if(motors._wait_count > GIMBAL_CONNECTION_ERROR_COUNT)
   {
     motors._wait_count = 1;
     gimbal_error |= GIMBAL_NO_CONNECTION;
     gimbal_kill();
   }
 }

 if((motors.pos_sp - motors._pos) < 0.01f &&
  (motors.pos_sp - motors._pos) > -0.01f)
   motors.in_position++;
 else
   motors.in_position = 0;

 if(motors.in_position > GIMBAL_IN_POSITION_COUNT)
 {
   motors.in_position = GIMBAL_IN_POSITION_COUNT;
	  in_position = true; //Motor is regarded as moved to postion
 }
}

static int16_t gimbal_controlPos(const motorPosStruct* const motor,
  pid_controller_t* const controller, const int16_t output_max)
{
  float error = motor->pos_sp - motor->_pos;
  controller->error_int += error * controller->ki;
  controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
  float output =
  error*controller->kp + controller->error_int - motor->_speed * controller->kd;

  return (int16_t)(boundOutput(output,output_max));
}

static inline float gimbal_controlSpeed (pi_controller_t *const vel,
  float speed_sp, float angle_vel)
{
  float error = speed_sp - angle_vel;
  vel->error_int += error * vel->ki;
  bound(&(vel->error_int), vel->error_int_max);

  float output = vel->kp * error + vel->error_int;
  return output;
}

static inline float gimbal_controlAttitude (pid_controller_t *const atti,
                                            const float target_atti,
                                            const float curr_atti,
                                            const float curr_atti_d)
{
  float error = target_atti - curr_atti;
  atti->error_int += error * atti->ki;
  bound(&(atti->error_int), atti->error_int_max);

  //PI-D attitude controller
  float output = atti->kp * error + atti->error_int - atti->kd * curr_atti_d;
  return output;
}


//static int16_t gimbal_controlspeed()
#define GIMBAL_UPDATE_PERIOD_US  1000000/GIMBAL_CONTROL_FREQ
static THD_WORKING_AREA(gimbal_control_wa, 1024);
static THD_FUNCTION(gimbal_control, p)
{
  (void)p;
  chRegSetThreadName("gimbal controller");

  float output;
  float output_max;
  uint32_t tick = chVTGetSystemTimeX();
  while(!chThdShouldTerminateX())
  {
    tick += US2ST(GIMBAL_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    //we can use this pos_cmd to control the gimabl
    gimbal_encoderUpdate();

    //Use controller to toggle gimbal state to screen
    if(rc->rc.channel3 < RC_CH_VALUE_MIN + 10)
    {
      gimbal_changePos(gimbal_pos_sp[1]);
      gimbal_state = GIMBAL_SCREEN; //Use controller to toggle gimbal state to screen
    }
    else if(gimbal_state == GIMBAL_SCREEN && rc->rc.channel3 > RC_CH_VALUE_MAX - 100)
    {
      gimbal_changePos(gimbal_pos_sp[0]);
      gimbal_state = GIMBAL_TRANSITION;
    }

    switch(gimbal_state)
    {
      case GIMBAL_TRANSITION:
        if(in_position)
        {
          gimbal_getAtti(EulerAngle);
          atti_cmd = EulerAngle[Pitch];
          gimbal_state = GIMBAL_STABLE;
          break;
        }
      case GIMBAL_INITING:
        output_max = 6000;

        output = gimbal_controlPos(&motors, &gimbal_pos, output_max);
        can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,
          output, 0, 0, 0);
        break;
      case GIMBAL_SCREEN:
      case GIMBAL_SCREEN_LOCK:
        output_max = GIMBAL_MAX_OUTPUT;

        output = gimbal_controlPos(&motors, &gimbal_pos, output_max);
        can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,
          output, 0, 0, 0);
        break;
      case GIMBAL_STABLE:
        output_max = GIMBAL_MAX_OUTPUT;

        gimbal_getAtti(EulerAngle);
        gimbal_attiCmd(); //RC attitude cmd input

        float angleVel = pIMU->gyroData[Y] + motors._speed;
                                        // ^
    //TODO: Check the sign here -----------|
    //Turning down --> positive

        float cosRoll = cosf(EulerAngle[Roll]);
        float d_pitch = cosRoll * angleVel - sinf(EulerAngle[Roll]) * pIMU->gyroData[Z];

        float speed_sp;
        if(cosRoll > 0.1f)
        {
          float pitch_atti_out = gimbal_controlAttitude(&gimbal_atti, atti_cmd,
                                                EulerAngle[Pitch], d_pitch);
          speed_sp = pitch_atti_out / cosRoll;
        }
        else //The vehicle flips over
          speed_sp = 0.0f;

        output = gimbal_controlSpeed(&gimbal_vel, speed_sp, angleVel);

        can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,
          output, 0, 0, 0);

        break;
      default:
        gimbal_kill();
        break;
    }
  }
}

void gimbal_ToScreen()
{
  //Only do action when gimbal is in stable mode
  if(gimbal_state == GIMBAL_STABLE || gimbal_state == GIMBAL_SCREEN)
  {
    gimbal_changePos(gimbal_pos_sp[1]);
    gimbal_state = GIMBAL_SCREEN_LOCK;
  }
}

void gimbal_ToStable()
{
  //Only do action when gimbal is in screen mode
  if(gimbal_state == GIMBAL_SCREEN_LOCK)
  {
    gimbal_changePos(gimbal_pos_sp[0]);
    gimbal_state = GIMBAL_TRANSITION;
  }
}

#define STALL_COUNT_MAX 100U
void gimbal_calibrate(void){
  bool init_state = false;
  gimbal_state = GIMBAL_INITING;
  float prev_pos;

  uint8_t stall_count = 0;
  const float motor_step = 0.04f;

  while(!init_state)
  {
    if(stall_count < STALL_COUNT_MAX)
    {
    	motors.pos_sp += motor_step;
    	if(motors._pos - prev_pos < motor_step * 0.2 ||
         motors._pos - prev_pos > -motor_step * 0.2)
    		stall_count++;
    	else if(stall_count > 10)
    		stall_count -= 10;
    	else
    		stall_count = 0;

    	prev_pos = motors._pos;
    }
    else
    {
    	init_state = true;
    	offset = motors._pos;
    }

    chThdSleepMilliseconds(30);
  }

  gimbal_changePos(gimbal_pos_sp[0]);

  while(!in_position)
    chThdSleepMilliseconds(100); //Wait for gimbal to reach set position
  chSysLock();

  gimbal_init_pos = motors._pos;
  gimbal_getAtti(EulerAngle);
  atti_cmd = EulerAngle[Pitch];
  gimbal_state = GIMBAL_STABLE;

  chSysUnlock();
}

static const char GimbalPos[] = "GimbalPos";
static const char GimbalVel[] = "GimbalVel";
static const char GimbalLimit[] = "Gimbal Limit";
static const char GimbalAtti[] = "GimbalAtti";

static const char GimbalPosName[] = "gimbal Pos";
static const char GimbalPosSubName[] = "up down";

static THD_WORKING_AREA(gimbal_test_wa, 512);
static THD_FUNCTION(gimbal_test, p)
{
  (void)p;
  chRegSetThreadName("gimbal test");

  float output;
  float output_max;

  gimbal_state = GIMBAL_INITING; //Use controller to toggle gimbal state to screen

  uint32_t tick = chVTGetSystemTimeX();
  while(!chThdShouldTerminateX())
  {
    tick += US2ST(GIMBAL_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      system_setTempWarningFlag();
      tick = chVTGetSystemTimeX();
    }

    //we can use this pos_cmd to control the gimabl
    gimbal_encoderUpdate();
    gimbal_getAtti(EulerAngle);

    //Use controller to toggle gimbal state to screen
    if(rc->rc.channel3 < RC_CH_VALUE_MIN + 10)
    {
      gimbal_state = GIMBAL_SCREEN;
      gimbal_changePos(gimbal_pos_sp[1]);
    }
    else if(rc->rc.channel3 > RC_CH_VALUE_MAX - 10)
    {
      gimbal_state = GIMBAL_SCREEN;
      gimbal_changePos(gimbal_pos_sp[0]);
    }

    switch(gimbal_state)
    {
      case GIMBAL_INITING:
        output_max = 10000;

        output = gimbal_controlPos(&motors, &gimbal_pos, output_max);
        can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,
          output, 0, 0, 0);
        break;
      case GIMBAL_SCREEN:
      case GIMBAL_SCREEN_LOCK:
        output_max = GIMBAL_MAX_OUTPUT;

        output = gimbal_controlPos(&motors, &gimbal_pos, output_max);
        can_motorSetCurrent(GIMBAL_CAN, GIMBAL_CAN_EID,
          output, 0, 0, 0);
        break;
    }
  }
}

#define GIMBAL_ERROR_INT_MAX  4000
void gimbal_init(void)
{
  memset(&motors, 0, sizeof(motorPosStruct));
  gimbal_encoders = can_getExtraMotor() + 6;
  rc = RC_get();

  lpfilter_init(&lp_speed , GIMBAL_CONTROL_FREQ, 24);
  gimbal_pos.error_int_max = GIMBAL_ERROR_INT_MAX;
  gimbal_vel.error_int_max = GIMBAL_ERROR_INT_MAX;
  gimbal_atti.error_int_max = 2.0f;

  pIMU = imu_get();

  params_set(&gimbal_atti, 0, 3, GimbalAtti, subname_PID, PARAM_PUBLIC);
  params_set(&gimbal_vel,  1, 2, GimbalVel,  subname_PI , PARAM_PUBLIC);
  params_set(&gimbal_pos_limit,  2, 2, GimbalLimit,  GimbalPosSubName , PARAM_PUBLIC);
  params_set(&gimbal_pos, 24, 3, GimbalPos, subname_PID,PARAM_PUBLIC);
  params_set(gimbal_pos_sp, 25, 2, GimbalPosName, GimbalPosSubName , PARAM_PUBLIC);

  chThdSleepSeconds(3);

  gimbal_state = GIMBAL_INITING;
  gimbal_thd = chThdCreateStatic(gimbal_control_wa, sizeof(gimbal_control_wa),
    NORMALPRIO, gimbal_control, NULL);
}
