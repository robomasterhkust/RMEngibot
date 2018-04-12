/*
 * chassis.c
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "mpu6500.h"
#include "chassis.h"
#include "math_misc.h"

static volatile chassisStruct chassis;
pi_controller_t motor_vel_controllers[CHASSIS_MOTOR_NUM];
pid_controller_t heading_controller;
lpfilterStruct lp_speed[CHASSIS_MOTOR_NUM];
static RC_Ctl_t* pRC;
static PIMUStruct pIMU;

//RC input of chassis controller
static float   strafe_rc = 0.0f, drive_rc = 0.0f, heading_rc = 0.0f;
static int8_t  rc_reversed       = 1; //-1: reverse, 0: kill, 1: no reverse
static float   rc_speed_limit_sp = 1.0f;
static float   rc_speed_limit    = 1.0f; //value from 0 ~ 1
static float   rc_accl_limit     = 100.0f;  //value from 0-100
static float   heading_sp;

#define chassis_canUpdate()   \
  (can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, \
    0, 0, 0, 0))

chassisStruct* chassis_get(void)
{
  return &chassis;
}

uint32_t chassis_getError(void)
{
  uint32_t errorFlag = chassis.errorFlag;
  chassis.errorFlag = 0;
  return errorFlag;
}

void chassis_setSpeedLimit(const float speed)
{
  if(speed >= 0.0f && speed <= 1.0f)
    rc_speed_limit_sp = speed;
}

void chassis_setAcclLimit(const float accl)
{
  if(accl > 0.0f && accl <= 100.0f)
    rc_accl_limit = accl;
}

void chassis_headingLockCmd(const uint8_t cmd)
{
  if(cmd == DISABLE)
    chassis.state &= ~(CHASSIS_HEADING_LOCK);
  else if(!(chassis.state & CHASSIS_HEADING_LOCK) && cmd != DISABLE)
  {
    chSysLock();
    chassis.state |= CHASSIS_HEADING_LOCK;
    heading_sp = pIMU->euler_angle[Yaw];
    chSysUnlock();
  }
}

void chassis_tempSuspend(const uint8_t cmd)
{
  if(cmd == DISABLE)
    chassis.state &= ~(CHASSIS_SUSPEND);
  else
    chassis.state |= CHASSIS_SUSPEND;
}

static void chassis_kill(void)
{
  LEDY_ON();
  if(chassis.state & CHASSIS_RUNNING)
  {
    chassis.state = CHASSIS_ERROR;
    can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, 0, 0, 0, 0);
  }
}

// Set dead-zone to 2% range to provide smoother control
#define THRESHOLD (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN)/100
static void chassis_inputCmd(void)
{
  float   last_strafe = strafe_rc,
          last_drive = drive_rc; //Used to limit strafe and drive acceleration

  int16_t RX_X2 = (pRC->rc.channel0 - RC_CH_VALUE_OFFSET
          #ifdef CHASSIS_USE_KEYBOARD
            + ((pRC->keyboard.key_code & KEY_D) - (pRC->keyboard.key_code & KEY_A))
            * RC_CHASSIS_KEYBOARD_SCALER * (pRC->keyboard.key_code & KEY_SHIFT ? 1.33f : 1.0f)
          #endif
          ) * rc_reversed,

          RX_Y1 = (pRC->rc.channel1 - RC_CH_VALUE_OFFSET
          #ifdef CHASSIS_USE_KEYBOARD
            + ((pRC->keyboard.key_code & KEY_W) - (pRC->keyboard.key_code & KEY_S))
            * RC_CHASSIS_KEYBOARD_SCALER * (pRC->keyboard.key_code & KEY_SHIFT ? 1.33f : 1.0f)
          #endif
          ) * rc_reversed,

          RX_X1 = (pRC->rc.channel2 - RC_CH_VALUE_OFFSET
          #ifdef CHASSIS_USE_KEYBOARD
            +pRC->mouse.x * RC_CHASSIS_MOUSE_SCALER
          #endif
          ) * rc_reversed;

  if(ABS(RX_X2) < THRESHOLD)
    RX_X2 = 0;
  if(ABS(RX_Y1) < THRESHOLD)
    RX_Y1 = 0;
  if(ABS(RX_X1) < THRESHOLD)
    RX_X1 = 0;

  if(RX_X2 > strafe_rc + CHASSIS_XYACCL_LIMIT_RATIO * rc_accl_limit)
    strafe_rc += CHASSIS_XYACCL_LIMIT_RATIO * rc_accl_limit;
  else if(RX_X2 < strafe_rc - CHASSIS_XYACCL_LIMIT_RATIO * rc_accl_limit)
    strafe_rc -= CHASSIS_XYACCL_LIMIT_RATIO * rc_accl_limit;
  else
    strafe_rc = RX_X2 ;

  if(RX_Y1 > drive_rc + rc_accl_limit)
    drive_rc += rc_accl_limit;
  else if(RX_Y1 < drive_rc - rc_accl_limit)
    drive_rc -= rc_accl_limit;
  else
    drive_rc = RX_Y1;

  heading_rc = RX_X1;

  if(rc_speed_limit < rc_speed_limit_sp)
    rc_speed_limit += 0.005f;
  else if(rc_speed_limit > rc_speed_limit_sp)
    rc_speed_limit -= 0.005f;

  strafe_rc = boundOutput(strafe_rc, 660 * rc_speed_limit);
  drive_rc = boundOutput(drive_rc, 660 * rc_speed_limit);
}

#define   CHASSIS_ANGLE_PSC 7.6699e-4 //2*M_PI/0x1FFF
#define   CHASSIS_SPEED_PSC 1.0f/((float)CHASSIS_GEAR_RATIO)
#define   CHASSIS_CONNECTION_ERROR_COUNT 20U
static void chassis_encoderUpdate(void)
{
  uint8_t i;
  for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
  {
    if(chassis._encoders[i].updated)
    {
      //Check validiaty of can connection
      chassis._encoders[i].updated = false;

      //float pos_input = chassis._encoders[i].raw_angle*CHASSIS_ANGLE_PSC;
      float speed_input = chassis._encoders[i].raw_speed*CHASSIS_SPEED_PSC;
      chassis._motors[i]._speed = lpfilter_apply(&lp_speed[i], speed_input);
      chassis._motors[i]._wait_count = 1;
    }
    else
    {
      chassis._motors[i]._wait_count++;
      if(chassis._motors[i]._wait_count > CHASSIS_CONNECTION_ERROR_COUNT)
      {
        LEDY_ON();
        chassis_kill();

        chassis.errorFlag |= CHASSIS0_NOT_CONNECTED << i;
        chassis._motors[i]._wait_count = 1;
      }
    }
  }
  #ifdef CHASSIS_USE_POS_MOTOR
  #endif
}

#define OUTPUT_MAX  30000
static int16_t chassis_controlSpeed(const motorStruct* motor, pi_controller_t* const controller)
{
  float error = motor->speed_sp - motor->_speed;
  controller->error_int += error * controller->ki;
  controller->error_int = boundOutput(controller->error_int, controller->error_int_max);
  float output = error*controller->kp + controller->error_int;
  return (int16_t)(boundOutput(output,OUTPUT_MAX));
}

#define HEADING_PSC VEL_MAX/HEADING_MAX

#ifdef CHASSIS_POWER_LIMIT
  #define TOTAL_OUTPUT_MAX 60000.0f
#endif

static void drive_kinematics(const float strafe_vel, const float drive_vel, const float heading_vel)
{
  chassis._motors[FRONT_RIGHT].speed_sp =
    -strafe_vel - drive_vel - heading_vel * HEADING_PSC;   // CAN ID: 0x201
  chassis._motors[BACK_RIGHT].speed_sp =
    -strafe_vel + drive_vel - heading_vel * HEADING_PSC;       // CAN ID: 0x202
  chassis._motors[FRONT_LEFT].speed_sp =
    strafe_vel + drive_vel - heading_vel * HEADING_PSC;       // CAN ID: 0x203
  chassis._motors[BACK_LEFT].speed_sp =
    strafe_vel - drive_vel - heading_vel * HEADING_PSC;     // CAN ID: 0x204

  uint8_t i;
  int16_t output[4];

  #ifdef CHASSIS_POWER_LIMIT
    float total_output = 0;
  #endif

  if(chassis.state & CHASSIS_SUSPEND)
    can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, 0, 0, 0, 0); //FL,FR,BR,BL

  for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
  {
    output[i] = chassis_controlSpeed(&chassis._motors[i], &motor_vel_controllers[i]);
    #ifdef CHASSIS_POWER_LIMIT
      total_output += ABS(output[i]);
    #endif
  }

  if(chassis.state & CHASSIS_SUSPEND_R)
  {
    output[FRONT_RIGHT] = 0;
    output[BACK_RIGHT] = 0;
  }

  if(chassis.state & CHASSIS_SUSPEND_L)
  {
    output[FRONT_LEFT] = 0;
    output[BACK_LEFT] = 0;
  }

  #ifdef CHASSIS_POWER_LIMIT
    float output_psc;
    if(total_output < TOTAL_OUTPUT_MAX)
      output_psc = 1.0f;
    else
      output_psc = TOTAL_OUTPUT_MAX / total_output;

    for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
      output[i] *= output_psc;
  #endif

  can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID,
      	output[FRONT_RIGHT], output[BACK_RIGHT], output[FRONT_LEFT], output[BACK_LEFT]); //FL,FR,BR,BL
}

/*
 * @brief     Stop chassis auto-driving by setting the speed commands to invalid values
 * @param[in] dir:    set the automation command of chassis direction
 *                    (CHASSIS_STRAFE, CHASSIS_DRIVE, CHASSIS_HEADING)
 * @param[in] cmd:    speed value of automatic driver, set value CHASSIS_DISABLE_AUTO to disable the auto driver
 */
void chassis_autoCmd(const uint8_t dir, const float cmd)
{
  if(dir > 2)
    return;

  chSysLock();
  switch(dir)
  {
    case CHASSIS_STRAFE:
      if(cmd == CHASSIS_DISABLE_AUTO)
      {
        chassis.strafe_cmd = 0.0f;
        chassis.state &= ~(CHASSIS_AUTO_STRAFE);
      }
      else if(cmd != chassis.strafe_cmd)
      {
        chassis.strafe_cmd = cmd;
        chassis.state |= CHASSIS_AUTO_STRAFE;
      }
      break;
    case CHASSIS_DRIVE:
      if(cmd == CHASSIS_DISABLE_AUTO)
      {
        chassis.drive_cmd = 0.0f;
        chassis.state &= ~(CHASSIS_AUTO_DRIVE);
      }
      else if(cmd != chassis.drive_cmd)
      {
        chassis.drive_cmd = cmd;
        chassis.state |= CHASSIS_AUTO_DRIVE;
      }
      break;
    case CHASSIS_HEADING:
      if(cmd == CHASSIS_DISABLE_AUTO)
        chassis.state &= ~(CHASSIS_AUTO_HEADING);
      else if(cmd != chassis.heading_cmd)
      {
        chassis.heading_cmd = cmd;
        chassis.state |= CHASSIS_AUTO_HEADING;
      }
      break;
  }
  chSysUnlock();
}

/*
 * @brief Stop chassis auto-driving by setting the speed commands to invalid values
 */
void chassis_killAutoDriver(void)
{
  chassis.state &= ~(CHASSIS_AUTO);
  chassis.strafe_cmd = 0.0f;
  chassis.drive_cmd = 0.0f;
  chassis.heading_cmd = 0.0f;
}

static THD_WORKING_AREA(chassis_control_wa, 2048);
static THD_FUNCTION(chassis_control, p)
{
  (void)p;
  chRegSetThreadName("chassis controller");

  //reserved for manual override of auto-driving
  uint16_t strafe_count = 0, drive_count = 0, heading_count = 0;
  uint16_t strafe_th = 65535, drive_th = 65535, heading_th = 65535;

  chThdSleepSeconds(2);

  uint32_t tick = chVTGetSystemTimeX();
  while(1)
  {
    tick += US2ST(CHASSIS_UPDATE_PERIOD_US);
    if(tick > chVTGetSystemTimeX())
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

    chassis_encoderUpdate();

    if(chassis.state & CHASSIS_RUNNING)
    {
      float strafe_vel, drive_vel, heading_vel;
      chassis_inputCmd();

      if(
          (chassis.state & CHASSIS_AUTO_STRAFE) &&
          threshold_count(ABS(strafe_rc) > THRESHOLD, strafe_th, &strafe_count)
        )
      {
        chassis.state &= ~(CHASSIS_AUTO_STRAFE); //Manual override of automatic control
        chassis.strafe_cmd = 0.0f;
        strafe_th = 65535;
      }
      if(
          (chassis.state & CHASSIS_AUTO_DRIVE) &&
          threshold_count(ABS(drive_rc) > THRESHOLD, drive_th, &drive_count)
        )
      {
        chassis.state &= ~(CHASSIS_AUTO_DRIVE); //Manual override of automatic control
        chassis.drive_cmd = 0.0f;
        drive_th = 65535;
      }
      if(
          (chassis.state & CHASSIS_AUTO_HEADING) &&
          threshold_count(ABS(heading_rc) > THRESHOLD, heading_th, &heading_count)
        )
      {
        chassis.state &= ~(CHASSIS_AUTO_HEADING); //Manual override of automatic control
        heading_th = 65535;
      }

      if(!strafe_rc && chassis.state & CHASSIS_AUTO_STRAFE)
        strafe_th = 100;

      if(!drive_rc && chassis.state & CHASSIS_AUTO_DRIVE)
        drive_th = 100;

      if(!heading_rc && chassis.state & CHASSIS_AUTO_HEADING)
        heading_th = 100;

      //These are remote control commands, mapped to match min and max CURRENT
      float strafe_input  = mapInput(strafe_rc, -660, 660, -VEL_MAX, VEL_MAX),
            drive_input   = mapInput(drive_rc, -660, 660, -VEL_MAX, VEL_MAX),
            heading_input = mapInput(heading_rc, -660, 660, -HEADING_MAX, HEADING_MAX);

      if(chassis.state & CHASSIS_AUTO_STRAFE)
        strafe_vel = chassis.strafe_cmd;
      else
        strafe_vel = strafe_input;

      if(chassis.state & CHASSIS_AUTO_DRIVE)
        drive_vel = chassis.drive_cmd;
      else
        drive_vel = drive_input;

      if(chassis.state & (CHASSIS_AUTO_HEADING | CHASSIS_HEADING_LOCK))
      {
        if(chassis.state & CHASSIS_AUTO_HEADING)
          heading_sp = chassis.heading_cmd;
        else
          heading_sp -= heading_input * CHASSIS_UPDATE_PERIOD_US / 1e6;

        float error = heading_sp - pIMU->euler_angle[Yaw];

        float drive = drive_vel, strafe = strafe_vel,
              sine = sinf(error), cosine = cosf(error);
        drive_vel = cosine * drive + sine * strafe;
        strafe_vel = cosine * strafe - sine * drive;

        heading_controller.error_int += error * heading_controller.ki;
        bound(&(heading_controller.error_int), heading_controller.error_int_max);
        heading_vel = heading_controller.kp * error + heading_controller.error_int -
                      heading_controller.kd * pIMU->d_euler_angle[Yaw];

        bound(&heading_vel, HEADING_MAX_AUTO);
      }
      else
        heading_vel = heading_input;

      drive_kinematics(strafe_vel, drive_vel, heading_vel);
    }
  }
}

static const FRvelName = "FR_vel";
static const FLvelName = "FL_vel";
static const BLvelName = "BL_vel";
static const BRvelName = "BR_vel";
static const HeadingName = "Heading";

#define MOTOR_VEL_INT_MAX 12000U
void chassis_init(void)
{
  memset(&chassis,0,sizeof(chassisStruct));
  pRC = RC_get();
  pIMU = imu_get();

  uint8_t i;
  params_set(&motor_vel_controllers[FRONT_LEFT], 9,2,FLvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[FRONT_RIGHT], 10,2,FRvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[BACK_LEFT], 11,2,BLvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[BACK_RIGHT], 12,2,BRvelName,subname_PI,PARAM_PUBLIC);

  params_set(&heading_controller, 17,3,HeadingName,subname_PID,PARAM_PUBLIC);

  for (i = 0; i < 4; i++)
  {
    chassis._motors[i].speed_sp = 0.0f;
    lpfilter_init(lp_speed + i, CHASSIS_UPDATE_FREQ, 20);
    motor_vel_controllers[i].error_int = 0.0f;
    motor_vel_controllers[i].error_int_max = MOTOR_VEL_INT_MAX;
  }
  heading_controller.error_int_max = 1.8f;
  chassis.heading_cmd = CHASSIS_DISABLE_AUTO;


  chassis._pGyro = gyro_get();
  chassis._encoders = can_getChassisMotor();
  chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa),
                          NORMALPRIO, chassis_control, NULL);

  chassis.state = CHASSIS_RUNNING;
}
