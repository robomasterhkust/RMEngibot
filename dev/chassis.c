/*
 * chassis.c
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"

#include "dbus.h"
#include "chassis.h"
#include "math_misc.h"

static volatile chassisStruct chassis;
pi_controller_t motor_vel_controllers[CHASSIS_MOTOR_NUM];
lpfilterStruct lp_speed[CHASSIS_MOTOR_NUM];

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

static void chassis_kill(void)
{
  LEDY_ON();
  if(chassis.state == CHASSIS_RUNNING)
  {
    chassis.state = CHASSIS_ERROR;
    can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID, 0, 0, 0, 0);
  }
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

void update_heading(void)
{
  /*TODO THIS WILL MAKE YOU LOSE HEADING IN A COLLISION, add control*/
  chassis.heading_sp = chassis._pGyro->angle;
}

void drive_kinematics(int RX_X2, int RX_Y1, int RX_X1)
{
  // Set dead-zone to 6% range to provide smoother control
  float THRESHOLD = (RC_CH_VALUE_MAX - RC_CH_VALUE_MIN)*3/100;

  // Create "dead-zone" for chassis.drive_sp
  if(ABS(RX_X2 - RC_CH_VALUE_OFFSET) < THRESHOLD)
    RX_X2 = RC_CH_VALUE_OFFSET;

  // Create "dead-zone" for chassis.strafe_sp
  if(ABS(RX_Y1 - RC_CH_VALUE_OFFSET) < THRESHOLD)
    RX_Y1 = RC_CH_VALUE_OFFSET;

  // Create "dead-zone" for chassis.heading_sp
  if(ABS(RX_X1 - RC_CH_VALUE_OFFSET) < THRESHOLD)
    RX_X1 = RC_CH_VALUE_OFFSET;
  else
    chassis.heading_sp = chassis._pGyro->angle;

  // Compute the Heading correction
  float heading_correction = chassis.heading_sp - chassis._pGyro->angle;

  //Remote Control Commands, Mapped to match min and max CURRENT
  chassis.strafe_sp  = (int16_t)mapInput(RX_X2, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -CURRENT_MAX, CURRENT_MAX);
  chassis.drive_sp = (int16_t)mapInput(RX_Y1, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -CURRENT_MAX, CURRENT_MAX);
  chassis.heading_sp = (int16_t)mapInput(RX_X1, RC_CH_VALUE_MIN, RC_CH_VALUE_MAX, -CURRENT_MAX, CURRENT_MAX);
  //heading_correction = HEADING_SCALE * (int16_t)map(heading_correction, HEADING_MIN, HEADING_MAX, CURRENT_MIN, CURRENT_MAX);
  heading_correction = 0.0f;
  // For later coordinate with chassis
  int rotate_feedback = 0;

  chassis._motors[FRONT_RIGHT].speed_sp =
    ((-chassis.strafe_sp - chassis.drive_sp - chassis.heading_sp) + rotate_feedback + heading_correction);   // CAN ID: 0x201
  chassis._motors[BACK_RIGHT].speed_sp =
    ((-chassis.strafe_sp + chassis.drive_sp - chassis.heading_sp) + rotate_feedback + heading_correction);       // CAN ID: 0x202
  chassis._motors[FRONT_LEFT].speed_sp =
    ((chassis.strafe_sp + chassis.drive_sp - chassis.heading_sp) + rotate_feedback + heading_correction);       // CAN ID: 0x203
  chassis._motors[BACK_LEFT].speed_sp =
    ((chassis.strafe_sp - chassis.drive_sp - chassis.heading_sp) + rotate_feedback + heading_correction);     // CAN ID: 0x204

  uint8_t i;
  int16_t output[4];
  for (i = 0; i < CHASSIS_MOTOR_NUM; i++)
    output[i] = chassis_controlSpeed(&chassis._motors[i], &motor_vel_controllers[i]);

  can_motorSetCurrent(CHASSIS_CAN, CHASSIS_CAN_EID,
    		output[FRONT_RIGHT], output[BACK_RIGHT], output[FRONT_LEFT], output[BACK_LEFT]); //FL,FR,BR,BL

}

static THD_WORKING_AREA(chassis_control_wa, 2048);
static THD_FUNCTION(chassis_control, p)
{
  (void)p;
  chRegSetThreadName("chassis controller");

  RC_Ctl_t* pRC = RC_get();
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
    if(chassis.state == CHASSIS_RUNNING)
      drive_kinematics(pRC->rc.channel0, pRC->rc.channel1, pRC->rc.channel2);
  }

}
static const FRvelName = "FR_vel";
static const FLvelName = "FL_vel";
static const BLvelName = "BL_vel";
static const BRvelName = "BR_vel";

#define MOTOR_VEL_INT_MAX 12000U
void chassis_init(void)
{
  memset(&chassis,0,sizeof(chassisStruct));

  chassis.drive_sp = 0.0f;
  chassis.strafe_sp = 0.0f;
  chassis.heading_sp = 0.0f;
  uint8_t i;
  params_set(&motor_vel_controllers[FRONT_LEFT], 9,2,FLvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[FRONT_RIGHT], 10,2,FRvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[BACK_LEFT], 11,2,BLvelName,subname_PI,PARAM_PUBLIC);
  params_set(&motor_vel_controllers[BACK_RIGHT], 12,2,BRvelName,subname_PI,PARAM_PUBLIC);

  for (i = 0; i < 4; i++)
  {
    chassis._motors[i].speed_sp = 0.0f;
    lpfilter_init(lp_speed + i, CHASSIS_UPDATE_FREQ, 20);
    motor_vel_controllers[i].error_int = 0.0f;
    motor_vel_controllers[i].error_int_max = MOTOR_VEL_INT_MAX;
  }
  chassis._pGyro = gyro_get();
  chassis._encoders = can_getChassisMotor();
  chThdCreateStatic(chassis_control_wa, sizeof(chassis_control_wa),
                          NORMALPRIO, chassis_control, NULL);

  chassis.state = CHASSIS_RUNNING;
}
