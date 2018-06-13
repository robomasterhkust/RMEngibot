/*
 * chassis.h
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_CHASSIS_H_
#define INC_CHASSIS_H_

#include "adis16470.h"
#include "canBusProcess.h"

#define CHASSIS_CAN  &CAND1         // Later should be CAND2
#define CHASSIS_CAN_EID  0x200

#define CHASSIS_UPDATE_FREQ 1000
#define CHASSIS_UPDATE_PERIOD_US 1000000/CHASSIS_UPDATE_FREQ
#define CHASSIS_HEADING_CONTROL

/*
 *  NOTE: We use different accl limit for strafe and drive,
 *  due to different dynamic property of the chassis
 *  VALUE = STRAFE_LIMIT / DRIVE_LIMIT
 */
#define CHASSIS_XYACCL_LIMIT_RATIO 0.75f

//#define CHASSIS_POWER_LIMIT
//#define CHASSIS_USE_KEYBOARD

// DBUS MACRO

#ifdef CHASSIS_USE_KEYBOARD
  #define RC_CHASSIS_MOUSE_SCALER 22
  #define RC_CHASSIS_KEYBOARD_SCALER  440
#endif

#define VEL_MAX    ((int16_t) 417)              //
#define VEL_MIN    ((int16_t)-417)              //
#define HEADING_MAX    6.0f
#define HEADING_MIN    -6.0f
#define HEADING_MAX_AUTO    2.0f
#define HEADING_MIN_AUTO    -2.0f
#define CHASSIS_GEAR_RATIO    19U

#define ABS(x)     ( ((x) > 0) ? (x) : (-(x)) ) //return abs value of x

#define CHASSIS_USE_POS_MOTOR
#define CHASSIS_DISABLE_AUTO      200000.0f

typedef enum {
  CHASSIS_STRAFE =  0,
  CHASSIS_DRIVE =   1,
  CHASSIS_HEADING = 2
} chassis_dir_t;

typedef enum {
  CHASSIS_UNINIT         = 0,
  CHASSIS_RUNNING        = 1,
  CHASSIS_AUTO_STRAFE    = 2,
  CHASSIS_AUTO_DRIVE     = 4,
  CHASSIS_AUTO_HEADING   = 8,
  CHASSIS_AUTO           = 14,
  CHASSIS_HEADING_LOCK   = 16,
  CHASSIS_SUSPEND_L      = 32,
  CHASSIS_SUSPEND_R      = 64,
  CHASSIS_SUSPEND        = 96,
  CHASSIS_ERROR          = 128
} chassis_state_t;

typedef enum {
  CHASSIS0_NOT_CONNECTED = 1 << 0,
  CHASSIS1_NOT_CONNECTED = 1 << 1,
  CHASSIS2_NOT_CONNECTED = 1 << 2,
  CHASSIS3_NOT_CONNECTED = 1 << 3,
  CHASSIS_NOT_CONNECTED = 0x0000ffff
} chassis_error_t;

typedef struct{
  float speed_sp;
  float _speed;
  uint8_t _wait_count;
} motorStruct;

typedef struct{
  float speed_sp;
  float _speed;
  float pos_sp;
  float _pos;

  int32_t rev;
  float _prev;

  uint8_t _wait_count;
  uint8_t in_position;
} motorPosStruct;

typedef struct{
  motorStruct _motors[CHASSIS_MOTOR_NUM];
  chassis_state_t state;
  chassis_error_t errorFlag;

  #ifdef CHASSIS_USE_POS_MOTOR
    motorPosStruct pos_motors[4];
  #endif

  float heading_cmd;
  float drive_cmd;
  float strafe_cmd; //These are external commands given from Auto-driving controllers
                    //Which can be manually overriden

  ChassisEncoder_canStruct* _encoders;
} chassisStruct;

// MATH definition

void chassis_killAutoDriver(void);
void chassis_tempSuspend(const uint8_t cmd);
void chassis_setSpeedLimit(const float speed_limit);
void chassis_setAcclLimit(const float accl_limit);
void chassis_reverseCmd(const uint8_t cmd);
void chassis_headingLockCmd(const uint8_t cmd);
void chassis_autoCmd(const uint8_t dir, const float cmd);

chassisStruct* chassis_get(void);
uint32_t chassis_getError(void);

void chassis_init(void);
#endif /* INC_CHASSIS_H_ */
