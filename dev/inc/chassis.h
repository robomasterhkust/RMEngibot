/*
 * chassis.h
 *
 *  Created on: 10 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_CHASSIS_H_
#define INC_CHASSIS_H_

#include "adis16265.h"
#include "canBusProcess.h"

#define CHASSIS_CAN  &CAND1         // Later should be CAND2
#define CHASSIS_CAN_EID  0x200

#define CHASSIS_UPDATE_FREQ 500
#define CHASSIS_UPDATE_PERIOD_US 1000000/CHASSIS_UPDATE_FREQ

// DBUS MACRO

#define CURRENT_MAX    ((int16_t) 277)              //
#define CURRENT_MIN    ((int16_t)-277)              //
#define CHASSIS_GEAR_RATIO    27U

#define HEADING_MIN     ((float) -3.14159) // - pi
#define HEADING_MAX     ((float) 3.14159)   // pi
#define HEADING_SCALE   ((uint16_t) 100)

#define ABS(x)     ( ((x) > 0) ? (x) : (-(x)) ) //return abs value of x

#define CHASSIS_USE_POS_MOTOR

typedef enum {
  CHASSIS_UNINIT = 0,
  CHASSIS_RUNNING,
  CHASSIS_ERROR
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

  float heading_sp;
  float drive_sp;
  float strafe_sp;

  ChassisEncoder_canStruct* _encoders;
  PGyroStruct _pGyro;
} chassisStruct;

// MATH definition

chassisStruct* chassis_get(void);
uint32_t chassis_getError(void);

void chassis_init(void);
#endif /* INC_CHASSIS_H_ */
