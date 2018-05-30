#ifndef GIMBAL_H_
#define GIMBAL_H_

#include "canBusProcess.h"
#include "dbus.h"
#define GIMBAL_CAN  &CAND1  
#define GIMBAL_CAN_EID  0x1FF

void gimbal_init(void);

#endif