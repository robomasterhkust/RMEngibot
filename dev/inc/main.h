#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"
#include "system_error.h"

#include "math_misc.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"

#include "attitude.h"
#include "calibrate_sensor.h"
#include "rangefinder.h"

#include "chassis.h"
#include "lift.h"
#include "island.h"
#include "gripper.h"

#include "exti.h"
#include "judge.h"
//#include "imu_temp.h"
#include "sdlog.h"

void shellStart(void);

bool power_check(void);
bool power_failure(void);

void PWM12_start(void);
void PWM12_setWidth(const uint8_t id, const uint16_t width);

#endif
