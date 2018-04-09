#ifndef _MAIN_H_
#define _MAIN_H_

#include "ch.h"
#include "hal.h"

#include "usbcfg.h"
#include "flash.h"
#include "chprintf.h"
#include "tft_display.h"

#include "math_misc.h"
#include "canBusProcess.h"
#include "dbus.h"
#include "params.h"

#include "mpu6500.h"
#include "ist8310.h"
#include "adis16265.h"
#include "attitude.h"
#include "calibrate_sensor.h"
#include "rangefinder.h"

#include "chassis.h"
#include "lift.h"
#include "island.h"
#include "gripper.h"

#include "exti.h"
#include "imu_temp.h"
#include "sdlog.h"

void shellStart(void);

bool power_check(void);
bool power_failure(void);

#endif
