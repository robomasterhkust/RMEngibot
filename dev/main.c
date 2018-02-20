/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/
#include "main.h"

static BaseSequentialStream* chp = (BaseSequentialStream*)&SDU1;
static const IMUConfigStruct imu1_conf =
  {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_250, MPU6500_AXIS_REV_X|MPU6500_AXIS_REV_Y};

static const magConfigStruct mag1_conf =
  {IST8310_ADDR_FLOATING, 200, IST8310_AXIS_REV_NO};

PIMUStruct pIMU;
PGyroStruct pGyro;

#define MPU6500_UPDATE_PERIOD_US 1000000U/MPU6500_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);
static THD_FUNCTION(Attitude_thread, p)
{
  chRegSetThreadName("IMU Attitude Estimator");

  (void)p;

  imuInit(pIMU, &imu1_conf);
  ist8310_init(&mag1_conf);

  chThdSleepSeconds(3);
  attitude_imu_init(pIMU);

  uint32_t tick = chVTGetSystemTimeX();

  while(true)
  {
    tick += US2ST(MPU6500_UPDATE_PERIOD_US);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
      pIMU->errorCode |= IMU_LOSE_FRAME;
    }

    imuGetData(pIMU);
    ist8310_update();

    attitude_update(pIMU);

    if(pIMU->accelerometer_not_calibrated || pIMU->gyroscope_not_calibrated)
    {
      chSysLock();
      chThdSuspendS(&(pIMU->imu_Thd));
      chSysUnlock();
    }
  }
}

typedef enum {
  STATE_GROUND,
  STATE_ONFOOT,
  STATE_ROLLER_IN,
  STATE_ROLLER_IN_2,
  STATE_RUSHDOWN_1,
  STATE_RUSHDOWN_2
} island_state_t;

#define ASCEND_MODE 1
#define DECEND_MODE 3
#define RUSH_MODE 2
#define DOG_ERECT() (PN2_ON())
#define DOG_RELAX() (PN2_OFF())

#define ISLAND_UPDATE_PERIOD_MS 5
static THD_WORKING_AREA(Island_thread_wa, 1024);
static THD_FUNCTION(Island_thread, p)
{
  chRegSetThreadName("Island climbing");

  (void)p;
  param_t pos_sp[7];
  param_t threshold[7];

  island_state_t state = STATE_GROUND;

  const char namePos[] = "pos_sp";
  const char subNamePos[] = "1 2 3 4 5 6 7";

  const char nameTH[] = "island_th";
  const char subNameTH[] = "Up_Pitch Dn_Pitch1 Dn_Pitch2 Dn_Pitch3 Up_RF1 Up_RF2 7";
  params_set(pos_sp,18,7,namePos,subNamePos,PARAM_PUBLIC);
  params_set(threshold,19,7,nameTH,subNameTH,PARAM_PUBLIC);
  DOG_RELAX();

  bool transition = false;
  RC_Ctl_t* rc = RC_get();

  float pos_cmd = 0.0f;

  uint16_t count = 0;
  uint16_t on_ground = 0;

  chThdSleepSeconds(2);
  if(!lift_getError())
    lift_calibrate();

  uint32_t tick = chVTGetSystemTimeX();
  while(true)
  {
    tick += MS2ST(ISLAND_UPDATE_PERIOD_MS);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }
    /* State controller for island climbing machine*/
    if(rc->rc.s2 == ASCEND_MODE || rc->rc.s2 == DECEND_MODE)
    {
      switch(rc->rc.s1)
      {
        case 1:
          if(transition)
          {
            if(state == STATE_ROLLER_IN_2)
              state = STATE_GROUND;
            else
              state++;
            transition = false;
            pos_cmd = 0.0f;
          }
          break;
        case 3:
          transition = true;
          break;
        case 2:
          if(transition)
          {
            if(state == STATE_GROUND && rc->rc.s2 == DECEND_MODE)
              state = STATE_ROLLER_IN_2;
            else if(state != STATE_GROUND)
              state--;
            transition = false;
            pos_cmd = 0.0f;
          }
          break;
      }
    }
    else if(rc->rc.s2 == RUSH_MODE)
    {
      DOG_RELAX();
      switch(rc->rc.s1)
      {
        case 1:
          if(transition)
          {
            if(state == STATE_RUSHDOWN_1)
              state = STATE_GROUND;
            transition = false;
            pos_cmd = 0.0f;
          }
          break;
        case 3:
          transition = true;
          break;
        case 2:
          if(transition)
          {
            if(state == STATE_GROUND)
              state = STATE_RUSHDOWN_1;
            transition = false;
            pos_cmd = 0.0f;
          }
          break;
      }
    }
    /**/


    int16_t input = rc->rc.channel3 - RC_CH_VALUE_OFFSET;
    if(input > 400)
      pos_cmd += 0.1f;
    else if(input > 100)
      pos_cmd += 0.025f;
    else if(input < -400)
      pos_cmd -= 0.1f;
    else if(input < -100)
      pos_cmd -= 0.025f;

    switch(state)
    {

      case STATE_ONFOOT:
        DOG_ERECT();
        rangeFinder_control(RANGEFINDER_INDEX_NOSE, ENABLE);

        lift_changePos(pos_sp[0] - pos_cmd, pos_sp[0] - pos_cmd ,
                      pos_sp[0] - pos_cmd, pos_sp[0] - pos_cmd);

        if(rc->rc.s2 == ASCEND_MODE && lift_inPosition() &&
          threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_NOSE) < threshold[4], 24, &count))
        {
          count = 0;
          state++;
        }

        break;
      case STATE_ROLLER_IN:
        DOG_ERECT();
        rangeFinder_control(RANGEFINDER_INDEX_NOSE, DISABLE);

        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                      pos_sp[1] - pos_cmd, pos_sp[1] - pos_cmd);

        if(rc->rc.s2 == ASCEND_MODE &&
          lift_inPosition() && threshold_count(pIMU->euler_angle[Pitch] > threshold[0], 2, &count))
        {
          count = 0;
          state++;
        }

        break;
      case STATE_ROLLER_IN_2:
        DOG_ERECT();
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_DOGBALL,  ENABLE);
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_DOGBALL, ENABLE);

        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                      pos_sp[1] + 4.15f - pos_cmd, pos_sp[1] + 4.15f - pos_cmd);

        if(rc->rc.s2 == DECEND_MODE &&
          lift_inPosition() && threshold_count(pIMU->euler_angle[Pitch] < threshold[0], 2, &count))
        {
          count = 0;
          state--;
        }
        break;
      case STATE_GROUND:
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_DOGBALL,  DISABLE);
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_DOGBALL, DISABLE);

        lift_changePos(47.0f - pos_cmd, 47.0f - pos_cmd ,
                      47.0f - pos_cmd, 47.0f - pos_cmd);

        if(rc->rc.s2 == DECEND_MODE || rc->rc.s2 == ASCEND_MODE)
          DOG_ERECT();
        else
          DOG_RELAX();
        break;
      case STATE_RUSHDOWN_1:
        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                    pos_sp[4] - pos_cmd, pos_sp[4] - pos_cmd);

        if(lift_inPosition() && threshold_count(pIMU->euler_angle[Pitch] > threshold[1], 2, &count))
        {
          count = 0;
          state++;
        }
        on_ground = 0;

        break;
      case STATE_RUSHDOWN_2:
      lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                  pos_sp[3] - pos_cmd, pos_sp[3] - pos_cmd);

        //if(pIMU->euler_angle[Pitch] > 0.1f)
        if(pIMU->euler_angle[Pitch] > threshold[2])
          on_ground++;

        if(on_ground >= 2 && threshold_count(pIMU->euler_angle[Pitch] < threshold[3], 2, &count))
        {
          count = 0;
          state = STATE_GROUND;
        }

        break;
    }
  }
}

/*
 * Application entry point.
 */
int main(void) {

  /*
   * System initializations.
   * - HAL initialization, this also initializes the configured device drivers
   *   and performs the board-specific initializations.
   * - Kernel initialization, the main() function becomes a thread and the
   *   RTOS is active.
   */
  halInit();
  chSysInit();

  shellStart();
  params_init();
  can_processInit();
  RC_init();

  extiinit(); //*
  tempControllerInit(); //*
  chassis_init();
  pGyro = gyro_init();
  rangeFinder_init();

  sdlog_init();
  lift_init();
  pIMU = imu_get(); //*

  chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa),
  NORMALPRIO + 5,
                    Attitude_thread, NULL); //*

  chThdCreateStatic(Island_thread_wa, sizeof(Island_thread_wa),
  NORMALPRIO + 5,
                    Island_thread, NULL); //*

  while (true)
  {
    chThdSleepSeconds(10);
  }

  return 0;
}
