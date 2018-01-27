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
  {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_250, MPU6500_AXIS_REV_Z};

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
    if(pIMU->inited == 2)
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
} island_state_t;

#define ASCEND_MODE 1
#define DECEND_MODE 2
#define RUSH_MODE 3
#define DOG_ERECT() (PN6_ON())
#define DOG_RELAX() (PN6_OFF())

static void liftWheel_calibration(motorPosStruct* const lift_motor, float* const offset)
{
  //To initialize the lift wheel, a calibration is needed
  //CAUTION!! Moving lift wheel may cause injury, stay back during power up!!

  uint8_t init_state = 0;

  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 5000));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 5000));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 5000));
  chThdSleepMilliseconds(500);
  pwmEnableChannel(&PWMD3, 0, PWM_PERCENTAGE_TO_WIDTH(&PWMD3, 0));
  chThdSleepMilliseconds(500);

  while(init_state < 0x0f)
  {
    if(!LS2_DOWN())
      lift_motor[0].pos_sp+= 0.02f;
    else
    {
      init_state |= 0x01;
      offset[0] = lift_motor[0].pos_sp;
    }
    if(!LS1_DOWN())
      lift_motor[1].pos_sp+= 0.02f;
    else
    {
      init_state |= 0x02;
      offset[1] = lift_motor[1].pos_sp;
    }
    if(!LS0_DOWN())
      lift_motor[2].pos_sp+= 0.02f;
    else
    {
      init_state |= 0x04;
      offset[2] = lift_motor[2].pos_sp;
    }
    if(!LS3_DOWN())
      lift_motor[3].pos_sp+= 0.02f;
    else
    {
      init_state |= 0x08;
      offset[3] = lift_motor[3].pos_sp;
    }

    chThdSleepMilliseconds(2);
  }
}

#define ISLAND_UPDATE_PERIOD_MS 10
static THD_WORKING_AREA(Island_thread_wa, 1024);
static THD_FUNCTION(Island_thread, p)
{
  chRegSetThreadName("Island climbing");

  (void)p;

  uint32_t tick = chVTGetSystemTimeX();
  param_t pos_sp[4];

  island_state_t state = STATE_GROUND;

  const char name[] = "pos_sp";
  const char subName[] = "BL BR FR FL";
  params_set(pos_sp,18,4,name,subName,PARAM_PUBLIC);
  DOG_RELAX();

  motorPosStruct* lift_motor = lift_get();
  bool transition = false;
  RC_Ctl_t* rc = RC_get();

  float offset[4];
  liftWheel_calibration(lift_motor, offset);

  while(true)
  {
    tick += MS2ST(ISLAND_UPDATE_PERIOD_MS);
    if(chVTGetSystemTimeX() < tick)
      chThdSleepUntil(tick);
    else
    {
      tick = chVTGetSystemTimeX();
    }

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
            else if(state == STATE_RUSHDOWN_1)
              state =  STATE_GROUND;
            transition = false;
          }
          break;
      }
    }

    switch(state)
    {

      case STATE_ONFOOT:
        lift_motor[0].pos_sp = offset[0] - pos_sp[0];
        lift_motor[1].pos_sp = offset[1] - pos_sp[0];
        lift_motor[2].pos_sp = offset[2] - pos_sp[0];
        lift_motor[3].pos_sp = offset[3] - pos_sp[0];
        DOG_ERECT();
        break;
      case STATE_ROLLER_IN:
        lift_motor[3].pos_sp = offset[3] - pos_sp[1];
        lift_motor[2].pos_sp = offset[2] - pos_sp[1];
        lift_motor[0].pos_sp = offset[0] - pos_sp[2];
        lift_motor[1].pos_sp = offset[1] - pos_sp[2];
        break;
      case STATE_ROLLER_IN_2:
        lift_motor[3].pos_sp = offset[3] - pos_sp[1] - 4.0f;
        lift_motor[2].pos_sp = offset[2] - pos_sp[1] - 4.0f;
        lift_motor[0].pos_sp = offset[0] - pos_sp[2];
        lift_motor[1].pos_sp = offset[1] - pos_sp[2];
        break;
      case STATE_GROUND:
        lift_motor[0].pos_sp = offset[0] - 47.0f;  //5.0f
        lift_motor[1].pos_sp = offset[1] - 47.0f;
        lift_motor[2].pos_sp = offset[2] - 47.0f;
        lift_motor[3].pos_sp = offset[3] - 47.0f;
        if(rc->rc.s2 == DECEND_MODE || rc->rc.s2 == ASCEND_MODE)
          DOG_ERECT();
        else
          DOG_RELAX();
        break;
      case STATE_RUSHDOWN_1:
        lift_motor[3].pos_sp = offset[0] - pos_sp[3];
        lift_motor[2].pos_sp = offset[1] - pos_sp[3];
        lift_motor[0].pos_sp = offset[2] - pos_sp[2];
        lift_motor[1].pos_sp = offset[3] - pos_sp[2];
        break;
/*
      case STATE_ONFOOT:
        lift_motor[0].pos_sp = 41.0f;
        lift_motor[1].pos_sp = 41.0f;
        lift_motor[2].pos_sp = 41.0f;
        lift_motor[3].pos_sp = 41.0f;
        DOG_ERECT();
        break;
      case STATE_ROLLER_IN:
        lift_motor[3].pos_sp = 42.5f;
        lift_motor[2].pos_sp = 42.5f;
        lift_motor[0].pos_sp = 0.8f;
        lift_motor[1].pos_sp = 0.8f;
        break;
      case STATE_GROUND:
        lift_motor[0].pos_sp = 5.0f;
        lift_motor[1].pos_sp = 5.0f;
        lift_motor[2].pos_sp = 5.0f;
        lift_motor[3].pos_sp = 5.0f;
        if(rc->rc.s2 == DECEND_MODE || rc->rc.s2 == ASCEND_MODE)
          DOG_ERECT();
        else
          DOG_RELAX();
        break;
      case STATE_RUSHDOWN_1:
        lift_motor[3].pos_sp = 38.0f;
        lift_motor[2].pos_sp = 38.0f;
        lift_motor[0].pos_sp = 5.0f;
        lift_motor[1].pos_sp = 5.0f;
        break;*/
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


  palSetPad(GPIOE, GPIOE_LED_R);
  palSetPad(GPIOF, GPIOF_LED_G);
  palClearPad(GPIOA, GPIOA_LED_Y);
  palClearPad(GPIOA, GPIOA_LED_B);


  shellStart();
  params_init();
  can_processInit();
  RC_init();
  gimbal_init();

//  gimbal_sys_iden_init(); //*
  pwm_shooter_init(); // *
  extiinit(); //*
  tempControllerInit(); //*
  chassis_init();
  pGyro = gyro_init();

  //pwm12init();
  sdlog_init();

  //tft_init(TFT_HORIZONTAL, CYAN, YELLOW, BLACK);
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

    chThdSleepMilliseconds(500);

  }

  return 0;
}
