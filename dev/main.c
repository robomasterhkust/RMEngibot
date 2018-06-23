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
//Buzzer TIM config
static const PWMConfig pwm12cfg = {
        800000,   /* 1MHz PWM clock frequency.   */
        1000,      /* Initial PWM period 1ms.    width   */
        NULL,
        {
          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
          {PWM_OUTPUT_ACTIVE_HIGH, NULL},
          {PWM_OUTPUT_DISABLED, NULL},
          {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

#define MPU6500_UPDATE_PERIOD_US 1000000U/MPU6500_UPDATE_FREQ
static THD_WORKING_AREA(Attitude_thread_wa, 4096);
static THD_FUNCTION(Attitude_thread, p)
{
  chRegSetThreadName("IMU Attitude Estimator");

  (void)p;

  PIMUStruct pIMU = imu_get();
  //PGyroStruct pGyro = gyro_get();

  static const IMUConfigStruct imu1_conf =
    {&SPID5, MPU6500_ACCEL_SCALE_8G, MPU6500_GYRO_SCALE_1000,
      MPU6500_AXIS_REV_X|MPU6500_AXIS_REV_Y};
  imuInit(pIMU, &imu1_conf);
  imuGetData(pIMU);

  if(pIMU->temperature > 0.0f)
    tempControllerInit();
  else
  {
    system_setErrorFlag();
    pIMU->errorCode |= IMU_TEMP_ERROR;
  }

  while(pIMU->temperature < 60.0f)
  {
    imuGetData(pIMU);
    system_setTempWarningFlag();
    chThdSleepMilliseconds(50);
  }

  pIMU->state = IMU_STATE_READY;
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
      //system_setTempWarningFlag();
      pIMU->errorCode |= IMU_LOSE_FRAME;
    }

    imuGetData(pIMU);
    attitude_update(pIMU);

    if(pIMU->accelerometer_not_calibrated || pIMU->gyroscope_not_calibrated)
    {
      chSysLock();
      chThdSuspendS(&(pIMU->imu_Thd));
      chSysUnlock();
    }
  }
}

#define attitude_init() (chThdCreateStatic(Attitude_thread_wa, sizeof(Attitude_thread_wa), \
                          NORMALPRIO + 5, \
                          Attitude_thread, NULL))

/*
 * Watchdog deadline set to more than one second (LSI=40000 / (64 * 1000)).
 */
static const WDGConfig wdgcfg =
{
  STM32_IWDG_PR_64,
  STM32_IWDG_RL(1000)
};

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

  /* Init sequence 1: central controllers, loggers*/
  shellStart();
  params_init();
  can_processInit();
  system_error_init();
  PWM12_start();

//  extiinit(); //*

  /* Init sequence 2: sensors, comm*/
  //gyro_init();
  rangeFinder_init();
  attitude_init();
  RC_init();
  judgeinit();

  //POWER1_ON();
  
  while(!power_check())
  {
    system_setTempWarningFlag();
    chThdSleepMilliseconds(100);
  }

  /* Init sequence 3: actuators, display*/
  gimbal_init();
  chassis_init();
  lift_init();
  //gripper_init();
  
  island_init();

  wdgStart(&WDGD1, &wdgcfg); //Start the watchdog

  while (true)
  {
    if(!power_failure())
      wdgReset(&WDGD1);
    else
      lift_kill();

    chThdSleepMilliseconds(200);
  }

  return 0;
}

void PWM12_start(void)
{
  PWMD12.tim = STM32_TIM12;
  PWMD12.channels = 2;

  uint32_t psc;
  uint32_t ccer;
  rccEnableTIM12(FALSE);
  rccResetTIM12();

  PWMD12.clock = STM32_TIMCLK1;

  PWMD12.tim->CCMR1 = STM32_TIM_CCMR1_OC1M(6) | STM32_TIM_CCMR1_OC1PE |
                     STM32_TIM_CCMR1_OC2M(6) | STM32_TIM_CCMR1_OC2PE;

  psc = (PWMD12.clock / pwm12cfg.frequency) - 1;

  PWMD12.tim->PSC  = psc;
  PWMD12.tim->ARR  = pwm12cfg.period - 1;
  PWMD12.tim->CR2  = pwm12cfg.cr2;
  PWMD12.period = pwm12cfg.period;

  ccer = 0;
  switch (pwm12cfg.channels[0].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    ccer |= STM32_TIM_CCER_CC1P;
  case PWM_OUTPUT_ACTIVE_HIGH:
    ccer |= STM32_TIM_CCER_CC1E;
  default:
    ;
  }
  switch (pwm12cfg.channels[1].mode & PWM_OUTPUT_MASK) {
  case PWM_OUTPUT_ACTIVE_LOW:
    ccer |= STM32_TIM_CCER_CC2P;
  case PWM_OUTPUT_ACTIVE_HIGH:
    ccer |= STM32_TIM_CCER_CC2E;
  default:
    ;
  }

  PWMD12.tim->CCER  = ccer;
  PWMD12.tim->SR    = 0;

  PWMD12.tim->CR1   = STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;

  PWMD12.state = PWM_READY;
}

void PWM12_setWidth(const uint8_t id, const uint16_t width)
{
  if(id < 2)
    PWMD12.tim->CCR[id] = width;
}

/**
  *   @brief Check whether the 24V power is on
  */
bool power_check(void)
{
  ChassisEncoder_canStruct* can = can_getChassisMotor();

  return can->updated;
}

/**
  *   @brief  Monitor the case of a failure on 24V power, indicating the vehicle being killed
  */
bool power_failure(void)
{
  /*
  uint32_t error = gimbal_get_error();

  return error & (GIMBAL_PITCH_NOT_CONNECTED | GIMBAL_YAW_NOT_CONNECTED) ==
    (GIMBAL_PITCH_NOT_CONNECTED | GIMBAL_YAW_NOT_CONNECTED);
    */
  return false
  ;
}
