/*
 * rangefinder.c
 *
 *  Created on: 26 Jan, 2018
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"

#include "rangefinder.h"

static float lastdistance = 0.0f;

void icuwidthcb(ICUDriver *icup) {

  icucnt_t width = icuGetWidthX(icup);
  lastdistance = (SPEED_OF_SOUND * width * M_TO_CM) / (ICU_TIM_FREQ * 2);
}

static PWMConfig pwm4cfg = {
        100000,   /* 1MHz PWM clock frequency.   */
        10000,      /* Initial PWM period 1ms.       */
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};


static ICUConfig icucfg2 = {
  ICU_INPUT_ACTIVE_HIGH,
  ICU_TIM_FREQ,                                    /* 1MHz ICU clock frequency.   */
  icuwidthcb,
  NULL,
  NULL,
  ICU_CHANNEL_1,
  0
};

float rangeFinder_getDistance(void)
{
  return lastdistance;
}

static THD_WORKING_AREA(ultrasonic_thread_wa, 256);
static THD_FUNCTION(ultrasonic_thread,p)
{
  (void)p;
  chRegSetThreadName("Ultrasonic");
  icuStart(&ICUD2, &icucfg2);

  icuStartCapture(&ICUD2);
  icuEnableNotifications(&ICUD2);

  pwmStart(&PWMD4, &pwm4cfg);
  PWMD4.tim->CCR[0] = 1;

  while(true)
  {
      /* Triggering */
      palSetPad(GPIOB, GPIOB_ADC1_IN9);
      chThdSleepMicroseconds(10);
      palClearPad(GPIOB, GPIOB_ADC1_IN9);

      chThdSleepMilliseconds(100);
    }
}

void rangeFinder_init(void){
  chThdCreateStatic(ultrasonic_thread_wa, sizeof(ultrasonic_thread_wa), NORMALPRIO, ultrasonic_thread, NULL);

}
