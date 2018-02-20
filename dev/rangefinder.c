/*
 * rangefinder.c
 *
 *  Created on: 26 Jan, 2018
 *      Author: ASUS
 */
#include "ch.h"
#include "hal.h"

#include "rangefinder.h"

static uint8_t  state          [RANGEFINDER_NUM];
static float    distance_cm    [RANGEFINDER_NUM];
static uint32_t captureDistance[RANGEFINDER_NUM];

static PWMConfig pwm8cfg = {
        200000,   /* 10kHz PWM clock frequency.   */
        10000,      /* Initial PWM period 1ms.       */
        NULL,
        {
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_ACTIVE_LOW, NULL},
                {PWM_OUTPUT_ACTIVE_HIGH, NULL},
                {PWM_OUTPUT_DISABLED, NULL}
        },
        0,
        0
};

#define RANGEFINDER_PSC   SPEED_OF_SOUND * M_TO_CM / (float)(RANGEFINDER_TIM_FREQ * 2)
void gpt5_cb(GPTDriver *gptp)
{
  uint16_t SR = gptp->tim->SR;
  uint16_t CCER = gptp->tim->CCER;

  if(SR & STM32_TIM_SR_CC1IF)
  {
    if(CCER & STM32_TIM_CCER_CC1P)
      distance_cm[RANGEFINDER_INDEX_NOSE] =
        (float)(gptp->tim->CCR[RANGEFINDER_INDEX_NOSE] - captureDistance[RANGEFINDER_INDEX_NOSE])
        * RANGEFINDER_PSC;
    else
      captureDistance[RANGEFINDER_INDEX_NOSE] =
        gptp->tim->CCR[RANGEFINDER_INDEX_NOSE];

    CCER ^= STM32_TIM_CCER_CC1P;
  }
  if(SR & STM32_TIM_SR_CC2IF)
  {
    if(CCER & STM32_TIM_CCER_CC2P)
      distance_cm[RANGEFINDER_INDEX_LEFT_DOGBALL] =
        (float)(gptp->tim->CCR[RANGEFINDER_INDEX_LEFT_DOGBALL] - captureDistance[RANGEFINDER_INDEX_LEFT_DOGBALL])
        * RANGEFINDER_PSC;
    else
      captureDistance[RANGEFINDER_INDEX_LEFT_DOGBALL] =
        gptp->tim->CCR[RANGEFINDER_INDEX_LEFT_DOGBALL];

    CCER ^= STM32_TIM_CCER_CC2P;
  }
  if(SR & STM32_TIM_SR_CC3IF)
  {
    if(CCER & STM32_TIM_CCER_CC3P)
      distance_cm[RANGEFINDER_INDEX_RIGHT_DOGBALL] =
        (float)(gptp->tim->CCR[RANGEFINDER_INDEX_RIGHT_DOGBALL] - captureDistance[RANGEFINDER_INDEX_RIGHT_DOGBALL])
        * RANGEFINDER_PSC;
    else
      captureDistance[RANGEFINDER_INDEX_RIGHT_DOGBALL] =
        gptp->tim->CCR[RANGEFINDER_INDEX_RIGHT_DOGBALL];

    CCER ^= STM32_TIM_CCER_CC3P;
  }

  gptp->tim->CCER = CCER;
}

/*
 * GPT3 configuration.
 */
static const GPTConfig gpt5_cfg = {
  RANGEFINDER_TIM_FREQ,
  gpt5_cb,   /* Timer callback.*/
  0,
  0
};

void rangeFinder_control(const uint8_t index, const bool enable)
{
  if(index >= RANGEFINDER_NUM || state[index] == enable)
    return;

  uint16_t CCER = GPTD5.tim->CCER;
  uint16_t DIER = GPTD5.tim->DIER;

  uint16_t PWM_CCR;

  captureDistance[index] = 0;
  distance_cm[index] = 0.0f;

  switch (index)
  {
    case RANGEFINDER_INDEX_NOSE:
      DIER ^= STM32_TIM_DIER_CC1IE;
      CCER ^= STM32_TIM_CCER_CC1E;
      PWM_CCR = 1;
      break;
    case RANGEFINDER_INDEX_LEFT_DOGBALL:
      DIER ^= STM32_TIM_DIER_CC2IE;
      CCER ^= STM32_TIM_CCER_CC2E;
      PWM_CCR = 9993;
      break;
    case RANGEFINDER_INDEX_RIGHT_DOGBALL:
      DIER ^= STM32_TIM_DIER_CC3IE;
      CCER ^= STM32_TIM_CCER_CC3E;
      PWM_CCR = 1;
      break;
  }

  GPTD5.tim->DIER = DIER;
  GPTD5.tim->CCER = CCER;

  PWMD8.tim->CCR[index] = PWM_CCR;
  state[index] = enable;
}

float rangeFinder_getDistance(const uint8_t index)
{
  if(index > RANGEFINDER_NUM || distance_cm[index] == 0.0f)
    return 999.9f;
  return distance_cm[index];
}

void rangeFinder_init(void)
{
  memset(distance_cm, 0, 4* RANGEFINDER_NUM);
  memset(state,       0,                  4);
  gptStart(&GPTD5, &gpt5_cfg);


  GPTD5.tim->DIER = 0;
  GPTD5.tim->CCER = 0;

  GPTD5.tim->CCMR1 |= STM32_TIM_CCMR1_CC1S(1) | STM32_TIM_CCMR1_CC2S(1)|
                  STM32_TIM_CCMR1_IC1F(3) | STM32_TIM_CCMR1_IC2F(3);
  GPTD5.tim->CCMR2 |= STM32_TIM_CCMR2_CC3S(1) |
                   STM32_TIM_CCMR2_IC3F(3);
  GPTD5.tim->CNT = 10000000U;
  GPTD5.tim->CR1 |= STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;

  pwmStart(&PWMD8, &pwm8cfg);
  PWMD8.tim->CR1 &= ~(STM32_TIM_CR1_CEN);
  PWMD8.tim->CR1 |= STM32_TIM_CR1_CMS(1) | STM32_TIM_CR1_CEN;
}
