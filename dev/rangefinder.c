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

#define RANGEFINDER_PSC  S_TO_MS / (float)(RANGEFINDER_TIM_FREQ) * RATIO /MM_TO_CM
static void gpt5_cb(GPTDriver *gptp)
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
  if(SR & STM32_TIM_SR_CC4IF)
  {
    if(CCER & STM32_TIM_CCER_CC4P)
      distance_cm[RANGEFINDER_INDEX_LEFT_BUM] =
        (float)(gptp->tim->CCR[RANGEFINDER_INDEX_LEFT_BUM] - captureDistance[RANGEFINDER_INDEX_LEFT_BUM])
        * RANGEFINDER_PSC;
    else
      captureDistance[RANGEFINDER_INDEX_LEFT_BUM] =
        gptp->tim->CCR[RANGEFINDER_INDEX_LEFT_BUM];

    CCER ^= STM32_TIM_CCER_CC4P;
  }

  gptp->tim->CCER = CCER;
}

static void gpt4_cb(GPTDriver *gptp)
{
  uint16_t SR = gptp->tim->SR;
  uint16_t CCER = gptp->tim->CCER;

  //LEDG_TOGGLE();

  if(SR & STM32_TIM_SR_CC1IF)
  {
    if(CCER & STM32_TIM_CCER_CC1P){
      if(gptp->tim->CCR[0] > captureDistance[RANGEFINDER_INDEX_RIGHT_BUM]){
        distance_cm[RANGEFINDER_INDEX_RIGHT_BUM] =
          (float)(gptp->tim->CCR[0] - captureDistance[RANGEFINDER_INDEX_RIGHT_BUM])
          * RANGEFINDER_PSC;
      }
      else{
        distance_cm[RANGEFINDER_INDEX_RIGHT_BUM] =
        (float)(65535-captureDistance[RANGEFINDER_INDEX_RIGHT_BUM] + gptp->tim->CCR[0])* RANGEFINDER_PSC;
      }
    }

    else
      captureDistance[RANGEFINDER_INDEX_RIGHT_BUM] =
        gptp->tim->CCR[0];

    CCER ^= STM32_TIM_CCER_CC1P;
  }

  gptp->tim->CCER = CCER;
}

/**
 * @brief   TIM8 interrupt handler.
 *
 * @isr
 */
// CH_IRQ_HANDLER(STM32_TIM8_CC_HANDLER) {

//   CH_IRQ_PROLOGUE();

//   gpt8_cb(&GPTD8);
//   GPTD8.tim->SR = 0;

//   CH_IRQ_EPILOGUE();
// }

/*
 * GPT5 configuration.
 */
static const GPTConfig gpt5_cfg = {
  RANGEFINDER_TIM_FREQ,
  gpt5_cb,   /* Timer callback.*/
  0,
  0
};

/*
 * GPT4 configuration.
 */
static const GPTConfig gpt4_cfg = {
  RANGEFINDER_TIM_FREQ,
  gpt4_cb,   /* Timer callback.*/
  0,
  0
};

void rangeFinder_control(const uint8_t index, const bool enable)
{
  if(index >= RANGEFINDER_NUM || state[index] == enable)
    return;

  uint16_t CCER, DIER;
  GPTDriver* gpt;

  if(index < RANGEFINDER_INDEX_RIGHT_BUM)
    gpt = &GPTD5;
  else
    gpt = &GPTD4;

  CCER = gpt->tim->CCER;
  DIER = gpt->tim->DIER;

  captureDistance[index] = 0;
  distance_cm[index] = 0.0f;

  switch (index)
  {
    case RANGEFINDER_INDEX_NOSE:
      DIER ^= STM32_TIM_DIER_CC1IE;
      CCER ^= STM32_TIM_CCER_CC1E;
      break;
    case RANGEFINDER_INDEX_LEFT_DOGBALL:
      DIER ^= STM32_TIM_DIER_CC2IE;
      CCER ^= STM32_TIM_CCER_CC2E;
      break;
    case RANGEFINDER_INDEX_RIGHT_DOGBALL:
      DIER ^= STM32_TIM_DIER_CC3IE;
      CCER ^= STM32_TIM_CCER_CC3E;
      break;
    case RANGEFINDER_INDEX_LEFT_BUM:
      DIER ^= STM32_TIM_DIER_CC4IE;
      CCER ^= STM32_TIM_CCER_CC4E;
      break;
    case RANGEFINDER_INDEX_RIGHT_BUM:
      DIER ^= STM32_TIM_DIER_CC1IE;
      CCER ^= STM32_TIM_CCER_CC1E;
      break;
  }

  gpt->tim->DIER = DIER;
  gpt->tim->CCER = CCER;

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
  memset(state,       0,    RANGEFINDER_NUM);

  //  GPTD5 setup
  gptStart(&GPTD5, &gpt5_cfg);
  GPTD5.tim->DIER = 0;
  GPTD5.tim->CCER = 0;
  GPTD5.tim->CCMR1 |= STM32_TIM_CCMR1_CC1S(1) | STM32_TIM_CCMR1_CC2S(1)|
                  STM32_TIM_CCMR1_IC1F(3) | STM32_TIM_CCMR1_IC2F(3);
  GPTD5.tim->CCMR2 |= STM32_TIM_CCMR2_CC3S(1) |STM32_TIM_CCMR2_IC3F(3) |
                  STM32_TIM_CCMR2_CC4S(1) |STM32_TIM_CCMR2_IC4F(3);
  GPTD5.tim->CNT = 10000000U;
  GPTD5.tim->CR1 |= STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;

  //  GPTD4 setup
  gptStart(&GPTD4, &gpt4_cfg);
  // nvicDisableVector(STM32_TIM8_UP_NUMBER); //TIM8 has two interrupt vectors, we are using the CC vector
  // nvicEnableVector(STM32_TIM8_CC_NUMBER, STM32_GPT_TIM8_IRQ_PRIORITY);
  GPTD4.tim->DIER = 0;
  GPTD4.tim->CCER = 0;
  GPTD4.tim->CCMR1 |= STM32_TIM_CCMR1_CC1S(1) | STM32_TIM_CCMR1_IC1F(3);
  GPTD4.tim->CNT = 0U;
  GPTD4.tim->CR1 |= STM32_TIM_CR1_ARPE | STM32_TIM_CR1_CEN;
}
