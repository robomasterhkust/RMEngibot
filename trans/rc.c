#include "rc.h"

#define  RC_GPT GPTD2
#define  RC_CHANNEL_NUM 3U

//Use of channel2-4 is supported, due to limited number of timers
static rc_channel_t rc_channel[RC_CHANNEL_NUM] = {1500U,1500U,1500U};

/*
 * GPT3 configuration.
 */
static const GPTConfig RC_cfg = {
  1000000,    /* 10kHz timer clock.*/
  NULL,   /* Timer callback.*/
  0,
  0
};

static void rc_update(void)
{
  if(RC_GPT.tim->CCR[0] > 900U && RC_GPT.tim->CCR[0] < 2100U)
  {
    rc_channel[0] = RC_GPT.tim->CCR[0];
    rc_channel[1] = RC_GPT.tim->CCR[2];
    rc_channel[2] = RC_GPT.tim->CCR[3];
  }
  else
    rc_channel[0] = rc_channel[1] = rc_channel[2] = 1500U;
}

static THD_WORKING_AREA(RC_thread_wa, 64);
static THD_FUNCTION(RC_thread, p)
{
  (void)p;
  chRegSetThreadName("Radio control");

  while (true)
  {
    rc_update();
    chThdSleepMilliseconds(200);
  }
}

rc_channel_t* rc_getChannels(void)
{
  return rc_channel;
}

rc_channel_t* rc_init(void)
{
  gptStart(&RC_GPT, &RC_cfg);

  RC_GPT.tim->CR1    = 0;                  /* Timer disabled.              */
  RC_GPT.tim->CCR[0] = 0;                  /* Comparator 1 disabled.       */
  RC_GPT.tim->CCR[1] = 0;                  /* Comparator 2 disabled.       */
  RC_GPT.tim->CCR[2] = 0;                  /* Comparator 2 disabled.       */
  RC_GPT.tim->CCR[3] = 0;                  /* Comparator 2 disabled.       */
  RC_GPT.tim->CNT    = 0;                  /* Counter reset to zero.       */

  //Use Channel1 as trigger
  RC_GPT.tim->CCMR1 |= STM32_TIM_CCMR1_CC1S(2) | STM32_TIM_CCMR1_CC2S(1)|
                  STM32_TIM_CCMR1_IC1F(4) | STM32_TIM_CCMR1_IC2F(4);

  RC_GPT.tim->CCMR2 |= STM32_TIM_CCMR2_CC3S(1) | STM32_TIM_CCMR2_CC4S(1) |
                   STM32_TIM_CCMR2_IC3F(4) | STM32_TIM_CCMR2_IC4F(4);

  RC_GPT.tim->CCER = STM32_TIM_CCER_CC1E | STM32_TIM_CCER_CC1P |
                    STM32_TIM_CCER_CC2E |
                    STM32_TIM_CCER_CC3E | STM32_TIM_CCER_CC3P |
                    STM32_TIM_CCER_CC4E | STM32_TIM_CCER_CC4P;


  RC_GPT.tim->SMCR |= STM32_TIM_SMCR_SMS(4) | STM32_TIM_SMCR_TS(6);


  RC_GPT.tim->CR1 |= STM32_TIM_CR1_CEN;

  chThdCreateStatic(RC_thread_wa, sizeof(RC_thread_wa),
  NORMALPRIO - 5,
                    RC_thread, NULL);

  return rc_channel;
}
