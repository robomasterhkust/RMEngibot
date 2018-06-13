#include "ch.h"
#include "hal.h"

#include "dbus.h"

#include "system_error.h"

static system_error_t system_error = 0;
static systime_t system_warning_start_time;

/*
 *====================LED INDICATOR========
 *   o-o-o-o-o-o-o-         DBUS Not CONNECTED
 *   o------o------         DBUS RC locked
 *   o-o----o-o----         DBUS RC unlocked
 *   oooooooooooooo         System failed
 *   --------------         System failed
 *   RED led blinking       ERROR occured, requires a reboot
 *   YELLOW led blinking    Warning occured,
 *   GREED led blinking     System normal
 *=========================================
 */

void system_setErrorFlag(void)
{
  system_error |= SYSTEM_ERROR;
}

void system_setWarningFlag(void)
{
  system_error |= SYSTEM_WARNING;
}

void system_setTempWarningFlag(void)
{
  system_warning_start_time = chVTGetSystemTimeX();
  system_error |= SYSTEM_TEMP_WARNING;
}

void system_clearWarningFlag(void)
{
  system_error &= ~(SYSTEM_TEMP_WARNING | SYSTEM_WARNING);
}

static THD_WORKING_AREA(system_error_wa, 128);
static THD_FUNCTION(system_error_thd, p)
{
  (void) p;
  chRegSetThreadName("System status indicator");

  LEDG_OFF();
  LEDR_OFF();

  rc_state_t rc_state;
  bool led_on; //Control the toggling of LED

  uint32_t count = 0;
  while(true)
  {
    //Control the flashing of green LED // Shift to Error.c
    rc_state = RC_getState();
    if(!(count % 5))
    {
      uint32_t blink_count = count / 5;

      if(!(blink_count % 10))
      {
        led_on = false;
        LEDG_OFF();
        LEDR_OFF();
      }

      if(!rc_state ||
          (
           #ifdef RC_SAFE_LOCK
             ((rc_state == RC_LOCKED || rc_state == RC_UNLOCKING) && (blink_count % 10 < 2)) ||
             (rc_state == RC_UNLOCKED && (blink_count % 10 < 4))
           #else
             (blink_count % 10 < 4)
           #endif
          )
        )
        {
          led_on = !led_on;
          if(!(system_error & SYSTEM_ERROR))
            led_on ? LEDG_ON() : LEDG_OFF();
          else
            LEDG_OFF();

          if(system_error)
            led_on ? LEDR_ON() : LEDR_OFF();
          else
            LEDR_OFF();
        }
    }

    count++;
    if(chVTGetSystemTimeX() > system_warning_start_time + S2ST(SYSTEM_TEMP_WARNING_DURATION))
      system_error &= ~(SYSTEM_TEMP_WARNING);

    chThdSleepMilliseconds(25);
  }
}

void system_error_init(void)
{
  chThdCreateStatic(system_error_wa, sizeof(system_error_wa),
                    NORMALPRIO - 7, system_error_thd, NULL);
}
