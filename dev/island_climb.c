#include "ch.h"
#include "hal.h"

#include "island.h"
#include "mpu6500.h"
#include "dbus.h"
#include "rangefinder.h"
#include "lift.h"
#include "math_misc.h"

static PIMUStruct pIMU;
static island_state_t island_state = STATE_STAIR_0;
static robot_state_t robot_state = STATE_GROUND;
static float pos_cmd = 0.0f;
static param_t pos_sp[7];
static param_t threshold[7];

static float start_yaw;

static void island_robotSetState(robot_state_t state)
{
  if(state < 0 || state > ROBOT_STATE_NUM)
    return;
  robot_state = state;
  pos_cmd = 0.0f;
}

static THD_WORKING_AREA(Island_thread_wa, 1024);
static THD_FUNCTION(Island_thread, p)
{
  chRegSetThreadName("Island climbing");

  (void)p;

  RC_Ctl_t* rc = RC_get();

  bool s1_reset = false;

  uint16_t count = 0, count_left = 0, count_right = 0, count_onGround = 0,count_back_left=0,count_back_right =0;
  const float roller_in = 9.6f; //state roller_in 2

  chThdSleepSeconds(2);
  if(!lift_getError())
    lift_calibrate();

  systime_t roller_in_start;

  uint8_t S1,S2;
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

    S1 = rc->rc.s1;
    S2 = rc->rc.s2;

    /* robot_state controller for island climbing machine*/
    if(S1 == S1_RESET)
      s1_reset = true;

    int16_t input = rc->rc.channel3 - RC_CH_VALUE_OFFSET;
    if(input > 400)
      pos_cmd += 0.1f;
    else if(input > 100)
      pos_cmd += 0.025f;
    else if(input < -400)
      pos_cmd -= 0.1f;
    else if(input < -100)
      pos_cmd -= 0.025f;

    uint8_t prev_state = robot_state;
    switch(robot_state)
    {
      case STATE_GROUND:
        DOG_RELAX();
        chassis_headingLock(DISABLE);
        chassis_killAutoDriver();
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_BUM,DISABLE);
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_BUM,DISABLE);

        lift_changePos(47.0f - pos_cmd, 47.0f - pos_cmd ,
                      47.0f - pos_cmd, 47.0f - pos_cmd);

        if(S2 == ASCEND_MODE && (s1_reset && S1 == S1_ASCEND))
          island_robotSetState(STATE_ONFOOT);
        else if(S2 == DECEND_MODE && (s1_reset && S1 == S1_DECEND))
          island_robotSetState(STATE_RUSHDOWN_1);

        break;
      case STATE_ONFOOT:
        DOG_ERECT();
        #ifndef ISLAND_AUTO_DRIVE
          chassis_killAutoDriver();
        #endif

        rangeFinder_control(RANGEFINDER_INDEX_NOSE, ENABLE);
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_BUM,DISABLE);
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_BUM,DISABLE);


        //Pos0: on_foot position setpoint
        lift_changePos(pos_sp[0] - pos_cmd, pos_sp[0] - pos_cmd ,
                      pos_sp[0] - pos_cmd, pos_sp[0] - pos_cmd);

        if(S2 == ASCEND_MODE)
        {
          if(island_state == STATE_STAIR_0 && s1_reset && S1 == S1_ASCEND)
          {
            start_yaw = pIMU->euler_angle[Yaw];
            island_robotSetState(STATE_ROLLER_IN);
            roller_in_start = chVTGetSystemTimeX();
            chassis_headingLock(ENABLE);
          }
          else if(
                  island_state == STATE_STAIR_1 &&
                  lift_inPosition() &&
                  threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_NOSE) < threshold[4], 6, &count)
                 )
          {
            island_robotSetState(STATE_ROLLER_IN);
            roller_in_start = chVTGetSystemTimeX();
          }
        }

        if(s1_reset && S1 == S1_DECEND)
          island_robotSetState(STATE_GROUND);

        if(S2 == DECEND_MODE && (s1_reset && S1 == S1_DECEND))
          island_robotSetState(STATE_RUSHDOWN_1);

        break;
      case STATE_ROLLER_IN:
        rangeFinder_control(RANGEFINDER_INDEX_NOSE, DISABLE);

        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                      pos_sp[1] - pos_cmd, pos_sp[1] - pos_cmd);

        #ifdef ISLAND_AUTO_DRIVE
          if(lift_inPosition())
            chassis_autoCmd(CHASSIS_DRIVE, -150.0f);
        #endif

        if(S2 == ASCEND_MODE &&
            (
              (s1_reset && S1 == S1_ASCEND) ||
              (
                 chVTGetSystemTimeX() > roller_in_start + MS2ST(1350)
              )
            )
          )
          island_robotSetState(STATE_ROLLER_IN_2);
        else if(s1_reset && S1 == S1_DECEND)
          island_robotSetState(STATE_ONFOOT);

        break;
      case STATE_ROLLER_IN_2:
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_DOGBALL,  ENABLE);
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_DOGBALL, ENABLE);

        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                      pos_sp[1] + roller_in - pos_cmd, pos_sp[1] + roller_in - pos_cmd);

        if(S2 == ASCEND_MODE &&(s1_reset && S1 == S1_ASCEND))
        {
          island_robotSetState(STATE_CRAW);
          island_state++;
        }
        else if(S2 == ASCEND_MODE &&
              (lift_inPosition() &&
              threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_LEFT_DOGBALL) < threshold[5], 6, &count_left)))
        {
          #ifdef ISLAND_AUTO_DRIVE
            chassis_autoCmd(CHASSIS_DRIVE, -20.0f);
          #endif
          island_robotSetState(STATE_ROLLER_IN_LEFT);
        }
        else if(S2 == ASCEND_MODE &&
              (lift_inPosition() &&
              threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_RIGHT_DOGBALL) < threshold[5], 6, &count_right)))
        {
          #ifdef ISLAND_AUTO_DRIVE
            chassis_autoCmd(CHASSIS_DRIVE, -20.0f);
          #endif
          island_robotSetState(STATE_ROLLER_IN_RIGHT);
        }
        else if(s1_reset && S1 == S1_DECEND)
        {
          chassis_killAutoDriver();
          island_robotSetState(STATE_ONFOOT);
        }

        break;
      case STATE_ROLLER_IN_LEFT:
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_DOGBALL,  DISABLE);

        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                      pos_sp[1] + roller_in - pos_cmd, pos_sp[2] - pos_cmd);

        if(S2 == ASCEND_MODE &&
            (
              (s1_reset && S1 == S1_ASCEND) ||
              (threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_RIGHT_DOGBALL) < threshold[5], 6, &count_right))
            )
          )
        {
          island_robotSetState(STATE_CRAW);
          island_state++;
        }
        else if(
                 s1_reset && S1 == S1_DECEND ||
                 pIMU->euler_angle[Pitch] < 0.2f
               )
          island_robotSetState(STATE_ROLLER_IN_2);

        break;
      case STATE_ROLLER_IN_RIGHT:
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_DOGBALL, DISABLE);

        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                      pos_sp[2] - pos_cmd, pos_sp[1] + roller_in - pos_cmd);

        if(S2 == ASCEND_MODE &&
            (
              (s1_reset && S1 == S1_ASCEND) ||
              (threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_LEFT_DOGBALL) < threshold[5], 6, &count_left))
            )
          )
        {
          island_robotSetState(STATE_CRAW);
          island_state++;
        }
        else if(
                 s1_reset && S1 == S1_DECEND ||
                 pIMU->euler_angle[Pitch] < 0.2f
               )
          island_robotSetState(STATE_ROLLER_IN_2);

        break;
      case STATE_CRAW:
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_DOGBALL,  DISABLE);
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_DOGBALL, DISABLE);

        #ifdef ISLAND_AUTO_DRIVE
          if(lift_inPosition())
            chassis_autoCmd(CHASSIS_DRIVE, -150.0f);
        #endif

        if(island_state == STATE_STAIR_1 || island_state == STATE_STAIR_2)
        {
          rangeFinder_control(RANGEFINDER_INDEX_LEFT_BUM, ENABLE);
          rangeFinder_control(RANGEFINDER_INDEX_RIGHT_BUM, ENABLE);
        }

        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                      pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd);

        if(
            (S2 == ASCEND_MODE && (s1_reset && S1 == S1_ASCEND))||
            (
              threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_RIGHT_BUM) < threshold[6], 3, &count_back_right) &&
              threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_LEFT_BUM) < threshold[6], 3, &count_back_left) &&
              pIMU->euler_angle[Pitch] < 0.2f
            )
          )
        {
          if(island_state == STATE_STAIR_1)
          {
            island_robotSetState(STATE_ONFOOT);

            #ifdef ISLAND_AUTO_DRIVE
              chassis_autoCmd(CHASSIS_DRIVE, -75.0f);
              chassis_autoCmd(CHASSIS_HEADING, start_yaw + DEG2RAD(90.0f));
            #endif
          }
          else if(island_state == STATE_STAIR_2)
            island_robotSetState(STATE_ISLAND_1);
          else
            island_robotSetState(STATE_GROUND);
        }
        else if(S2 == DECEND_MODE && (s1_reset && S1 == S1_DECEND))
          island_robotSetState(STATE_RUSHDOWN_1);
        else if((s1_reset && S1 == S1_DECEND) || pIMU->euler_angle[Pitch] > 0.2f)
        {
          #ifdef ISLAND_AUTO_DRIVE
            chassis_autoCmd(CHASSIS_DRIVE, -20.0f);
          #endif
          island_robotSetState(STATE_ROLLER_IN_2);
          island_state--;
        }

        break;
      case STATE_ISLAND_1:
        DOG_RELAX();
        chassis_headingLock(DISABLE);
        chassis_killAutoDriver();
        lift_changePos(pos_sp[5] - pos_cmd, pos_sp[5] - pos_cmd ,
                      pos_sp[5] - pos_cmd, pos_sp[5] - pos_cmd);

        if(s1_reset && S1 == S1_ASCEND)
          island_robotSetState(STATE_ISLAND_2);
        else if(S2 == DECEND_MODE && (s1_reset && S1 == S1_DECEND))
          island_robotSetState(STATE_RUSHDOWN_1);
        break;
      case STATE_ISLAND_2:
        DOG_RELAX();
        lift_changePos(pos_sp[6] - pos_cmd, pos_sp[6] - pos_cmd ,
                      pos_sp[6] - pos_cmd, pos_sp[6] - pos_cmd);

        if(s1_reset && S1 == S1_DECEND)
        {
          if(S2 == DECEND_MODE)
            island_robotSetState(STATE_RUSHDOWN_1);
          else
            island_robotSetState(STATE_ISLAND_1);
        }

        break;
      case STATE_RUSHDOWN_1:
        DOG_RELAX();
        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                    pos_sp[4] - pos_cmd, pos_sp[4] - pos_cmd);

        if(lift_inPosition()
        && threshold_count(pIMU->euler_angle[Pitch] > threshold[1], 3, &count))
        {
          count_onGround = 0;
          island_robotSetState(STATE_RUSHDOWN_2);
        }
        else if(S2 == DECEND_MODE && (s1_reset && S1 == S1_ASCEND))
        {
          count = 0;
          island_robotSetState(STATE_GROUND);
        }

        break;
      case STATE_RUSHDOWN_2:
        lift_changePos(pos_sp[2] - pos_cmd, pos_sp[2] - pos_cmd ,
                  pos_sp[3] - pos_cmd, pos_sp[3] - pos_cmd);

        if(pIMU->euler_angle[Pitch] > threshold[2])
          count_onGround++;

        if(count_onGround >= 2 && threshold_count(pIMU->euler_angle[Pitch] < threshold[3], 3, &count))
        {
          if(island_state != STATE_STAIR_0)
            island_state--;

          if(island_state == STATE_STAIR_1)
            island_robotSetState(STATE_RUSHDOWN_1);
          else if(island_state == STATE_STAIR_0)
            island_robotSetState(STATE_GROUND);
        }
        else if(S2 == DECEND_MODE && (s1_reset && S1 == S1_DECEND))
        {
          if(island_state != STATE_STAIR_0)
            island_state--;

          island_robotSetState(STATE_GROUND);
        }

        break;
    }

    if(robot_state != prev_state)
      s1_reset = false;
  }
}

const char namePos[] = "pos_sp";
const char subNamePos[] = "1 2 3 4 5 6 7";

const char nameTH[] = "island_th";
const char subNameTH[] = "Up_Pitch Dn_Pitch1 Dn_Pitch2 Dn_Pitch3 Up_RF1 Up_RF2 Up_RF3";

void island_init(void)
{
  pIMU = imu_get();

  params_set(pos_sp,18,7,namePos,subNamePos,PARAM_PUBLIC);
  params_set(threshold,19,7,nameTH,subNameTH,PARAM_PUBLIC);

  chThdCreateStatic(Island_thread_wa, sizeof(Island_thread_wa),
  NORMALPRIO + 5,
      Island_thread, NULL); //*
}
