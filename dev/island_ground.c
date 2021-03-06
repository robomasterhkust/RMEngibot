#include "ch.h"
#include "hal.h"

#include "island.h"
#include "mpu6500.h"
#include "dbus.h"
#include "rangefinder.h"
#include "lift.h"
#include "gripper.h"
#include "math_misc.h"
#include "gimbal.h"
static PIMUStruct pIMU;
static RC_Ctl_t* rc;

static island_state_t island_state = STATE_STAIR_0;
static robot_state_t robot_state = STATE_GROUND;
//static robot_state_t robot_state = STATE_ISLAND_1;
//testing the gripper

static param_t pos_sp[7];
static param_t threshold[7];
static param_t gripper_pos_sp[6];
static param_t hero_interact_pos_sp[2];


static float start_yaw;
static systime_t pour_ammo_time = 0;
static systime_t gripper_release_time = 0;
static systime_t hero_interact_time = 0;

island_state_t island_getState(void)
{
  return island_state;
}

robot_state_t island_getRobotState(void)
{
  return robot_state;
}

void island_robotSetState(robot_state_t state)
{
  if(state < 0 || state > ROBOT_STATE_NUM)
    return;
  robot_state = state;
}

static inline bool island_ascend(void)
{
  return rc->rc.s1 == S1_ASCEND || (rc->keyboard.key_code & KEY_F);
}

static inline bool island_decend(void)
{
  return rc->rc.s1 == S1_DECEND || (rc->keyboard.key_code & KEY_R);
}

static THD_WORKING_AREA(Island_thread_wa, 1024);
static THD_FUNCTION(Island_thread, p)
{
  chRegSetThreadName("Island climbing");

  (void)p;

  rc = RC_get();

  bool s1_reset = false;

  uint16_t count = 0, count_left = 0, count_right = 0, count_onGround = 0,count_back_left=0,count_back_right =0;
  const float roller_in = 9.6f; //state roller_in 2

  chThdSleepSeconds(2);

  do{
    chThdSleepMilliseconds(1000);
  }while(lift_getError());

  lift_calibrate();

  // do{
  //   chThdSleepMilliseconds(1000);
  // }while(gripper_getError());
  // gripper_calibrate();

  gimbal_calibrate();

  systime_t roller_in_start;

  uint8_t S1,S2;
  uint32_t tick = chVTGetSystemTimeX();
  gimbal_ToStable();
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
    if(S1 == S1_RESET && !(rc->keyboard.key_code & (KEY_F | KEY_R)))
    {
      s1_reset = true;
    }

    uint8_t prev_state = robot_state;

  //  if(robot_state == STATE_GROUND || (robot_state <= STATE_RUSHDOWN_2 && robot_state >= STATE_HERO_INTERACT_1 ))
  //    gimbal_ToStable();
  //  else
//      gimbal_ToScreen();

    switch(robot_state)
    {
      case STATE_GROUND:
      rangeFinder_control(RANGEFINDER_INDEX_LEFT_BUM,DISABLE);
      rangeFinder_control(RANGEFINDER_INDEX_RIGHT_BUM,DISABLE);
      case STATE_TEST:
      DOG_RELAX();
      CLOSE_LID();
        gripper_changePos(gripper_pos_sp[2], gripper_pos_sp[4]); //swing back, open hand

        chassis_setSpeedLimit(1.0f);
        chassis_setAcclLimit(100);
        chassis_headingLockCmd(DISABLE);
        chassis_killAutoDriver();

        lift_changePos(40.0f  , 40.0f   ,
          40.0f  , 40.0f  );

        if(S2 == ASCEND_MODE && (s1_reset && island_ascend()))
        {
          island_robotSetState(STATE_ONFOOT);
          gimbal_ToScreen();
        }
        else if(S2 == DECEND_MODE && (s1_reset && island_decend()))
          island_robotSetState(STATE_RUSHDOWN_1);//we can change this to resecue mode
        else if(S2 == LOCK_MODE &&(s1_reset && island_ascend()))
        {
          island_robotSetState(STATE_HERO_INTERACT_1);
        }

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
        lift_changePos(pos_sp[0]  , pos_sp[0]   ,
          pos_sp[0]  , pos_sp[0]  );
        chassis_setSpeedLimit(ISLAND_SPEED_LIMIT_HIGH);
        chassis_setAcclLimit(ISLAND_ACCL_LIMIT_LOW);

        if(S2 == ASCEND_MODE)
        {
          if(island_state == STATE_STAIR_0 && s1_reset && island_ascend())
          {
            start_yaw = pIMU->euler_angle[Yaw];
            island_robotSetState(STATE_ROLLER_IN);
            roller_in_start = chVTGetSystemTimeX();
            //no gyro, disable this
            chassis_headingLockCmd(DISABLE);
          }
          else if(
            island_state == STATE_STAIR_1 &&
            lift_inPosition() &&
            (threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_NOSE) < threshold[4], 6, &count)
              ||s1_reset && island_ascend())

            )
          {
            island_robotSetState(STATE_ROLLER_IN);
            roller_in_start = chVTGetSystemTimeX();
          }
        }

        if(s1_reset && island_decend())
        {
          island_robotSetState(STATE_GROUND);
          gimbal_ToStable();
        }

        if(S2 == DECEND_MODE && (s1_reset && island_decend()))
          island_robotSetState(STATE_RUSHDOWN_1);

        break;
        case STATE_ROLLER_IN:
        rangeFinder_control(RANGEFINDER_INDEX_NOSE, DISABLE);

        lift_changePos( pos_sp[1]  , pos_sp[1]  ,
          pos_sp[2]  , pos_sp[2]  );
        chassis_setSpeedLimit(ISLAND_SPEED_LIMIT_HIGH);
        chassis_setAcclLimit(ISLAND_ACCL_LIMIT_HIGH);

        #ifdef ISLAND_AUTO_DRIVE
        if(lift_inPosition())
          chassis_autoCmd(CHASSIS_DRIVE, -150.0f);
        #endif

        if(S2 == ASCEND_MODE &&
          (
            (s1_reset && island_ascend()) ||
            (
             chVTGetSystemTimeX() > roller_in_start + MS2ST(1350)
             )
            )
          )
          island_robotSetState(STATE_ROLLER_IN_2);
        else if(s1_reset && island_decend())
          island_robotSetState(STATE_ONFOOT);

        break;
        case STATE_ROLLER_IN_2:
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_DOGBALL,  ENABLE);
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_DOGBALL, ENABLE);

        lift_changePos(  pos_sp[1] + roller_in  , pos_sp[1] + roller_in ,
         pos_sp[2]  , pos_sp[2] );

        if(S2 == ASCEND_MODE &&(s1_reset && island_ascend()))
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

        else if(s1_reset && island_decend())
        {
          chassis_killAutoDriver();
          island_robotSetState(STATE_ONFOOT);
        }

        break;
        case STATE_ROLLER_IN_LEFT:
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_DOGBALL,  DISABLE);

        lift_changePos( pos_sp[1] + roller_in , pos_sp[2]   ,
         pos_sp[2] , pos_sp[2]  );

        if(S2 == ASCEND_MODE &&
          (
            (s1_reset && island_ascend()) ||
            (threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_RIGHT_DOGBALL) < threshold[5], 6, &count_right))
            )
          )
        {
          island_robotSetState(STATE_CRAW);
          island_state++;
        }
        else if(
         (s1_reset && island_decend()) 
         ||pIMU->euler_angle[Pitch] > 0.2f
         )
          island_robotSetState(STATE_ROLLER_IN_2);

        break;
        case STATE_ROLLER_IN_RIGHT:
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_DOGBALL, DISABLE);

        lift_changePos(pos_sp[2]  ,  pos_sp[1] + roller_in,
           pos_sp[2]   ,    pos_sp[2]   );

        if(S2 == ASCEND_MODE &&
          (
            (s1_reset && island_ascend()) ||
            (threshold_count(rangeFinder_getDistance(RANGEFINDER_INDEX_LEFT_DOGBALL) < threshold[5], 6, &count_left))
            )
          )
        {
          island_robotSetState(STATE_CRAW);
          island_state++;
        }
        else if(
         (s1_reset && island_decend())
         ||pIMU->euler_angle[Pitch] > 0.2f
         )
          island_robotSetState(STATE_ROLLER_IN_2);

        break;
        case STATE_CRAW:
        rangeFinder_control(RANGEFINDER_INDEX_LEFT_DOGBALL,  DISABLE);
        rangeFinder_control(RANGEFINDER_INDEX_RIGHT_DOGBALL, DISABLE);
        chassis_headingLockCmd(DISABLE);
        #ifdef ISLAND_AUTO_DRIVE
        if(lift_inPosition())
          chassis_autoCmd(CHASSIS_DRIVE, -150.0f);
        #endif

        chassis_setSpeedLimit(ISLAND_SPEED_LIMIT_HIGH);
        chassis_setAcclLimit(ISLAND_SPEED_LIMIT_HIGH);

        if(island_state == STATE_STAIR_1 || island_state == STATE_STAIR_2)
        {
          rangeFinder_control(RANGEFINDER_INDEX_LEFT_BUM, ENABLE);
          rangeFinder_control(RANGEFINDER_INDEX_RIGHT_BUM, ENABLE);
        }

        lift_changePos(pos_sp[2]  , pos_sp[2]   ,
          pos_sp[2]  , pos_sp[2]  );

        if(
          (S2 == ASCEND_MODE && (s1_reset && island_ascend()))||
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
        else if(S2 == DECEND_MODE && (s1_reset && island_decend()))
          island_robotSetState(STATE_RUSHDOWN_1);
        else if((s1_reset && island_decend()) || pIMU->euler_angle[Pitch] > 0.2f)
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
        chassis_headingLockCmd(DISABLE);
        chassis_killAutoDriver();
        lift_changePos(pos_sp[4]  , pos_sp[4]   ,
          pos_sp[4]  , pos_sp[4]  );
        chassis_setSpeedLimit(ISLAND_SPEED_LIMIT_LOW);
        chassis_setAcclLimit(ISLAND_ACCL_LIMIT_LOW);

        if(chVTGetSystemTimeX() - gripper_release_time > MS2ST(1000))
          gripper_changePos(gripper_pos_sp[3], gripper_pos_sp[4]); //strech up, open hand

        if(s1_reset && island_ascend())
          island_robotSetState(STATE_ISLAND_2);
        else if(S2 == DECEND_MODE && (s1_reset && island_decend()))
          island_robotSetState(STATE_RUSHDOWN_1);
        break;
        case STATE_ISLAND_2:
        DOG_RELAX();

        //strech out, open hand
        lift_changePos(pos_sp[4]  , pos_sp[4]   ,
                      pos_sp[4]  , pos_sp[4]  ); //lift up a little bit
        gripper_changePos(gripper_pos_sp[1], gripper_pos_sp[4]); //strech out, open hand

        if(s1_reset && island_ascend())
        {
          chassis_tempSuspend(ENABLE);
          //strech out, close hand, get the block
          gripper_changePos(gripper_pos_sp[1], gripper_pos_sp[5]); //strech out, close hand
          island_robotSetState(STATE_ISLAND_3);
        }
        else if(S2 == ASCEND_MODE && (s1_reset && island_decend()))
          island_robotSetState(STATE_ISLAND_1);
        else if(S2 == DECEND_MODE && (s1_reset && island_decend()))
          island_robotSetState(STATE_RUSHDOWN_1);
        break;
        case STATE_ISLAND_3:
        DOG_RELAX();

        if(gripper_inPosition(GRIPPER_HAND))
        {
          lift_changePos(pos_sp[5]  , pos_sp[5]   ,
                        pos_sp[5]  , pos_sp[5]  ); //lift up a little bit
          if(lift_inPosition())
          {
            chassis_tempSuspend(DISABLE);
            gripper_changePos(gripper_pos_sp[3], gripper_pos_sp[5]); //swing up. close hand
            if(gripper_inPosition(GRIPPER_ARM))
            {
              pour_ammo_time = chVTGetSystemTimeX();
              island_robotSetState(STATE_ISLAND_4);
            }
          }
        }
        else if(S2 == ASCEND_MODE && s1_reset && island_decend())
        {
          chassis_tempSuspend(DISABLE);
          island_robotSetState(STATE_ISLAND_1);
        }
        else if(S2 == DECEND_MODE && s1_reset && island_decend())
        {
          chassis_tempSuspend(DISABLE);
          island_robotSetState(STATE_RUSHDOWN_1);
        }
        break;
        case STATE_ISLAND_4:
        DOG_RELAX();

        if(chVTGetSystemTimeX() > pour_ammo_time + MS2ST(1500))
        {
          gripper_changePos(gripper_pos_sp[1], gripper_pos_sp[4]); //strech out. close hand
          gripper_release_time = chVTGetSystemTimeX();
          island_robotSetState(STATE_ISLAND_5);
        }
        else if(S2 == DECEND_MODE && (s1_reset && island_decend()))
          island_robotSetState(STATE_RUSHDOWN_1);
        break;
        case STATE_ISLAND_5:
        DOG_RELAX();

        if(chVTGetSystemTimeX() > gripper_release_time + MS2ST(700))
        {
          //gripper_changePos(gripper_pos_sp[1], gripper_pos_sp[4]); //strech out. open hand
          if(gripper_inPosition(GRIPPER_HAND))
            island_robotSetState(STATE_ISLAND_1);
        }
        else if(S2 == DECEND_MODE && (s1_reset && island_decend()))
          island_robotSetState(STATE_RUSHDOWN_1);
        break;
        case STATE_HERO_INTERACT_1:
        DOG_RELAX();
        CLOSE_LID();
        lift_changePos(hero_interact_pos_sp[0]  , hero_interact_pos_sp[0]   ,
         hero_interact_pos_sp[0] ,hero_interact_pos_sp[0]  );
        chassis_setSpeedLimit(ISLAND_SPEED_LIMIT_LOW);
        chassis_setAcclLimit(ISLAND_ACCL_LIMIT_LOW);

        if(lift_inPosition())
        {
          if(S2 == LOCK_MODE && (s1_reset && island_ascend()))
            island_robotSetState(STATE_HERO_INTERACT_2);
          else if(S2 == LOCK_MODE && (s1_reset && island_decend()))
            island_robotSetState(STATE_GROUND);
        }
        break;
        case STATE_HERO_INTERACT_2:
        lift_changePos(hero_interact_pos_sp[1]  , hero_interact_pos_sp[1]   ,
         hero_interact_pos_sp[0] ,hero_interact_pos_sp[0]  );
        if(lift_inPosition()){
          if(S2 == LOCK_MODE && (s1_reset && island_ascend()))
            island_robotSetState(STATE_HERO_INTERACT_3);
        }
        break;
        case STATE_HERO_INTERACT_3:
        POUR_AMMO();
        hero_interact_time = chVTGetSystemTimeX();
        if(S2 == LOCK_MODE && (s1_reset && island_ascend()))
        {
          CLOSE_LID();
          island_robotSetState(STATE_GROUND);
        }
        break;
        case STATE_RUSHDOWN_1:
        DOG_RELAX();
        lift_changePos(  pos_sp[4]  , pos_sp[4],
          pos_sp[2]  , pos_sp[2]   );
        gripper_changePos(gripper_pos_sp[2], gripper_pos_sp[4]); //swing back, open hand
        chassis_setSpeedLimit(ISLAND_SPEED_LIMIT_HIGH);
        chassis_setAcclLimit(100U);

        if(lift_inPosition()
          && threshold_count(pIMU->euler_angle[Pitch] > threshold[1], 3, &count))
        {
          count_onGround = 0;
          island_robotSetState(STATE_RUSHDOWN_2);
        }
        else if(S2 == DECEND_MODE && (s1_reset && island_ascend()))
        {
          count = 0;
          island_robotSetState(STATE_GROUND);
          island_state = STATE_STAIR_0;
        }

        break;
        case STATE_RUSHDOWN_2:
        lift_changePos(   pos_sp[3]  , pos_sp[3] ,
         pos_sp[2]  , pos_sp[2]  );

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
        else if(S2 == DECEND_MODE && (s1_reset && island_decend()))
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

  static const char PosName[] = "Gripper Pos";
  static const char PosSubName[] = "ArmIni ArmOut ArmIn ArmUp Open Close";

  static const char PosInteractName[] = "Hero interact";
  static const char PosInteractSubName[] = "front back";




  void island_init(void)
  {
    pIMU = imu_get();

    params_set(pos_sp,18,7,namePos,subNamePos,PARAM_PUBLIC);
    params_set(threshold,19,7,nameTH,subNameTH,PARAM_PUBLIC);
    params_set(gripper_pos_sp, 22,6,PosName,PosSubName,PARAM_PUBLIC);
    params_set(hero_interact_pos_sp,23,2,PosInteractName,PosInteractSubName,PARAM_PUBLIC);

    chThdCreateStatic(Island_thread_wa, sizeof(Island_thread_wa),
  NORMALPRIO + 5, Island_thread, NULL); //*
  }
