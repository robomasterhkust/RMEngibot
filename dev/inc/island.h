#ifndef _ISLAND_H_
#define _ISLAND_H_

#define ISLAND_SPEED_LIMIT_LOW    0.3f
#define ISLAND_SPEED_LIMIT_HIGH   0.6f

#define ROBOT_STATE_NUM  19U
typedef enum {
  STATE_GROUND,
  STATE_TEST,
  STATE_ONFOOT,
  STATE_ROLLER_IN,
  STATE_ROLLER_IN_2,
  STATE_ROLLER_IN_LEFT,
  STATE_ROLLER_IN_RIGHT,
  STATE_CRAW,
  STATE_ISLAND_1,
  STATE_ISLAND_2,
  STATE_ISLAND_3,
  STATE_ISLAND_4,
  STATE_ISLAND_5,
  STATE_HERO_INTERACT_1,
  STATE_HERO_INTERACT_2,
  STATE_HERO_INTERACT_3,
  STATE_RUSHDOWN_1,
  STATE_RUSHDOWN_2,
  STATE_RUSHDOWN_3
} robot_state_t;

typedef enum {
  STATE_STAIR_0,
  STATE_STAIR_1,
  STATE_STAIR_2,
} island_state_t;

#define ASCEND_MODE   1
#define LOCK_MODE     3
#define DECEND_MODE   2

#define S1_ASCEND   1
#define S1_RESET    3
#define S1_DECEND   2

#define DOG_ERECT() (PN2_ON())
#define DOG_RELAX() (PN2_OFF())

#define POUR_AMMO() (PN1_ON())
#define CLOSE_LID() (PN1_OFF())

#define ISLAND_UPDATE_PERIOD_MS 5
//#define ISLAND_AUTO_DRIVE

island_state_t island_getState(void);
robot_state_t island_getRobotState(void);
void island_robotSetState(robot_state_t state);
void island_init(void);

#endif
