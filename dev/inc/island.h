#ifndef _ISLAND_H_
#define _ISLAND_H_

#define ROBOT_STATE_NUM  11U
typedef enum {
  STATE_GROUND,
  STATE_ONFOOT,
  STATE_ROLLER_IN,
  STATE_ROLLER_IN_2,
  STATE_ROLLER_IN_LEFT,
  STATE_ROLLER_IN_RIGHT,
  STATE_CRAW,
  STATE_ISLAND_1,
  STATE_ISLAND_2,
  STATE_RUSHDOWN_1,
  STATE_RUSHDOWN_2
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

#define ISLAND_UPDATE_PERIOD_MS 5
#define ISLAND_AUTO_DRIVE

void island_init(void);

#endif
