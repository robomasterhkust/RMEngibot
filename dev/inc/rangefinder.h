/*
 * hcsr04.h
 *
 *  Created on: 26 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_RANGEFINDER_H_
#define INC_RANGEFINDER_H_

#define RANGEFINDER_TIM_FREQ        100000
#define M_TO_CM                     100.0f
//#define SPEED_OF_SOUND              343.2f
#define S_TO_MS 					1000.0f
#define RATIO						100.0f
#define MM_TO_CM					10

#define RANGEFINDER_INDEX_NOSE            0
#define RANGEFINDER_INDEX_LEFT_DOGBALL    1
#define RANGEFINDER_INDEX_RIGHT_DOGBALL   2
#define RANGEFINDER_INDEX_LEFT_BUM        3
#define RANGEFINDER_INDEX_RIGHT_BUM       4

#define RANGEFINDER_NUM                   5

typedef enum{
  RANGEFINDER_DISABLE = 0,
  RANGEFINDER_ENABLE
} rangefinder_cmd_t;

inline void rangeFinder_control(const uint8_t index, const bool enable);
float rangeFinder_getDistance(const uint8_t index);

void rangeFinder_init(void);

#endif /* INC_HCSR04_H_ */
