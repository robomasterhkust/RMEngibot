/*
 * hcsr04.h
 *
 *  Created on: 26 Jan, 2018
 *      Author: ASUS
 */

#ifndef INC_RANGEFINDER_H_
#define INC_RANGEFINDER_H_

#define ICU_TIM_FREQ        1000000
#define M_TO_CM             100.0f
#define SPEED_OF_SOUND      343.2f

float rangeFinder_getDistance(void);
void rangeFinder_init(void);

#endif /* INC_HCSR04_H_ */
