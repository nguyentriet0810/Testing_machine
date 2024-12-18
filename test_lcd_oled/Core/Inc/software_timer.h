/*
 * software_timer.h
 *
 *  Created on: Oct 25, 2024
 *      Author: 23tri
 */

#ifndef INC_SOFTWARE_TIMER_H_
#define INC_SOFTWARE_TIMER_H_

#include "main.h"
#include "button.h"

extern int timer3_flag;

extern int timer4_flag;

void set_timer3(int duration);

void set_timer4(int duration);

void run_timer3();

void run_timer4();


#endif /* INC_SOFTWARE_TIMER_H_ */
