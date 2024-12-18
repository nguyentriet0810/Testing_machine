/*
 * software_timer.c
 *
 *  Created on: Oct 25, 2024
 *      Author: 23tri
 */
#include "software_timer.h"

int timer3_counter = 0;
int timer4_counter = 0;

int timer3_flag = 0;
int timer4_flag = 0;

void set_timer3(int duration)
{
	timer3_counter = duration;
	timer3_flag = 0;

}

void set_timer4(int duration)
{
	timer4_counter = duration;
	timer4_flag = 0;
}

void run_timer3()
{
	if (timer3_counter > 0)
	{
		timer3_counter--;
		if(timer3_counter <= 0) {
			timer3_flag = 1;
		}
	}
}

void run_timer4()
{
	if (timer4_counter > 0)
	{
		timer4_counter--;
		if(timer4_counter <= 0) {
			timer4_flag = 1;
		}
	}
}
