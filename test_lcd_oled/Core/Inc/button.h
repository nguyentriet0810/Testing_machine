/*
 * button.h
 *
 *  Created on: Oct 25, 2024
 *      Author: nguyen hoang minh triet
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_

#include "main.h"
#include "fonts.h"
#include "ssd1306.h"

extern uint8_t counter;
extern uint8_t status;

extern uint8_t set_mode;
extern uint8_t set_times;
extern uint8_t set_step;
extern uint8_t set_lengh;

#define NORMAL_STATE GPIO_PIN_SET
#define PRESSED_STATE GPIO_PIN_RESET

void getKeyInput();

void subKeyProcess();

#endif /* INC_BUTTON_H_ */
