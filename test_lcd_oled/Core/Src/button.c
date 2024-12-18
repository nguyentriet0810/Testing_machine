/*
 * button.c
 *
 *  Created on: Oct 25, 2024
 *      Author: nguyen hoang minh triet
 */

#include "button.h"

int keyReg0 = NORMAL_STATE;
int keyReg1 = NORMAL_STATE;
int keyReg2 = NORMAL_STATE;

int keyReg3 = NORMAL_STATE;

int timeForKeyPress = 200;

void subKeyProcess()
{

}

void getKeySTART_STOP()
{
	keyReg0 = keyReg1;
	keyReg1 = keyReg2;
	keyReg2 = HAL_GPIO_ReadPin(START_STOP_GPIO_Port, START_STOP_Pin);

	if ((keyReg0 == keyReg1) && (keyReg1 == keyReg2))
	{
		if (keyReg3 != keyReg2)
		{
			keyReg3 = keyReg2;
			if(keyReg2 == PRESSED_STATE)
			{
				//to do somethings
				timeForKeyPress = 200;
				if (counter < 5){
					counter++;
				}
				else{
					status++;
					if (status == 2) status = 0;
					if (status == 3) status = 1;
					if (status == 0){
						SSD1306_GotoXY (56, 11);
						SSD1306_Puts ("STOP     ", &Font_7x10, 1);
						SSD1306_UpdateScreen();
					}
					else if (status == 1){
						SSD1306_GotoXY (56, 11);
						SSD1306_Puts ("START    ", &Font_7x10, 1);
						SSD1306_UpdateScreen();
					}
				}
			}
		}
		else
		{
			timeForKeyPress--;
			if(timeForKeyPress == 0)
			{
				if(keyReg2 == PRESSED_STATE)
				{
					//to do somethings

				}
				timeForKeyPress = 200;
			}
		}
	}
}

void getKeyEMERGENCY()
{
	keyReg0 = keyReg1;
	keyReg1 = keyReg2;
	keyReg2 = HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin);

	if ((keyReg0 == keyReg1) && (keyReg1 == keyReg2))
	{
		if (keyReg3 != keyReg2)
		{
			keyReg3 = keyReg2;
			if(keyReg2 == PRESSED_STATE)
			{
				//to do somethings
				timeForKeyPress = 200;
				if (counter > 4) {
					status = 2;
					SSD1306_GotoXY (56, 11);
					SSD1306_Puts ("E_STOP     ", &Font_7x10, 1);
					SSD1306_UpdateScreen();
				}
			}
		}
	}
}
