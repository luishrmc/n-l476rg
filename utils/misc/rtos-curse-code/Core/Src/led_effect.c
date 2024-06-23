/*
 * led_effect.c
 *
 *  Created on: Feb 24, 2024
 *      Author: luish
 */


#include "main.h"

void ledEffectTurnAllOff(void)
{
	HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_RESET);
}

void ledEffectTurnAllOn(void)
{
	HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_SET);
}

void ledEffectCornerLed(void)
{
	HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_SET);
}

void ledEffectCentralLed(void)
{
	HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_RESET);
}

void ledEffect1(void)
{
	// always when the function is called
	// the flag is inverted
	static uint8_t flag = 1;
	// Truth Table XOR
	/*
	 * 0 0 0
	 * 0 1 1 <-
	 * 1 0 1
	 * 1 1 0 <-
	 * */
	(flag ^= 1) ? ledEffectTurnAllOff() : ledEffectTurnAllOn();
}

void ledEffect2(void)
{
	// always when the function is called
	// the flag is inverted
	static uint8_t flag = 1;
	// Truth Table XOR
	/*
	 * 0 0 0
	 * 0 1 1 <-
	 * 1 0 1
	 * 1 1 0 <-
	 * */
	(flag ^= 1) ? ledEffectCornerLed() : ledEffectCentralLed();

}
void ledEffect3(void)
{
	static uint8_t counter = 1;

	switch(counter)
	{
		case 1: {
			HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_RESET);
			counter++;
			break;
		}

		case 2: {
			HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_RESET);
			counter++;
			break;
		}

		case 3: {
			HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_SET);
			counter = 1;
			break;
		}

		default: {
			break;
		}
	}
}
void ledEffect4(void)
{
	static uint8_t counter = 3;

	switch(counter)
	{
		case 1: {
			HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_RESET);
			counter = 3;
			break;
		}

		case 2: {
			HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_SET);
			HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_RESET);
			counter--;
			break;
		}

		case 3: {
			HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(LED3_GPIO_PORT, LED3, GPIO_PIN_SET);
			counter--;
			break;
		}

		default: {
			break;
		}
	}
}
