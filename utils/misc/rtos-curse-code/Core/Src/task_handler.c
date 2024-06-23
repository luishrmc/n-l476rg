/*
 * task_handler.c
 *
 *  Created on: Feb 23, 2024
 *      Author: luish
 */

#include "main.h"
#include <string.h>

void processCommand(cmd_t* cmd);
void getState(state_t* currState);
void setState(state_t newState);

state_t state = MAIN_MENU;

const char* msgMenu = "\r\n\r\n========================\r\n"
			  	  	  "|         Menu         |\r\n"
			  	  	  "========================\r\n"
			  	  	  "LED Effect	-> 0\r\n"
					  "Data and Time	-> 1\r\n"
					  "Exit		-> 2\r\n"
			  	  	  "Enter your choice here : ";

const char* msgLed = "\r\n\r\n========================\r\n"
			  	  	  "|      LED Effect     |\r\n"
			  	  	  "========================\r\n"
			  	  	  "(none, e1, e2, e3 ,e4)\r\n"
			  	  	  "Enter your choice here : ";

const char* msgInvalid = "\r\n\r\n////Invalid Option////\r\n\r\n";

void menuTask (void* pvParameters)
{
	BaseType_t ret;
	uint32_t cmdAddr;
	cmd_t* cmd;

	xTaskHandle xLed, xRtc;
	xLed = xTaskGetHandle("LED");
	xRtc = xTaskGetHandle("RTC");

	while(1)
	{
		// sending the pointer of the string
		xQueueSend((*(QueueHandle_t*)(pvParameters)), &msgMenu, portMAX_DELAY);

		// wait until receive some notification with command address
		ret = xTaskNotifyWait(0,0, &cmdAddr, portMAX_DELAY);

		if(ret == pdTRUE)
		{
			cmd = (cmd_t *)cmdAddr;

			if(cmd->buffSize == 1)
			{
				operation_t opt = cmd->buff[0] - 48; // ASCII to number

				switch(opt)
				{
					case LED: {
						setState(LED_EFFECT);
						xTaskNotify(xLed, 0, eNoAction);
					break;
					}

					case DATA_TIME: {
						setState(RTC_MENU);
						xTaskNotify(xRtc, 0, eNoAction);
					break;
					}

					case EXIT: {

					break;
					}

					default:
						// sending the pointer of the string
						xQueueSend((*(QueueHandle_t*)(pvParameters)), &msgInvalid, portMAX_DELAY);
						break;
				}
			}
			else
			{
				// sending the pointer of the string
				xQueueSend((*(QueueHandle_t*)(pvParameters)), &msgInvalid, portMAX_DELAY);
				continue;
			}
		}

		//wait to run again when some other task notifies
		xTaskNotifyWait(0,0, NULL, portMAX_DELAY);
	}
}

void cmdTask (void* pvParameters)
{
	BaseType_t ret;

	state_t currState;

	uint8_t data, idx;

	cmd_t cmd;

	xTaskHandle xMenu, xLed;
	xMenu = xTaskGetHandle("MENU");
	xLed = xTaskGetHandle("LED");
//	xRtc = xTaskGetHandle("RTC");

	while(1)
	{
		// wait until receive some notification -> portMAX_DELAY
		ret = xTaskNotifyWait(0,0, NULL, portMAX_DELAY);

		// the notification is received
		if(ret == pdTRUE)
		{
			ret = uxQueueMessagesWaiting( (*(QueueHandle_t*)(pvParameters)) );

			if(ret != 0)
			{
				memset(&cmd, 0, sizeof(cmd));
				idx = 0;
				data = 0;

				while(1)
				{
					ret = xQueueReceive( (*(QueueHandle_t*)(pvParameters)), &data, 0);
					if(ret == pdTRUE)
					{
						if(data == 0xD)
							break;

						cmd.buff[idx] = data;
						idx++;
					}
				}
				cmd.buffSize = idx;

				getState(&currState);
				switch(currState)
				{
					case MAIN_MENU: {
						xTaskNotify(xMenu, (uint32_t)&cmd, eSetValueWithOverwrite);
					break;
					}

					case LED_EFFECT: {
						xTaskNotify(xLed, (uint32_t)&cmd, eSetValueWithOverwrite);
					break;
					}

					case RTC_MENU: {

					break;
					}

					case RTC_CONF_TIME: {

					break;
					}

					case RTC_CONF_DATE: {

					break;
					}

					case RTC_REPORT: {

					break;
					}

					default:
						break;
				}
			}
		}
	}
}

void printTask (void* pvParameters)
{
	uint32_t* msg;
	while(1)
	{
		// stay blocked until some data is inserted into the queue
		// getting the pointer of the string
		xQueueReceive( (*(QueueHandle_t*)(pvParameters)), &msg, portMAX_DELAY );
//		printMsg(msg);
	}
}

void ledTask (void* pvParameters)
{
	BaseType_t ret;
	uint32_t cmdAddr;
	cmd_t* cmd;

	xTaskHandle xMenu;
	xMenu = xTaskGetHandle("MENU");

	while(1)
	{
		//wait to run again when some other task notifies
		xTaskNotifyWait(0,0, NULL, portMAX_DELAY);

		// sending the pointer of the string
		xQueueSend((*(QueueHandle_t*)(pvParameters)), &msgLed, portMAX_DELAY);

		ret = xTaskNotifyWait(0,0, &cmdAddr, portMAX_DELAY);

		if(ret == pdTRUE)
		{
			cmd = (cmd_t *)cmdAddr;

			if(cmd->buffSize <= 4)
			{
				if(! strcmp( (char*)cmd->buff , "none"))
					ledEffectStop();

				else if(! strcmp( (char*)cmd->buff , "e1"))
					ledEffectStart(1);

				else if(! strcmp( (char*)cmd->buff , "e2"))
					ledEffectStart(2);

				else if(! strcmp( (char*)cmd->buff , "e3"))
					ledEffectStart(3);

				else if(! strcmp( (char*)cmd->buff , "e4"))
					ledEffectStart(4);
				else
					xQueueSend((*(QueueHandle_t*)(pvParameters)), &msgInvalid, portMAX_DELAY);
			}
			else
			{
				// sending the pointer of the string
				xQueueSend((*(QueueHandle_t*)(pvParameters)), &msgInvalid, portMAX_DELAY);
			}
			setState(MAIN_MENU);
			xTaskNotify(xMenu, 0, eNoAction);
		}
	}
}

void rtcTask (void* pvParameters)
{
	while(1)
	{

	}
}

void getState(state_t* currState)
{
	portENTER_CRITICAL();
	(*currState) = state;
	portEXIT_CRITICAL();

}

void setState(state_t newState)
{
	portENTER_CRITICAL();
	state = newState;
	portEXIT_CRITICAL();
}

