 /*
 * man_im_moist.c
 *
 *  Created on: Mar. 11, 2022
 *      Author: rdeje
 */

#include "man_im_moist.h"

void flood_ON(void) //sets pc9, turning on the feed tank pump
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
}

void flood_OFF(void) //resets pc9, turning off the feed tank pump
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
}

void be_like_water(void) //pc8 is your gpio port for this, the mixing pump
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_SET); //sets pc8 as high, turning on the pump
}

void be_not_like_water(void) //pc8 is your gpio port for this
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET); //sets pc8 as low, turning off the pump
}

float moisture_sense(void) //moisture sensing pb0, returns whatever pb0 is seeing
{
	float moisture = 0;
	moisture = ((ADC_Average(2)*3.3)/4096); //averages 10 entries of the moisture sensor
	return moisture; //returns the moisture level
}

void flood_logic(void) //turn on flood pump if moisture needed
{
	float MOIST = 0; //moisture comparison variable
	MOIST = moisture_sense(); //check the moisture level
	if(MOIST > 2500) //if it's super dry
	{
		flood_ON(); //turn on the flood pump
		while(MOIST > 2500) //keeps checking the moisture level
		{
			MOIST = moisture_sense();
			//if the code is in this loop, then the flood pump will be providing water
		}
		flood_OFF();
	}
	else
	{
		//if moisture is adequate, do nothing
	}
}

void water_level_check(void) //PA12
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) //if pa12 is pulled low by the switch (switch open)
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_SET);
		//send a signal to the server, low water level
	}
	else
	{
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, GPIO_PIN_RESET);
		//do nothing
	}
}

