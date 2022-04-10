 /*
 * man_im_moist.c
 *
 *  Created on: Mar. 11, 2022
 *      Author: Don de Jesus - 200246283
*      This code is responsible for all the functions regarding the watering system
 */

#include "water_system.h"
#include "pwm.h"
uint16_t dry_boundary = 2600; //analog moisture value where flood pump is told to turn on
uint16_t saturated_boundary = 1900; //analog moisture value where flood pump is told to turn off

void rain_reservoir_ON(void)//turns on the rain reservoir pump on, PA8
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void rain_reservoir_OFF(void)//turns on the rain reservoir pump off, PA8
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

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

float moisture_percentage(void) //converts the moisture level into a rough percentage for easier reading
{
	float moisture = 0;
	moisture = ((2600 - moisture_sense())/1100)*100; //converts the moisture value into a percentage based on the range 1900-3000
	return moisture;
}

void water_level_check(void) //controls the water level switch + rainwater pump logic on PA12
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_12)) //if pa12 is pulled low by the switch (switch open)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET); //sets pa8 (rainwater pump) if water level is low
		rain_reservoir_ON();
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET); //resets pa8 if water level is good
		rain_reservoir_OFF();
	}
}

