/*
 * pwm.c
 *
 *  Created on: Mar 28, 2022
 *      Author: rdeje
 */

#include "pwm.h"
#include "time.h"

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void shutter_motor_forwards(uint16_t delay) //forwards is on timer 3, channel 1
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 10); //provides PWM
	HAL_Delay(delay); //keeps it on for however long it takes to fully cover the structure
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_1, 0); //turns off
}

void shutter_motor_backwards(uint16_t delay) //backwards is on timer 3, channel 2
{
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 10); //backwards
	HAL_Delay(delay);
	__HAL_TIM_SetCompare(&htim3, TIM_CHANNEL_2, 0);
}

void light_bar(uint16_t lightvalue) //light bar is on timer 4 channel 1
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, lightvalue); //updates the PWM with the passed through variable
}
