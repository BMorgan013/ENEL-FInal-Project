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

uint8_t shutter_motor_forwards(uint16_t delay) //forwards is on timer 3, channel 1
{
	uint8_t shutter_status;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
	HAL_Delay(delay);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
	shutter_status = 1; //1 means shutter is now covering the plant
	return shutter_status;
}

uint8_t shutter_motor_backwards(uint16_t delay) //backwards is on timer 3, channel 2
{
	uint8_t shutter_status;
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(delay);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
	shutter_status = 0; //0 means shutter is now retracted
	return shutter_status;
}

void light_bar(uint16_t lightvalue) //light bar is on timer 4 channel 1
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, lightvalue); //updates the PWM with the passed through variable
}

void fans(uint16_t fanValue) //fans are on timer 2, channel 2
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, fanValue);
}
