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
{	/*MAY HAVE TO CHANGE PINS BASED ON WHAT KLAUDE SETS FOR THE GPIO'S*/
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); //set pin PC6, IN1, tells motor to run forwards
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //resets pin PC6, IN1
		HAL_Delay(delay);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //sets pin PB16, IN2, tells motor to run the other way
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); //resets pin PC6, IN1
		HAL_Delay(delay);
}

void shutter_motor_backwards(uint16_t delay) //backwards is on timer 3, channel 2
{
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET); //set pin PC6, IN1, tells motor to run forwards
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); //resets pin PC6, IN1
	HAL_Delay(delay);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); //sets pin PB16, IN2, tells motor to run the other way
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET); //resets pin PC6, IN1
	HAL_Delay(delay);
}

void light_bar(uint16_t lightvalue) //light bar is on timer 4 channel 1
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, lightvalue); //updates the PWM with the passed through variable
}

void fans(uint16_t fanValue) //fans are on timer 2, channel 2
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, fanValue);
}
