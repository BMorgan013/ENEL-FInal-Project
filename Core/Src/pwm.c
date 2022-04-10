/*
 * pwm.c
 *
 *  Created on: Mar 28, 2022
 *      Author: Don de Jesus - 200246283
 *      This code is responsible for running the components that use pwm or other related functions
 */

#include "pwm.h"
#include "time.h"

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

void outdoor_light_sense(float luxVal) //luxVal comparison values are smaller for testing/demo purposes
{
	if(luxVal > 3000) //sunny day (45000+)
	{
		let_there_be_light(0); //no need for the lights
	}
	else if ((1000 < luxVal) && (3000 > luxVal)) //overcast (between 30000-45000)
	{
		let_there_be_light(500); //half power
	}
	else //less than overcast
	{
		let_there_be_light(1000); //full light
	}
}

void shutter_motor_forwards(uint16_t delay)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET); //sets the pin that
	HAL_Delay(delay);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
}

void shutter_motor_backwards(uint16_t delay)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_SET);
	HAL_Delay(delay);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, GPIO_PIN_RESET);
}

void let_there_be_light(uint16_t lightvalue) //light bar is on timer 4 channel 1
{
	__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, lightvalue); //updates the PWM with the passed through variable
}

void fans(uint16_t fanValue) //fans are on timer 2, channel 2
{
	__HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_2, fanValue);
}

void fans_temp_control(float temp, float humidity)
{
	  if ((temp <= 18) && (humidity <= 60)) //if temp is below 18 and humidity is below 80%
	  {
		  fans(50); //run fans at lowest setting
	  }
	  else if (((temp >= 19) && (humidity <= 60)) || ((temp <= 24) && (humidity <=60))) //if temp is within ideal range of 19-24 degrees, and 80% humidity
	  {
		  fans(500); //run fans at 50%
	  }
	  else //if temp/humidity is above 24 degrees C and above 80% humidity
	  {
		  fans(1000); //run fans full blast
	  }

}

uint8_t shutter_temp_control(float temp, uint8_t shutter) //controls the shutters, based on the current temperatuer and shutter status
{
	  if ((temp >= 26) && (shutter != 1)) //if temperature is above 25 celsius, and the shutter isn't already closed
	  {
		shutter_motor_forwards(1650); //closes the blinds, to cool it down
		shutter = 1;
	  }
	  else if ((temp <= 24) && (shutter != 0)) //if temp is less than or equal to 24C, and the shutter isn't already opened
	  {
		shutter_motor_backwards(1650);
		shutter = 0;
	  }
	  else
	  {

	  }
	  return shutter;
}
