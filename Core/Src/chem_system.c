/*
 * pH_system.c
 *
 *  Created on: Mar. 12, 2022
 *      Author: Don de Jesus - 200246283
 *      This code is responsible for handling the values that are read through the 3 analog channels
 *      we're using for the pH sensor, EC sensor, and moisture sensor
 *
 */

#include "chem_system.h"

float pH_sense(void) //pa0
{
	float pH = 0.0;
	pH = ((ADC_Average(1)*3.3)/4096);
	pH = (4 +((pH-2.03)*(-7.14))); //converts analog to a readable pH value
	return pH;
}

float EC_sense(void) //pa4, EC sensor
{
	float EC = 0.0;
	EC = (((ADC_Average(1)*3.3))/4096); //gives the voltage on the pa4 pin
	EC = ((0.006705848*EC) + 0.000306535)*1000; //converts voltage to applicable EC value
	return EC;
}

void pH_UP(uint16_t time) //pa11
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //sets the pin that triggers the pH dosing pump
	HAL_Delay(time); //keeps the pump running for the set time
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //turns pump off
}

void pH_DOWN(uint16_t time) //pa10
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET); //same logic as above
	HAL_Delay(time);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}



