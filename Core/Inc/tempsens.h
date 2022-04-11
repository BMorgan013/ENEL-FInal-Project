/*****************************************************************************
UNIVERSITY OF REGINA FACULTY OF APPLIED SCIENCE & ENGINEERING
Title: Group 5 - Automated Greenhouse System Capstone
Link: https://www.micropeta.com/video48
Datasheet: https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf
Comments: We are using Nizar Mohideen's DHT22 library from MicroPeta
******************************************************************************/

/*
 * tempsens.h
 *
 *  Created on: Mar 9, 2022
 *      Author: klaudius
 */

#ifndef SRC_TEMPSENS_H_
#define SRC_TEMPSENS_H_

#include "stm32f1xx_hal.h"
#include "main.h"

typedef struct
{
	float tCelsius;
	float RH;
}DHT_DataTypedef;

void readDHT22 (DHT_DataTypedef *DHT_Data);
void readDHT22Sunflowers (DHT_DataTypedef *DHT_Data);
void readDHT22Strawberries (DHT_DataTypedef *DHT_Data);
void readDHT22Tomatoes (DHT_DataTypedef *DHT_Data);
//void fanSet();
//void shadeMotor();

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

#endif /* SRC_TEMPSENS_H_ */
