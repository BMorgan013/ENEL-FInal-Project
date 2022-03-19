/*
 * adc_3channels.h
 *
 *  Created on: Mar 6, 2022
 *      Author: Don de Jesus - 200246283
 *      This code is responsible for handling the values that are read through the 3 analog channels
 *      we're using for the pH sensor, EC sensor, and moisture sensor
 */

#ifndef INC_ADC_3CHANNELS_H_
#define INC_ADC_3CHANNELS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "main.h"

#include <string.h>
#include <stdio.h>

//Functions responsible for selecting which adc channel is to be used
void ADC_Select_CH0(void); //pa0
void ADC_Select_CH1(void); //pa4
void ADC_Select_CH2(void); //pb0

uint16_t ADC_Read_CH0(void); //function reads what analog value is on the channel, then returns it as an int value
uint16_t ADC_Read_CH1(void);
uint16_t ADC_Read_CH2(void);

ADC_HandleTypeDef hadc1;

float ADC_Average(uint16_t channel); //averages 10 adc inputs

#endif /* INC_ADC_3CHANNELS_H_ */
