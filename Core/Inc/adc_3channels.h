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

/*
 * Function: ADC_Select_CHX
 * Configures and selects the right ADC channel
 * Parameters:
 * Returns:
*/
void ADC_Select_CH0(void); //pa0
void ADC_Select_CH1(void); //pa4
void ADC_Select_CH2(void); //pb0

/*
 * Function: ADC_Read_CHX
 * After selecting the targetted ADC channel, reads from that pin and stores into an array
 * Parameters:
 * Returns: ADC_VAL[X]
*/
uint16_t ADC_Read_CH0(void); //function reads what analog value is on the channel, then returns it as an int value
uint16_t ADC_Read_CH1(void);
uint16_t ADC_Read_CH2(void);

ADC_HandleTypeDef hadc1;

/*
 * Function: ADC_Average
 * Configures and reads a selected ADC channel 10 times, then averages the 10 entries, and then returns that average
 * Parameters: channel
 * Returns: avg
*/
float ADC_Average(uint16_t channel); //averages 10 adc inputs for a better sensor value result

#endif /* INC_ADC_3CHANNELS_H_ */
