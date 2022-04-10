/*
 * pH_system.h
 *
 *  Created on: Mar. 12, 2022
 *      Author: Don de Jesus - 200246283
 *      This code is responsible for handling the values that are read through the 3 analog channels
 *      we're using for the pH sensor, EC sensor, and moisture sensor
 *
 */

#ifndef INC_CHEM_SYSTEM_H_
#define INC_CHEM_SYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "main.h"
#include "adc_3channels.h"

float pH_sense(void); //checks the ADC pin that the pH sensor is on PA0, then returns an analog value
float EC_sense(void); //returns the ADC value on PA4, corresponding to the EC sensor
//AS OF NOW ONLY THE ABOVE 2 FUNCTIONS ARE BEING USED IN BASELINE PROJECT MODEL

void pH_UP(uint16_t time); //turns on a doser pump responsible for increasing pH, input variable is how long you want the pump to run in ms
void pH_DOWN(uint16_t time); //turns on doser pump for x amount of ms, decreasing pH

#endif /* INC_CHEM_SYSTEM_H_ */
