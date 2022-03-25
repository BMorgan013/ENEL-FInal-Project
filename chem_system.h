/*
 * pH_system.h
 *
 *  Created on: Mar. 12, 2022
 *      Author: rdeje
 */

#ifndef INC_CHEM_SYSTEM_H_
#define INC_CHEM_SYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "main.h"
#include "adc_3channels.h"

float pH_sense(void); //checks the ADC pin that the pH sensor is on PA0, then returns an analog value (5% PLUS/MINUS)
float EC_sense(void); //returns the ADC value on PA4, corresponding to the EC sensor
//AS OF NOW ONLY THE ABOVE 2 FUNCTIONS ARE BEING USED IN BASELINE PROJECT MODEL

void pH_UP(uint16_t time); //turns on a doser pump responsible for increasing pH, input variable is how long you want the pump to run in ms
void pH_DOWN(uint16_t time); //turns on doser pump for x amount of ms, decreasing pH
void pH_logic(uint16_t upper, uint16_t lower); //logic tree for pH range correction
void EC_logic(uint16_t upper, uint16_t lower); //logic tree for EC range correction
void pH_watch(float upper, float lower); //function that checks the pH and does stuff idk, may not need this

#endif /* INC_CHEM_SYSTEM_H_ */
