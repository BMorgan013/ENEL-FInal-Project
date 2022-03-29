/*
 * pwm.h
 *
 *  Created on: Mar 28, 2022
 *      Author: rdeje
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "main.h"

void shutter_motor_forwards(uint16_t delay); //function to turn on the shutter motor in the forward direction

void shutter_motor_backwards(uint16_t delay);

void light_bar(uint16_t lightvalue);

#endif /* INC_PWM_H_ */

