/*
 * pwm.h
 *
 *  Created on: Mar 28, 2022
 *      Author: Don de Jesus - 200246283
 *      This code is responsible for running the components that use pwm or other related functions
 */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "main.h"

/*
 * Function:
 *
 * Parameters:
 * Returns:
*/

/*
 * Function: outdoor_light_sense
 * If the user chooses an outdoor plant configuration, decides how bright to shine the grow lights given the current lux value of the sun
 * Parameters: luxVal
 * Returns:
*/
void outdoor_light_sense(float luxVal);

/*
 * Function: shutter_motor_forwards
 * Turns on the motor in the forward direction for the shade shutter. Parameter is pre-determined to run motor for just enought time
 * to exactly stop when it reaches the end
 * Parameters: delay
 * Returns:
*/
void shutter_motor_forwards(uint16_t delay); //function to turn on the shutter motor in the forward direction

/*
 * Function:
 * Turns on the motor in the backwards direction for the shade shutter. Parameter pre-determined.
 * Parameters: delay
 * Returns:
*/
void shutter_motor_backwards(uint16_t delay);

/*
 * Function: let_there_be_light
 * Turns on the grow lights at the intensity that the user inputs as the lightvalue (0-1000)
 * Parameters: lightValue
 * Returns:
*/
void let_there_be_light(uint16_t lightvalue);

/*
 * Function: fans
 * Turns on the intake/exhaust fans at the intensity the user inputs as the fanValue
 * Parameters: fanValue
 * Returns:
*/
void fans(uint16_t fanValue);

/*
 * Function: shutter_temp_control
 * Opens/Closes the shutter based on the current temperature and shutter open/close status. After opening or closing the shutter, returns the updated
 * shutter status
 * Parameters: temp, shuuter
 * Returns: shutter
*/
uint8_t shutter_temp_control(float temp, uint8_t shutter);

/*
 * Function: fans_temp_control
 * Changes the fan speed based on temperature and humidity
 * Parameters: temp, humidity
 * Returns:
*/
void fans_temp_control(float temp, float humidity);

#endif /* INC_PWM_H_ */
