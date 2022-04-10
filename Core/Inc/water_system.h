/*
 * man_im_moist.h
 *
 *  Created on: Mar. 11, 2022
 *      Author: Don de Jesus - 200246283
 *      This code is responsible for all the functions regarding the watering system
 */

#ifndef INC_WATER_SYSTEM_H_
#define INC_WATER_SYSTEM_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "main.h"
#include "adc_3channels.h"
#include "chem_system.h"

/*
 * Function:
 *
 * Parameters:
 * Returns:
*/

/*
 * Function: rain_reservoir_ON/OFF
 * Turns on/off the rain water transfer doser pump to the feed tank
 * Parameters:
 * Returns:
*/
void rain_reservoir_ON(void);
void rain_reservoir_OFF(void);

/*
 * Function: flood_ON/OFF
 * Tursn on/off the flood pump to the flood tray
 * Parameters:
 * Returns:
*/
void flood_ON(void); //turns on the flood pump PC9
void flood_OFF(void); //turns off the flood pump

/*
 * Function: be_like_water/be_not_like_water
 * Turns on/off the mixing pump
 * Parameters:
 * Returns:
*/
void be_like_water(void); //function responsible for powering the perpetually persistent pump to mix the reservoir
void be_not_like_water(void); //turns off mixer

/*
 * Function: moisture_sense
 * Returns the moisture analog value on the ADC pin connected to the moisture sensor
 * Parameters:
 * Returns: moisture
*/
float moisture_sense(void); //function responsible for moisture sensing

/*
 * Function: water_level_check
 * Checks the status of the water level switch in the feed tank. If low water, run water from rain to feed tank.
 * Parameters:
 * Returns:
*/
void water_level_check(void);

#endif /* INC_WATER_SYSTEM_H_ */
