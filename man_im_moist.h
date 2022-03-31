/*
 * man_im_moist.h
 *
 *  Created on: Mar. 11, 2022
 *      Author: rdeje
 */

#ifndef INC_MAN_IM_MOIST_H_
#define INC_MAN_IM_MOIST_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"
#include "main.h"
#include "adc_3channels.h"
#include "chem_system.h"

void rain_reservoir_ON(void);
void rain_reservoir_OFF(void);
void flood_ON(void); //turns on the flood pump PC9
void flood_OFF(void); //turns off the flood pump
void be_like_water(void); //function responsible for powering the perpetually persistent pump to mix the reservoir
void be_not_like_water(void); //turns off mixer
float moisture_sense(void); //function responsible for moisture sensing
void flood_logic(void); //function responsible for decision making with the moisture level
void water_level_check(void); /*checks the status of the water level switch on PA12
if the water level switch is open, this means that the water level is low, beneath the switch
in this case, send a signal to the user
i am feeding 3.3V through the switch to a GPIO input, once the water level dips below the switch, the normally closed
switch will open, interrupting the GPIO and making it go low.
At this point, a warning to the user will be sent*/


#endif /* INC_MAN_IM_MOIST_H_ */
