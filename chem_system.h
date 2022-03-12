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

uint16_t pH_sense(void);
uint16_t EC_sense(void);
void pH_UP(uint16_t time);
void pH_DOWN(uint16_t time);
void pH_logic(uint16_t upper, uint16_t lower);
void EC_logic(uint16_t upper, uint16_t lower);

#endif /* INC_CHEM_SYSTEM_H_ */
