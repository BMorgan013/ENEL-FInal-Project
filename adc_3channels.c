/*
 * adc_3channels.c
 *
 *  Created on: Mar 6, 2022
 *      Author: Don de Jesus - 200246283
 */

#include "adc_3channels.h"

uint16_t ADC_VAL[3]; //array to store values

uint16_t var = 0;
float val = 0.0;

void ADC_Select_CH0()
{
	ADC_ChannelConfTypeDef sConfig = {0}; //it wouldn't work without this, idk what it is
	 /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_0; //this code was taken from the MX_ADC1_Init function in main.c
	  sConfig.Rank = 1; //ADC_REGULAR_RANK_1
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH1()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1; //ADC_REGULAR_RANK_8
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2()
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_8;
	  sConfig.Rank = 1; //ADC_REGULAR_RANK_3;
	  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

uint16_t ADC_Read_CH0() //reads and returns the analog value on the channel
{
	ADC_Select_CH0(); //selects the channel to be read
	HAL_ADC_Start(&hadc1); //starts ADC, waits for it to be finished, then stores the analog value into an array
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[0] = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return ADC_VAL[0]; //returns the analog value
}

uint16_t ADC_Read_CH1()
{
	ADC_Select_CH1();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[1] = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return ADC_VAL[1];
}

uint16_t ADC_Read_CH2()
{
	ADC_Select_CH2();
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	ADC_VAL[2] = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    return ADC_VAL[2];
}

