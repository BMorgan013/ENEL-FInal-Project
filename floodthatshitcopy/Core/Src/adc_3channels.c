/*
 * adc_3channels.c
 *
 *      Author: Don de Jesus - 200246283
 *      This code is responsible for handling the values that are read through the 3 analog channels
 *      we're using for the pH sensor, EC sensor, and moisture sensor
 */

#include "adc_3channels.h"

uint16_t ADC_VAL[3]; //array to store values

uint16_t var = 0;
float val = 0.0;

uint16_t ADC_AVERAGE_ARRAY[10];
float sum;
float avg;
int i, j, n;

float ADC_Average(uint16_t channel)
{
sum = 0.0; //initialize sum and average variables
avg = 0.0;
	switch(channel)
	{
	case 0: //adc channel 0, PA0, pH sensor
		ADC_Select_CH0(); //calls function to select channel 0
		for (i = 0; i < 10; i++)
		{
			ADC_AVERAGE_ARRAY[i] = ADC_Read_CH0(); //fills an array with 10 analog values
		}
		for (j = 0; j < 10; j++)
		{
			sum += ADC_AVERAGE_ARRAY[j]; //adds up the array's analog values
		}
		avg = sum/10; //finds the average of the array
		return avg; //returns the average
	break;

	case 1: //adc channel 1, PA4, EC sensor
		ADC_Select_CH1();
		for (i = 0; i < 10; i++)
		{
			ADC_AVERAGE_ARRAY[i] = ADC_Read_CH1();
		}
		for (j = 0; j < 10; j++)
		{
			sum += ADC_AVERAGE_ARRAY[j];
		}
		avg = sum/10;
		return avg;
	break;

	case 2: //adc channel 2, PB0, moisture sensor
		ADC_Select_CH2();
		for (i = 0; i < 10; i++)
		{
			ADC_AVERAGE_ARRAY[i] = ADC_Read_CH2();
		}
		for (j = 0; j < 10; j++)
		{
			sum += ADC_AVERAGE_ARRAY[j];
		}
		avg = sum/10;
		return avg;
	break;
	default:
		return avg;
	}
	return avg;
}

void ADC_Select_CH0() //pa0
{
	ADC_ChannelConfTypeDef sConfig = {0}; //it wouldn't work without this
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

void ADC_Select_CH1() //pa4
{
	ADC_ChannelConfTypeDef sConfig = {0};
	  /** Configure Regular Channel
	  */
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1; //ADC_REGULAR_RANK_8
	  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
	  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_CH2() //pb0
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
    HAL_ADC_Stop(&hadc1); //stops adc

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

