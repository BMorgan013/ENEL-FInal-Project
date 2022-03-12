/*
 * pH_system.c
 *
 *  Created on: Mar. 12, 2022
 *      Author: rdeje
 */

#include "chem_system.h"

uint16_t pH_sense(void) //pa0
{
	uint16_t pH = 0;
	ADC_Select_CH0(); //selects pa0 analog adc, then returns what the pin sees
	pH = ADC_Read_CH();
	return pH;
}

uint16_t EC_sense(void) //pa0
{
	uint16_t EC = 0;
	ADC_Select_CH0(); //selects pa0 analog adc, then returns what the pin sees
	EC = ADC_Read_CH();
	return EC;
}

void pH_UP(uint16_t time) //pa11
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
	HAL_Delay(time);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
}

void pH_DOWN(uint16_t time) //pa10
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
	HAL_Delay(time);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
}

void pH_logic(uint16_t upper, uint16_t lower)
{
	uint16_t pH = 0;
	pH = pH_sense();

	//equation to convert analog to pH value, OR THE OPPOSITE, CONVERT UPPER AND LOWER INTO ANALOG VALUES
	//using the analog values might be a pain in the ass but it'll probably be better for accuracy

	//function below lowers the pH of the feed solution, if the current pH was higher than the specified range
	if(pH > upper) //if pH value is above the upper specified range, lower pH till less than upper (MAYBE BREAK THIS INTO A PH_UPPER FORMULA)
	{
		while(pH > upper) //while current pH above upper pH range
			{
			/*
			ADD LOGIC:

			uint16_t reference = 0;

			find out how far the gap is from the current pH to the upper boundary


			if(the gap to the nearest boundary is greater than 5.0pH away)
			{
				while(gap between the current value and target value is less than over 5.0)
				{
					turn on the pump to do -5.0pH
					check the current pH value
					check the gap, will exit loop if gap to nearest boundary is less than 5.0
					UPDATE GAP VALUE
				}
			else if(gap to nearest boundary less than 5.0 AND over 2.5)
			{
				while(gap to nearest boundary less than 5.0 AND over 2.5)
				turn on pump to do -5.0pH
				check current pH value
				check gap, will exit loop if gap to nearest boundary less than 2.5
				UPDATE GAP VALUE
			}
			else if (gap to nearest boundary less than 5.0 away AND less than 2.5 AND over 1.0 away)
			{
				while (gap to nearest boundary less than 5.0 AND over 2.5)
				turn on pump to do -1.0pH
				check current pH value
				check gap, exits loop if gap to nearest boundary less than 1.0
				UPDATE GAP VALUE
			}
			else
			{
			it's within the specified range!!
			note: this may result in a pH value sticking to one side of the user provided range
			that's ok tho maybe we can just specify that the range can't be too broad
			}
			*/
			}
	}
	else if(pH < lower) //while current pH below pH lower range
	{
		while(pH > upper) //while current above upper pH range
			{
			/*
			ADD LOGIC:

			uint16_t reference = 0;

			find out which boundary the current pH value is closest to
			track that gap with a variable

			if(the gap to the nearest boundary is greater than 5.0pH away)
			{
				while(gap between the current value and target value is less than over 5.0)
				{
					turn on the pump to do +5.0pH
					check the current pH value
					check the gap, will exit loop if gap to nearest boundary is less than 5.0
					UPDATE GAP VALUE
				}
			else if(gap to nearest boundary less than 5.0 AND over 2.5)
			{
				while(gap to nearest boundary less than 5.0 AND over 2.5)
				turn on pump to do +2.5pH
				check current pH value
				check gap, will exit loop if gap to nearest boundary less than 2.5
				UPDATE GAP VALUE
			}
			else if (gap to nearest boundary less than 5.0 away AND less than 2.5 AND over 1.0 away)
			{
				while (gap to nearest boundary less than 5.0 AND over 2.5)
				turn on pump to do +1.0pH
				check current pH value
				check gap, exits loop if gap to nearest boundary less than 1.0
			}
			else
			{
			it's within the specified range!!
			note: this may result in a pH value sticking to one side of the user provided range
			that's ok tho maybe we can just specify that the range can't be too broad
			}
			*/
			}
	}
	else
	{

	}
}

void EC_logic(uint16_t upper, uint16_t lower)
{
	uint16_t EC = 0;
		EC = EC_sense();

		//equation to convert analog to EC value, OR THE OPPOSITE, CONVERT UPPER AND LOWER INTO ANALOG VALUES
		//using the analog values might be a pain in the ass but it'll probably be better for accuracy

		//function below lowers the EC of the feed solution, if the current pH/EC was higher than the specified range
		if(EC > upper) //if EC value is above the upper specified range, lower pH/EC till less than upper (MAYBE BREAK THIS INTO A EC_UPPER FORMULA)
		{
			while(EC > upper) //while current EC above upper pH range
				{
				/*
				ADD LOGIC:

				uint16_t reference = 0;

				find out which boundary the current EC value is closest to
				track that gap with a variable

				if(the gap to the nearest boundary is greater than 5.0EC away)
				{
					while(gap between the current value and target value is less than over 5.0)
					{
						turn on the pump to do -5.0EC
						check the current EC value
						check the gap, will exit loop if gap to nearest boundary is less than 5.0EC
						UPDATE GAP VALUE
					}
				else if(gap to nearest boundary less than 5.0 AND over 2.5)
				{
					while(gap to nearest boundary less than 5.0 AND over 2.5)
					turn on pump to do -5.0pH
					check current EC value
					check gap, will exit loop if gap to nearest boundary less than 2.5
					UPDATE GAP VALUE
				}
				else if (gap to nearest boundary less than 5.0 away AND less than 2.5 AND over 1.0 away)
				{
					while (gap to nearest boundary less than 5.0 AND over 2.5)
					turn on pump to do -1.0pH
					check current EC value
					check gap, exits loop if gap to nearest boundary less than 1.0
					UPDATE GAP VALUE
				}
				else
				{
				it's within the specified range!!
				note: this may result in a EC value sticking to one side of the user provided range
				that's ok tho maybe we can just specify that the range can't be too broad
				}
				*/
				}
		}
		else if(EC < lower) //while current EC below EC lower range
		{
			while(EC > upper) //while current above upper EC range
				{
				/*
				ADD LOGIC:

				uint16_t reference = 0;

				find out which boundary the current EC value is closest to
				track that gap with a variable

				if(the gap to the nearest boundary is greater than 5.0EC away)
				{
					while(gap between the current value and target value is less than over 5.0)
					{
						turn on the pump to do +5.0pH
						check the current EC value
						check the gap, will exit loop if gap to nearest boundary is less than 5.0
						UPDATE GAP VALUE
					}
				else if(gap to nearest boundary less than 5.0 AND over 2.5)
				{
					while(gap to nearest boundary less than 5.0 AND over 2.5)
					turn on pump to do +2.5pH
					check current EC value
					check gap, will exit loop if gap to nearest boundary less than 2.5
					UPDATE GAP VALUE
				}
				else if (gap to nearest boundary less than 5.0 away AND less than 2.5 AND over 1.0 away)
				{
					while (gap to nearest boundary less than 5.0 AND over 2.5)
					turn on pump to do +1.0pH
					check current EC value
					check gap, exits loop if gap to nearest boundary less than 1.0
				}
				else
				{
				it's within the specified range!!
				note: this may result in a EC value sticking to one side of the user provided range
				that's ok tho maybe we can just specify that the range can't be too broad
				}
				*/
				}
		}
		else
		{

		}
}
