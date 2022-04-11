/*****************************************************************************
UNIVERSITY OF REGINA FACULTY OF APPLIED SCIENCE & ENGINEERING
Title: Group 5 - Automated Greenhouse System Capstone
Link: https://www.micropeta.com/video48
Datasheet: https://cdn-shop.adafruit.com/datasheets/Digital+humidity+and+temperature+sensor+AM2302.pdf
Comments: We are using Nizar Mohideen's DHT22 library from MicroPeta
******************************************************************************/

#include "stm32f1xx_hal.h"
#include "tempsens.h"
#define DHT22_PORT GPIOC
#define DHT22_PIN GPIO_PIN_10

uint32_t pMillis, cMillis;

//DHT_DataTypedef DHT_Data;
//float Temperature1, Humidity1;

uint8_t RH1, RH2, TC1, TC2, SUM, CHECK;
float tCelsius = 0;
float tFahrenheit = 0;
float RH = 0;

void microDelay (uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(&htim1, 0);
  while (__HAL_TIM_GET_COUNTER(&htim1) < delay);
}

uint8_t DHT22_Start (void)
{
  uint8_t Response = 0;
  GPIO_InitTypeDef GPIO_InitStructPrivate = {0};
  GPIO_InitStructPrivate.Pin = DHT22_PIN;
  GPIO_InitStructPrivate.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructPrivate.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStructPrivate.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as output
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 0);   // pull the pin low
  microDelay (1300);   // wait for 1300us
  HAL_GPIO_WritePin (DHT22_PORT, DHT22_PIN, 1);   // pull the pin high
  microDelay (30);   // wait for 30us
  GPIO_InitStructPrivate.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructPrivate.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStructPrivate); // set the pin as input
  microDelay (40);
  if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))
  {
    microDelay (80);
    if ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN))) Response = 1;
  }
  pMillis = HAL_GetTick();
  cMillis = HAL_GetTick();
  while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
  {
    cMillis = HAL_GetTick();
  }
  return Response;
}

uint8_t DHT22_Read (void)
{
  uint8_t a,b;
  for (a=0;a<8;a++)
  {
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go high
      cMillis = HAL_GetTick();
    }
    microDelay (40);   // wait for 40 us
    if (!(HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)))   // if the pin is low
      b&= ~(1<<(7-a));
    else
      b|= (1<<(7-a));
    pMillis = HAL_GetTick();
    cMillis = HAL_GetTick();
    while ((HAL_GPIO_ReadPin (DHT22_PORT, DHT22_PIN)) && pMillis + 2 > cMillis)
    {  // wait for the pin to go low
      cMillis = HAL_GetTick();
    }
  }
  return b;
}

void readDHT22 (DHT_DataTypedef *DHT_Data)
{
	if(DHT22_Start())
    {
      RH1 = DHT22_Read(); // First 8bits of humidity
      RH2 = DHT22_Read(); // Second 8bits of Relative humidity
      TC1 = DHT22_Read(); // First 8bits of Celsius
      TC2 = DHT22_Read(); // Second 8bits of Celsius
      SUM = DHT22_Read(); // Check sum
      CHECK = RH1 + RH2 + TC1 + TC2;
      if (CHECK == SUM)
      {
        if (TC1>127) // If TC1=10000000, negative temperature
        {
        	DHT_Data->tCelsius = (float)TC2/10*(-1);
        }
        else
        {
        	DHT_Data->tCelsius = (float)((TC1<<8)|TC2)/10;
        }
        //DHT_Data->tFahrenheit = tCelsius * 9/5 + 32;
        DHT_Data->RH = (float) ((RH1<<8)|RH2)/10;
      }
      HAL_Delay(1);
    }

     /**********************PWM Fans************************/

	 //if temperature gets to 18C and humidity 50%, shut off fans
//	if (DHT_Data->tCelsius <= 18 && DHT_Data->RH <= 50)
//	{
//		TIM2->CCR2 = 10;
//		HAL_Delay(1);
//	}
//		 //if temperature gets to 23C and humidity 75%, run fans at 50%
//	else if ((DHT_Data->tCelsius >= 19 && DHT_Data->RH <= 75) || (DHT_Data->tCelsius <= 24 && DHT_Data->RH <= 75))
//	{
//		TIM2->CCR2 = 500;
//		HAL_Delay(1);
//	}
//	//if temperature exceeds 24C and above 75%, run fans at full bore
//	else
//	{
//	TIM2->CCR2 = 1000;
//	HAL_Delay(1);
//	}

}
/*******************************************/
void readDHT22Tomatoes (DHT_DataTypedef *DHT_Data)
{
	if(DHT22_Start())
    {
      RH1 = DHT22_Read(); // First 8bits of humidity
      RH2 = DHT22_Read(); // Second 8bits of Relative humidity
      TC1 = DHT22_Read(); // First 8bits of Celsius
      TC2 = DHT22_Read(); // Second 8bits of Celsius
      SUM = DHT22_Read(); // Check sum
      CHECK = RH1 + RH2 + TC1 + TC2;
      if (CHECK == SUM)
      {
        if (TC1>127) // If TC1=10000000, negative temperature
        {
        	DHT_Data->tCelsius = (float)TC2/10*(-1);
        }
        else
        {
        	DHT_Data->tCelsius = (float)((TC1<<8)|TC2)/10;
        }
        //DHT_Data->tFahrenheit = tCelsius * 9/5 + 32;
        DHT_Data->RH = (float) ((RH1<<8)|RH2)/10;
      }
      HAL_Delay(1);
    }

     /**********************PWM Fans************************/

	 //if temperature gets to 18C and humidity 50%, shut off fans
	if (DHT_Data->tCelsius <= 20 && DHT_Data->RH <= 50)
		{
			TIM2->CCR2 = 10;
			HAL_Delay(1);
		}
			 //if temperature gets to 23C and humidity 75%, run fans at 50%
		else if ((DHT_Data->tCelsius >= 20 && DHT_Data->RH <= 60) || (DHT_Data->tCelsius <= 24 && DHT_Data->RH <= 60))
		{
			TIM2->CCR2 = 500;
			HAL_Delay(1);
		}
		//if temperature exceeds 24C and above 75%, run fans at full bore
		else
		{
		TIM2->CCR2 = 1000;
		HAL_Delay(1);
		}


	/**********SHADE MOTOR****************/
	//temperature 18 and below, ensure shaders are open
	//Motor Reverse
	/* if(DHT_Data->tCelsius <= 22)
	{
	 TIM3->CCR1 = 200;
	 HAL_Delay(1); //need to figure out delay
	}
	//temperature 25 and above, close shaders to help reduce temp
	//Motor Forward
	else if (DHT_Data->tCelsius >= 25)
	{
	 TIM3->CCR2 = 200;
	 HAL_Delay(1);
	}
	//otherwise motors are off
	else
	{
	TIM3->CCR1 = 0;
	TIM3->CCR2 = 0;
	HAL_Delay(1);
	}*/
}
/*******************************************/
void readDHT22Strawberries (DHT_DataTypedef *DHT_Data)
{
	if(DHT22_Start())
    {
      RH1 = DHT22_Read(); // First 8bits of humidity
      RH2 = DHT22_Read(); // Second 8bits of Relative humidity
      TC1 = DHT22_Read(); // First 8bits of Celsius
      TC2 = DHT22_Read(); // Second 8bits of Celsius
      SUM = DHT22_Read(); // Check sum
      CHECK = RH1 + RH2 + TC1 + TC2;
      if (CHECK == SUM)
      {
        if (TC1>127) // If TC1=10000000, negative temperature
        {
        	DHT_Data->tCelsius = (float)TC2/10*(-1);
        }
        else
        {
        	DHT_Data->tCelsius = (float)((TC1<<8)|TC2)/10;
        }
        //DHT_Data->tFahrenheit = tCelsius * 9/5 + 32;
        DHT_Data->RH = (float) ((RH1<<8)|RH2)/10;
      }
      HAL_Delay(1);
    }

     /**********************PWM Fans************************/

	 //if temperature gets to 17C and humidity 50%, shut off fans
	if (DHT_Data->tCelsius <= 16 && DHT_Data->RH <= 50)
			{
				TIM2->CCR2 = 10;
				HAL_Delay(1);
			}
				 //if temperature gets to 23C and humidity 75%, run fans at 50%
			else if ((DHT_Data->tCelsius >= 17 && DHT_Data->RH <= 60) || (DHT_Data->tCelsius <= 27 && DHT_Data->RH <= 60))
			{
				TIM2->CCR2 = 500;
				HAL_Delay(1);
			}
			//if temperature exceeds 24C and above 75%, run fans at full bore
			else
			{
			TIM2->CCR2 = 1000;
			HAL_Delay(1);
			}



	/**********SHADE MOTOR****************/
	//temperature 18 and below, ensure shaders are open
	//Motor Reverse
//	 if(DHT_Data->tCelsius <= 22)
//	{
//	 TIM3->CCR1 = 200;
//	 HAL_Delay(1); //need to figure out delay
//	}
//	//temperature 25 and above, close shaders to help reduce temp
//	//Motor Forward
//	else if (DHT_Data->tCelsius >= 25)
//	{
//	 TIM3->CCR2 = 200;
//	 HAL_Delay(1);
//	}
//	//otherwise motors are off
//	else
//	{
//	TIM3->CCR1 = 0;
//	TIM3->CCR2 = 0;
//	HAL_Delay(1);
//	}
}
/*******************************************/
void readDHT22Sunflowers (DHT_DataTypedef *DHT_Data)
{
	if(DHT22_Start())
    {
      RH1 = DHT22_Read(); // First 8bits of humidity
      RH2 = DHT22_Read(); // Second 8bits of Relative humidity
      TC1 = DHT22_Read(); // First 8bits of Celsius
      TC2 = DHT22_Read(); // Second 8bits of Celsius
      SUM = DHT22_Read(); // Check sum
      CHECK = RH1 + RH2 + TC1 + TC2;
      if (CHECK == SUM)
      {
        if (TC1>127) // If TC1=10000000, negative temperature
        {
        	DHT_Data->tCelsius = (float)TC2/10*(-1);
        }
        else
        {
        	DHT_Data->tCelsius = (float)((TC1<<8)|TC2)/10;
        }
        //DHT_Data->tFahrenheit = tCelsius * 9/5 + 32;
        DHT_Data->RH = (float) ((RH1<<8)|RH2)/10;
      }
      HAL_Delay(1);
    }

     /**********************PWM Fans************************/

	//if temperature gets to 17C and humidity 50%, shut off fans
	if (DHT_Data->tCelsius <= 21 && DHT_Data->RH <= 50)
			{
				TIM2->CCR2 = 10;
				HAL_Delay(1);
			}
				 //if temperature gets to 23C and humidity 75%, run fans at 50%
			else if ((DHT_Data->tCelsius >= 22 && DHT_Data->RH <= 70) || (DHT_Data->tCelsius <= 26 && DHT_Data->RH <= 70))
			{
				TIM2->CCR2 = 500;
				HAL_Delay(1);
			}
			//if temperature exceeds 24C and above 75%, run fans at full bore
			else
			{
			TIM2->CCR2 = 1000;
			HAL_Delay(1);
			}



	/**********SHADE MOTOR****************/
	//temperature 18 and below, ensure shaders are open
	//Motor Reverse
//	 if(DHT_Data->tCelsius <= 22)
//	{
//	 TIM3->CCR1 = 200;
//	 HAL_Delay(1); //need to figure out delay
//	}
//	//temperature 25 and above, close shaders to help reduce temp
//	//Motor Forward
//	else if (DHT_Data->tCelsius >= 25)
//	{
//	 TIM3->CCR2 = 200;
//	 HAL_Delay(1);
//	}
//	//otherwise motors are off
//	else
//	{
//	TIM3->CCR1 = 0;
//	TIM3->CCR2 = 0;
//	HAL_Delay(1);
//	}
}
/*******************************************/


