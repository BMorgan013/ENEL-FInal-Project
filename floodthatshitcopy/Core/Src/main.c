/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc_3channels.h"
#include "chem_system.h"
#include "man_im_moist.h"
#include "TSL2591.h"

#include <stdlib.h>
#include <stdio.h>
#include "stdint.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint16_t ADC_VAL[3];
uint16_t ADC_CH0_VAL = 0;
uint16_t ADC_CH1_VAL = 0;
uint16_t ADC_CH2_VAL = 0;

char s[100] = "4.08982";
float test = 0.0;
float test2 = 0.0;
float test3 = 0.0;
float test4 = 0.0;
float upper = 7.5;
float lower = 5.5;

uint16_t count = 0;
UWORD luxVal;

uint16_t plant_choose = 2;

struct plant
{
	float upper_pH;
	float lower_pH;
	float current_pH;
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
void set_Plant(uint16_t a, struct plant *PLANT_STRUCT);
//void correct_pH(struct plant *PLANT_STRUCT);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	struct plant PLANT_STRUCT;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */
  set_Plant(plant_choose, &PLANT_STRUCT); //CALLS THE FUNCTION THAT SETS THE UPPER AND LOWER PH RANGES
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);

	DEV_ModuleInit();

	TSL2591_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  ADC_CH0_VAL = ADC_Read_CH0(); //pH sensor
	  ADC_CH1_VAL = ADC_Read_CH1(); //EC sensor
	  ADC_CH2_VAL = ADC_Read_CH2(); //moisture sensor

	  test2 = PLANT_STRUCT.current_pH; //this works, checks the struct values (.current_pH checks the value)
	  test3 = PLANT_STRUCT.upper_pH;
	  test4 = PLANT_STRUCT.lower_pH;

	  //pH_UP(2000);
//	  TIM4->CCR1 = 500;
//	  	HAL_Delay(1);
	  luxVal = TSL2591_Read_Lux();

//	  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 200); //update pwm value
//	  HAL_Delay(1000);
//	  __HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, 500); //update pwm value
//	  HAL_Delay(1000);
//
//	  for (pwm=0;pwm<=3906;pwm=pwm+1)  //darkest to brightest: 0-100% duty cycle
//			{
//			__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_1, pwm); //update pwm value
//			HAL_Delay(5);           //Wait for 5ms
//			}

		/*if(PLANT_STRUCT.current_pH > PLANT_STRUCT.upper_pH) //pH correction if/else statement
			{
				while (PLANT_STRUCT.current_pH > PLANT_STRUCT.upper_pH) //while the pH is still above the upper specified boundary
				{
					pH_DOWN(2000); //turn on pH_Down doser pump for 2 seconds

					while (count < 1000) //this while loop acts as a 30 second delay without having to stop all code processes (alternative to hal_delay)
					{
						PLANT_STRUCT.current_pH = pH_sense(); //lets you update live variables within a "delay"
						test = PLANT_STRUCT.current_pH;
						//check temp
						//check moisture
						//check
						count++;
					}
					count = 0;
					PLANT_STRUCT.current_pH = pH_sense(); //check the pH again, update it
					test = PLANT_STRUCT.current_pH;
				}
			}

			else if(PLANT_STRUCT.current_pH < PLANT_STRUCT.lower_pH)
			{
				while (PLANT_STRUCT.current_pH < PLANT_STRUCT.lower_pH)
				{
					if (PLANT_STRUCT.current_pH < 3.5)
					{
						pH_UP(4000); //give a larger dose because the pH_UP solution has less effect on the lower pH values
					}
					else
					{
						pH_UP(2000); //turns on pH_UP doser pump for 2 seconds
					}
					while (count < 1000) //this while loop acts as a 30 second delay without having to stop all code processes (alternative to hal_delay)
					{
						PLANT_STRUCT.current_pH = pH_sense(); //lets you update live variables within a "delay"
						test = PLANT_STRUCT.current_pH;
						//check temp
						//check moisture
						//check
						count++;
					}
					count = 0;

					PLANT_STRUCT.current_pH = pH_sense(); //check the pH again, update it
					test = PLANT_STRUCT.current_pH;
				}
			}
*/
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */
//comment out the line beneath
  /* USER CODE END ADC1_Init 0 */

  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */
//CHANGE NBR OF CONVERSION FROM 3 TO 1
  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  /*sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  * Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  * Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();*/
  //}
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5|doser_water_gpio_out_Pin|light_sensor_out_Pin|doser_ph_down_gpio_down_Pin
                          |doser_ph_up_gpio_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, mixing_gpio_out_Pin|flood_tray_gpio_out_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA5 doser_water_gpio_out_Pin light_sensor_out_Pin doser_ph_down_gpio_down_Pin
                           doser_ph_up_gpio_out_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_5|doser_water_gpio_out_Pin|light_sensor_out_Pin|doser_ph_down_gpio_down_Pin
                          |doser_ph_up_gpio_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : mixing_gpio_out_Pin flood_tray_gpio_out_Pin */
  GPIO_InitStruct.Pin = mixing_gpio_out_Pin|flood_tray_gpio_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : level_switch_gpio_in_Pin */
  GPIO_InitStruct.Pin = level_switch_gpio_in_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(level_switch_gpio_in_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */ //this is the interrupt pin on pa9
  GPIO_InitStruct.Pin = light_sensor_out_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(light_sensor_out_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//

/**
* @brief plant choosing function,
* @param
* a, user chosen plant value
* PLANT_STRUCT
* This function takes the value received from that myRxData shit and uses it in a switch statement
* @retval None
*/
void set_Plant(uint16_t a, struct plant *PLANT_STRUCT) // the choosing value is a float
{
	switch(a)
	{
	case 0: //tomatoes
		PLANT_STRUCT->upper_pH = 7.5; //-> sets the value
		PLANT_STRUCT->lower_pH = 5.5;
	break;
	case 1: //sunflowers
		PLANT_STRUCT->upper_pH = 7.5;
		PLANT_STRUCT->lower_pH = 6.0;
	break;
	case 2: //potato
		PLANT_STRUCT->upper_pH = 6.5;
		PLANT_STRUCT->lower_pH = 4.8;
	break;
	default:
		break;
	}
}

/**
* @brief pH correction function
* @param plant struct
* @retval None
*/

/*void correct_pH(struct plant PLANT_STRUCT) //new pH correction function with structs and pointers
{
//	float *fptr;
//	fptr = &(PLANT_STRUCT.current_pH); //tells the float pointer to point at the current pH in the plant struct
	if(PLANT_STRUCT->current_pH > PLANT_STRUCT->upper_pH)
		{
			while (PLANT_STRUCT->current_pH > PLANT_STRUCT->upper_pH) //while the pH is still above the upper specified boundary
			{
				pH_DOWN(2000); //turn on pH_Down doser pump for 2 seconds
				HAL_Delay(20000); //20 second delay for mixing
				PLANT_STRUCT->current_pH = pH_sense(); //check the pH again, update it
			}
		}

		else if(PLANT_STRUCT->current_pH < PLANT_STRUCT->lower_pH)
		{
			while (PLANT_STRUCT->current_pH < PLANT_STRUCT->lower_pH)
			{
				pH_UP(2000); //turns on pH_UP doser pump for 2 seconds
				HAL_Delay(10000); //20 second delay for mixing
				PLANT_STRUCT->current_pH = pH_sense(); //check the pH again, update it
			}
		}
}*/

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

