/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tempsens.h"
#include "stdio.h"
#include "stdint.h"
#include "adc_3channels.h"
#include "TSL2591.h"
#include "chem_system.h"
#include "time.h"
#include "pwm.h"
#include "water_system.h"
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

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
RTC_TimeTypeDef gTime; //VERY IMPORTANT
uint16_t ADC_VAL[3];
uint16_t ADC_CH0_VAL = 0;
uint16_t ADC_CH1_VAL = 0;
uint16_t ADC_CH2_VAL = 0;

UWORD luxVal;

uint8_t plant_choose = 70; //this is the letter that will be received in order to pick which of the 6 hardcoded fuckin plants we have set up
//we are using A-F in ASCII, 65-70

struct plant
{
	float upper_pH;
	float lower_pH;
	float current_pH;
	float location; //0 = outside, 1 = inside
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/*
 * Function: set_Plant
 * Sets the user chosen plant variable, setting pH upper/lower, and outdoor/indoor setting
 * Parameters: a (received choice), *PLANT_STRUCT
 * Returns:
*/
void set_Plant(uint8_t a, struct plant *PLANT_STRUCT); //plant set function, takes in the received UART value
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float Temperature, Humidity,TomatoesTemperature,TomatoesHumidity, SunflowersTemperature, SunflowersHumidity,StrawberryTemperature, StrawberryHumidity ;
float pH, EC, moist, luxValf;
DHT_DataTypedef DHT_Data;


//TX Buffers
char myTempData[7];
char myHumData[6];
uint8_t myRxData[1];
char myMoistData[8];
char myECData[10];
char myPHData[9];
char myLuxData[15];

uint8_t minutes;
uint8_t seconds;
uint8_t hours;

float test1, test2, test3, test4, test5, test6, test7;
float EC, pH, moist; //COMMS VAR
float pH_upper, pH_lower, location;


uint8_t shutter = 0; //0 is open, 1 is closed
uint16_t count = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
	{
	  HAL_UART_Receive_IT(&huart2, myRxData, 1);
	}
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
  MX_I2C1_Init();
  MX_RTC_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start(&htim1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); //fan pwm
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); //lights pwm
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
  HAL_UART_Receive_IT(&huart2, myRxData, 1);

  DEV_ModuleInit();
  TSL2591_Init();
  set_Plant(plant_choose, &PLANT_STRUCT); //CALLS THE FUNCTION THAT SETS THE UPPER AND LOWER PH RANGES, as well as outside vs inside

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*BIG PUMP TESTING BEGIN*/
//	  flood_ON();
//	  test1 = moisture_sense();
//	  HAL_Delay(2500);
//	  flood_OFF();
//	  HAL_Delay(1000);
//	  be_like_water();
//	  HAL_Delay(2500);
//	  be_not_like_water();
	  /*BIG PUMP TESTING END*/

	  /*DOSER PUMP TESTING BEGIN*/
//	  rain_reservoir_ON();
//	  HAL_Delay(2500);
//	  rain_reservoir_OFF();
//	  HAL_Delay(1000);
//	  pH_UP(2500); //far one is up
//	  HAL_Delay(1000);
//	  pH_DOWN(2500);
	  /*DOSER PUMP TESTING END*/

	  /*SENSOR TESTING BEGIN*/
//	  pH_upper = PLANT_STRUCT.upper_pH;
//	  pH_lower = PLANT_STRUCT.lower_pH;
//	  pH = pH_sense();
//	  EC = EC_sense();
//	  moist = moisture_sense();
//	  test4 = moisture_percentage();
//	  luxVal = TSL2591_Read_Lux();
//	  readDHT22 (&DHT_Data);
//	  Temperature =  DHT_Data.tCelsius;
//	  Humidity = DHT_Data.RH;
	  /*SENSOR TESTING END*/

	  /*MOTOR AND LIGHTS TESTING BEGIN*/
//	  let_there_be_light(500);
//	  HAL_Delay(3000);
//	  let_there_be_light(0);
//	  HAL_Delay(3000);
//	  let_there_be_light(100);
//	  HAL_Delay(3000);
//	  let_there_be_light(0);
//	  HAL_Delay(3000);
//	  shutter_motor_forwards(1650);
//	  HAL_Delay(3000);
//	  shutter_motor_backwards(1650);
	  /*MOTOR AND LIGHTS TESTING BEGIN*/

	  /*FAN TESTING BEGIN*/
//	  fans(0);
//	  HAL_Delay(5000);
//	  fans(1000);
//	  HAL_Delay(5000);
	  /*FAN TESTING END*/

	  /*WATER LEVEL SWITCH BEGIN*/
//	  water_level_check();
	  /*WATER LEVEL SWITCH END*/


	  /*REAL CODE BEGIN*/
	  HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN); //this starts the RTC at what it is initialized at, 00:00:00
	  hours = gTime.Hours;
	  minutes = gTime.Minutes;
	  seconds = gTime.Seconds;

	  PLANT_STRUCT.current_pH = pH_sense(); //checking all the sensors and their values
	  pH = PLANT_STRUCT.current_pH;
	  EC = EC_sense();
	  moist = moisture_sense();
	  luxVal = TSL2591_Read_Lux();
	  readDHT22 (&DHT_Data);
	  Temperature = DHT_Data.tCelsius;
	  Humidity = DHT_Data.RH;

	  HAL_UART_Transmit(&huart2, myHumData, sprintf(myHumData, "%.1f\r\n",Humidity), 3500); //transmits all the sensor data
	  HAL_UART_Transmit(&huart2, myTempData, sprintf(myTempData, "%.2f\r\n",Temperature), 3500);
	  HAL_UART_Transmit(&huart2, myMoistData, sprintf(myMoistData, "%.1f\r\n", moist), 3500);
	  HAL_UART_Transmit(&huart2, myPHData, sprintf(myPHData, "%.5f\r\n",pH), 3500);
	  HAL_UART_Transmit(&huart2, myECData, sprintf(myECData, "%.6f\r\n",EC), 3500);
	  HAL_UART_Transmit(&huart2, myLuxData, sprintf(myLuxData, "%.7f\r\n",luxVal*1.0), 3500);
	  HAL_UART_Receive_IT(&huart2, myRxData, 1); //checks if received a value from the gateway from the user
	  set_Plant(myRxData[0], &PLANT_STRUCT); //calls the plant choosing

	  be_like_water(); //turns on mixing pump
	  water_level_check(); //checks if float switch is open (if feed reservoir low on water)

	  /*CODE THAT TURNS ON THE FLOOD PUMP IF DRY, TURNS OFF IF SATURATED*/
	  if (moist > 2600) //if kinda dry, turn on flood pump
	  {
		  flood_ON();
	  }
	  else if (moist < 1900) //if saturated enough, turn off flood pump
	  {
		  flood_OFF();
	  }
	  else
	  {

	  }

	  /*CODE FOR TURNING LIGHTS ON/OFF AT SPECIFIC TIMES*/
	  if ((hours == 0) && (PLANT_STRUCT.location == 1)) //if at 0 hours AND indoors, turn light bar on
	  {
		  let_there_be_light(1000);
	  }
	  else if ((minutes == 10) && (PLANT_STRUCT.location == 1)) //if at 18 hours AND indoors, turn light bar off
	  {
		  let_there_be_light(0);
	  }
	  else if (PLANT_STRUCT.location == 0)
	  {
		  outdoor_light_sense(luxVal);
	  }
	  else
	  {

	  }
	  /*SHUTTER + TEMP CONTROL*/
	  shutter = shutter_temp_control(Temperature, shutter);
	  fans_temp_control(Temperature, Humidity);


	  /*CODE FOR SENSING AND CORRECTING THE PH*/
	  if(PLANT_STRUCT.current_pH > PLANT_STRUCT.upper_pH) //pH correction if/else statement
	  {
		  test1 = 4;
		  while (PLANT_STRUCT.current_pH > PLANT_STRUCT.upper_pH) //while the pH is still above the upper specified boundary
		  {
			  pH_DOWN(2000); //turn on pH_Down doser pump for 2 seconds

			  while (count < 1000) //this while loop acts as a 30 second delay without having to stop all code processes (alternative to hal_delay), allows updating of live variables within a "Delay"
			  {
				  water_level_check();
				  PLANT_STRUCT.current_pH = pH_sense(); //checking all the sensors
				  pH = PLANT_STRUCT.current_pH;
				  EC = EC_sense();
				  moist = moisture_sense();
				  luxVal = TSL2591_Read_Lux();
				  readDHT22 (&DHT_Data);
				  Temperature = DHT_Data.tCelsius;
				  Humidity = DHT_Data.RH;

				  HAL_UART_Transmit(&huart2, myHumData, sprintf(myHumData, "%.1f\r\n",Humidity), 3500); //transmits all data
				  HAL_UART_Transmit(&huart2, myTempData, sprintf(myTempData, "%.2f\r\n",Temperature), 3500);
				  HAL_UART_Transmit(&huart2, myMoistData, sprintf(myMoistData, "%.1f\r\n", moist), 3500);
				  HAL_UART_Transmit(&huart2, myPHData, sprintf(myPHData, "%.5f\r\n",pH), 3500);
				  HAL_UART_Transmit(&huart2, myECData, sprintf(myECData, "%.6f\r\n",EC), 3500);
				  HAL_UART_Transmit(&huart2, myLuxData, sprintf(myLuxData, "%.7f\r\n",luxVal*1.0), 3500);
				  HAL_UART_Receive_IT(&huart2, myRxData, 1);
				  set_Plant(myRxData[0], &PLANT_STRUCT);

				  if ((hours == 0) && (PLANT_STRUCT.location == 1)) //if at 0 hours AND indoors, turn light bar on
				  {
					  let_there_be_light(1000);
				  }
				  else if ((hours == 18) && (PLANT_STRUCT.location == 1)) //if at 18 hours AND indoors, turn light bar off
				  {
					  let_there_be_light(0);
				  }
				  else if (PLANT_STRUCT.location == 0) //if outdoor setting
				  {
					  outdoor_light_sense(luxVal);
				  }
				  else
				  {

				  }

				  /*FLOOD TRAY CHECK AGAIN*/
				  if (moist > 2600) //if kinda dry, turn on flood pump
				  {
					  flood_ON();
				  }
				  else if (moist < 1900) //if saturated enough, turn off flood pump
				  {
					  flood_OFF();
				  }
				  else
				  {

				  }

//				  shutter = shutter_temp_control(Temperature, shutter);
				  fans_temp_control(Temperature, Humidity);
				  count++;
			  }

			  count = 0; //now outside of the 30 second delay, recheck all sensors
			  water_level_check();
			  PLANT_STRUCT.current_pH = pH_sense(); //updating the pH value, checking it again while inside the while loop
			  pH = PLANT_STRUCT.current_pH;
			  EC = EC_sense();
			  moist = moisture_sense();
			  luxVal = TSL2591_Read_Lux();
			  readDHT22 (&DHT_Data);
			  Temperature = DHT_Data.tCelsius;
			  Humidity = DHT_Data.RH;

			  if ((hours == 0) && (PLANT_STRUCT.location == 1)) //if at 0 hours AND indoors, turn light bar on
			  {
				  let_there_be_light(1000);
			  }
			  else if ((hours == 18) && (PLANT_STRUCT.location == 1)) //if at 18 hours AND indoors, turn light bar off
			  {
				  let_there_be_light(0);
			  }

			  else if (PLANT_STRUCT.location == 0) //if outdoor setting
			  {
				  outdoor_light_sense(luxVal);
			  }
			  else
			  {

			  }

			  /*FLOOD TRAY CHECK AGAIN*/
			  if (moist > 2600) //if kinda dry, turn on flood pump
			  {
				  flood_ON();
			  }
			  else if (moist < 1900) //if saturated enough, turn off flood pump
			  {
				  flood_OFF();
			  }
			  else
			  {

			  }

			  HAL_UART_Transmit(&huart2, myHumData, sprintf(myHumData, "%.1f\r\n",Humidity), 3500); //transmitting
			  HAL_UART_Transmit(&huart2, myTempData, sprintf(myTempData, "%.2f\r\n",Temperature), 3500);
			  HAL_UART_Transmit(&huart2, myMoistData, sprintf(myMoistData, "%.1f\r\n", moist), 3500);
			  HAL_UART_Transmit(&huart2, myPHData, sprintf(myPHData, "%.5f\r\n",pH), 3500);
			  HAL_UART_Transmit(&huart2, myECData, sprintf(myECData, "%.6g\r\n",EC), 3500);
			  HAL_UART_Transmit(&huart2, myLuxData, sprintf(myLuxData, "%.7f\r\n",luxVal*1.0), 3500);
			  HAL_UART_Receive_IT(&huart2, myRxData, 1); //receiving
			  set_Plant(myRxData[0], &PLANT_STRUCT);

			  shutter = shutter_temp_control(Temperature, shutter);
			  fans_temp_control(Temperature, Humidity);

		  }
	  }
	  else if(PLANT_STRUCT.current_pH < PLANT_STRUCT.lower_pH) //if the current pH is below the specified range
	  {
		  test2 = 8;
		  while (PLANT_STRUCT.current_pH < PLANT_STRUCT.lower_pH)
		  {
			  if (PLANT_STRUCT.current_pH < 3.5)
			  {
				  pH_UP(4000); //give a larger dose because the pH_UP solution has less effect on the lower pH values such as 3.5
			  }
			  else
			  {
				  pH_UP(2000); //turns on pH_UP doser pump for 2 seconds
			  }
			  while (count < 1000) //this while loop acts as a 30 second delay without having to stop all code processes (alternative to hal_delay)
			  {
				  water_level_check();
				  PLANT_STRUCT.current_pH = pH_sense();
				  pH = PLANT_STRUCT.current_pH;
				  EC = EC_sense();
				  moist = moisture_sense();
				  luxVal = TSL2591_Read_Lux();
				  readDHT22 (&DHT_Data);
				  Temperature = DHT_Data.tCelsius;
				  Humidity = DHT_Data.RH;

				  HAL_UART_Transmit(&huart2, myHumData, sprintf(myHumData, "%.1f\r\n",Humidity), 3500);
				  HAL_UART_Transmit(&huart2, myTempData, sprintf(myTempData, "%.2f\r\n",Temperature), 3500);
				  HAL_UART_Transmit(&huart2, myMoistData, sprintf(myMoistData, "%.1f\r\n", moist), 3500);
				  HAL_UART_Transmit(&huart2, myPHData, sprintf(myPHData, "%.5f\r\n",pH), 3500);
				  HAL_UART_Transmit(&huart2, myECData, sprintf(myECData, "%.6g\r\n",EC), 3500);
				  HAL_UART_Transmit(&huart2, myLuxData, sprintf(myLuxData, "%.7f\r\n",luxVal*1.0), 3500);
				  HAL_UART_Receive_IT(&huart2, myRxData, 1);
				  set_Plant(myRxData[0], &PLANT_STRUCT);

				  if ((hours == 0) && (PLANT_STRUCT.location == 1)) //if at 0 hours AND indoors, turn light bar on
				  {
					  let_there_be_light(1000);
				  }
				  else if ((hours == 18) && (PLANT_STRUCT.location == 1)) //if at 18 hours AND indoors, turn light bar off
				  {
					  let_there_be_light(0);
				  }
				  else if (PLANT_STRUCT.location == 0)
				  {
					  outdoor_light_sense(luxVal);
				  }
				  else
				  {

				  }

				  /*FLOOD TRAY CHECK AGAIN*/
				  if (moist > 2600) //if kinda dry, turn on flood pump
				  {
					  flood_ON();
				  }
				  else if (moist < 1900) //if saturated enough, turn off flood pump
				  {
					  flood_OFF();
				  }
				  else
				  {

				  }

				  count++;

				  shutter = shutter_temp_control(Temperature, shutter);
				  fans_temp_control(Temperature, Humidity);
			  }
			  count = 0;

			  water_level_check();
			  PLANT_STRUCT.current_pH = pH_sense(); //check the pH again, update it
			  pH = PLANT_STRUCT.current_pH;
			  EC = EC_sense();
			  moist = moisture_sense();
			  luxVal = TSL2591_Read_Lux();
			  readDHT22 (&DHT_Data);
			  Temperature = DHT_Data.tCelsius;
			  Humidity = DHT_Data.RH;

			  if ((hours == 0) && (PLANT_STRUCT.location == 1)) //if at 0 hours AND indoors, turn light bar on
			  {
				  let_there_be_light(100);
			  }
			  else if ((hours == 18) && (PLANT_STRUCT.location == 1)) //if at 18 hours AND indoors, turn light bar off
			  {
				  let_there_be_light(0);
			  }
			  else if (PLANT_STRUCT.location == 0)
			  {
				  outdoor_light_sense(luxVal);
			  }
			  else
			  {

			  }

			  /*FLOOD TRAY CHECK AGAIN*/
			  if (moist > 2600) //if kinda dry, turn on flood pump
			  {
				  flood_ON();
			  }
			  else if (moist < 1900) //if saturated enough, turn off flood pump
			  {
				  flood_OFF();
			  }
			  else
			  {


			  }

			  HAL_UART_Transmit(&huart2, myHumData, sprintf(myHumData, "%.1f\r\n",Humidity), 3500);
			  HAL_UART_Transmit(&huart2, myTempData, sprintf(myTempData, "%.2f\r\n",Temperature), 3500);
			  HAL_UART_Transmit(&huart2, myMoistData, sprintf(myMoistData, "%.1f\r\n", moist), 3500);
			  HAL_UART_Transmit(&huart2, myPHData, sprintf(myPHData, "%.5f\r\n",pH), 3500);
			  HAL_UART_Transmit(&huart2, myECData, sprintf(myECData, "%.6g\r\n",EC), 3500);
			  HAL_UART_Transmit(&huart2, myLuxData, sprintf(myLuxData, "%.7f\r\n",luxVal*1.0), 3500);
			  HAL_UART_Receive_IT(&huart2, myRxData, 1);
			  set_Plant(myRxData[0], &PLANT_STRUCT);

			  shutter = shutter_temp_control(Temperature, shutter);
			  outdoor_light_sense(luxVal);
	  		}
	  	}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
//comment out the below sign
  /* USER CODE END ADC1_Init 0 */

  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

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
 /* sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  * Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_41CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  * Configure Regular Channel

  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }*/
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
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef DateToUpdate = {0};
  RTC_AlarmTypeDef sAlarm = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_ALARM;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;

  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  DateToUpdate.WeekDay = RTC_WEEKDAY_MONDAY;
  DateToUpdate.Month = RTC_MONTH_JANUARY;
  DateToUpdate.Date = 0x1;
  DateToUpdate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &DateToUpdate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

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
  htim4.Init.Period = 1000;
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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, mixing_pump_OUT_Pin|flood_tray_OUT_Pin|Light_sensor_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, rain_pump_OUT_Pin|pH_Up_OUT_Pin|pH_Down_OUT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : mixing_pump_OUT_Pin flood_tray_OUT_Pin Light_sensor_Pin */
  GPIO_InitStruct.Pin = mixing_pump_OUT_Pin|flood_tray_OUT_Pin|Light_sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : rain_pump_OUT_Pin pH_Up_OUT_Pin pH_Down_OUT_Pin */
  GPIO_InitStruct.Pin = rain_pump_OUT_Pin|pH_Up_OUT_Pin|pH_Down_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : light_sensor_IN_Pin */
  GPIO_InitStruct.Pin = light_sensor_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(light_sensor_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : water_level_switch_IN_Pin */
  GPIO_InitStruct.Pin = water_level_switch_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(water_level_switch_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */
  GPIO_InitStruct.Pin = INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//copy and paste the gpio init code below in the gpio init function above
/*Configure GPIO pin : PtPin */
/*GPIO_InitStruct.Pin = INT_Pin;
GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
GPIO_InitStruct.Pull = GPIO_NOPULL;
HAL_GPIO_Init(INT_GPIO_Port, &GPIO_InitStruct);*/

void set_Plant(uint8_t a, struct plant *PLANT_STRUCT) // the choosing value is a float
{
	switch(a) //passes the received data from the gateway into a switch statement
	{
	case 65: //tomatoes outdoors
		PLANT_STRUCT->upper_pH = 7.5; //-> sets the value
		PLANT_STRUCT->lower_pH = 5.5;
		PLANT_STRUCT->location = 0;
		readDHT22Tomatoes(&DHT_Data);
	break;
	case 66: //sunflowers outdoors
		PLANT_STRUCT->upper_pH = 7.5;
		PLANT_STRUCT->lower_pH = 6.0;
		PLANT_STRUCT->location = 0;
		readDHT22Sunflowers(&DHT_Data);
	break;
	case 67: //strawberries outdoors
		PLANT_STRUCT->upper_pH = 6.5;
		PLANT_STRUCT->lower_pH = 4.8;
		PLANT_STRUCT->location = 0;
		readDHT22Strawberries(&DHT_Data);
	break;
	case 68: //tomatoes inside
		PLANT_STRUCT->upper_pH = 7.5; //-> sets the value
		PLANT_STRUCT->lower_pH = 5.5;
		PLANT_STRUCT->location = 1;
		readDHT22Tomatoes(&DHT_Data);
	break;
	case 69: //sunflowers inside
		PLANT_STRUCT->upper_pH = 7.5;
		PLANT_STRUCT->lower_pH = 6.0;
		PLANT_STRUCT->location = 1;
		readDHT22Sunflowers(&DHT_Data);
	break;
	case 70: //strawberries indoors
		PLANT_STRUCT->upper_pH = 9;
		PLANT_STRUCT->lower_pH = 4.8;
		PLANT_STRUCT->location = 1;
		readDHT22Strawberries(&DHT_Data);
	break;
	default:
		break;
	}
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
