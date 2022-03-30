/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define pH_ADC_IN_Pin GPIO_PIN_0
#define pH_ADC_IN_GPIO_Port GPIOA
#define fans_PWM_Pin GPIO_PIN_1
#define fans_PWM_GPIO_Port GPIOA
#define EC_ADC_IN_Pin GPIO_PIN_4
#define EC_ADC_IN_GPIO_Port GPIOA
#define moisture_ADC_IN_Pin GPIO_PIN_0
#define moisture_ADC_IN_GPIO_Port GPIOB
#define mixing_pump_OUT_Pin GPIO_PIN_8
#define mixing_pump_OUT_GPIO_Port GPIOC
#define flood_tray_OUT_Pin GPIO_PIN_9
#define flood_tray_OUT_GPIO_Port GPIOC
#define rain_pump_OUT_Pin GPIO_PIN_8
#define rain_pump_OUT_GPIO_Port GPIOA
#define light_sensor_IN_Pin GPIO_PIN_9
#define light_sensor_IN_GPIO_Port GPIOA
#define pH_Up_OUT_Pin GPIO_PIN_10
#define pH_Up_OUT_GPIO_Port GPIOA
#define pH_Down_OUT_Pin GPIO_PIN_11
#define pH_Down_OUT_GPIO_Port GPIOA
#define water_level_switch_IN_Pin GPIO_PIN_12
#define water_level_switch_IN_GPIO_Port GPIOA
#define grow_light_PWM_Pin GPIO_PIN_6
#define grow_light_PWM_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define INT_Pin GPIO_PIN_9
#define INT_GPIO_Port GPIOA
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
