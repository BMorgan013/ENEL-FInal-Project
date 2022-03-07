/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define EC_sensor_ADC_in11_Pin GPIO_PIN_1
#define EC_sensor_ADC_in11_GPIO_Port GPIOC
#define moisture_sensor_ADC_in0_Pin GPIO_PIN_0
#define moisture_sensor_ADC_in0_GPIO_Port GPIOA
#define pH_sensor_ADC_in8_Pin GPIO_PIN_0
#define pH_sensor_ADC_in8_GPIO_Port GPIOB
#define mixing_gpio_out_Pin GPIO_PIN_8
#define mixing_gpio_out_GPIO_Port GPIOC
#define flood_tray_gpio_out_Pin GPIO_PIN_9
#define flood_tray_gpio_out_GPIO_Port GPIOC
#define doser_water_gpio_out_Pin GPIO_PIN_8
#define doser_water_gpio_out_GPIO_Port GPIOA
#define doser_fertilizer_gpio_out_Pin GPIO_PIN_9
#define doser_fertilizer_gpio_out_GPIO_Port GPIOA
#define doser_ph_down_gpio_down_Pin GPIO_PIN_10
#define doser_ph_down_gpio_down_GPIO_Port GPIOA
#define doser_ph_up_gpio_out_Pin GPIO_PIN_11
#define doser_ph_up_gpio_out_GPIO_Port GPIOA
#define level_switch_gpio_in_Pin GPIO_PIN_12
#define level_switch_gpio_in_GPIO_Port GPIOA
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
