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
#define pH_Sensor_IN_Pin GPIO_PIN_0
#define pH_Sensor_IN_GPIO_Port GPIOA
#define PWM_Fans_Pin GPIO_PIN_1
#define PWM_Fans_GPIO_Port GPIOA
#define EC_Sensor_IN_Pin GPIO_PIN_4
#define EC_Sensor_IN_GPIO_Port GPIOA
#define Moisture_Sensor_IN_Pin GPIO_PIN_0
#define Moisture_Sensor_IN_GPIO_Port GPIOB
#define Flood_Tray_OUT_Pin GPIO_PIN_9
#define Flood_Tray_OUT_GPIO_Port GPIOC
#define Doser_Water_OUT_Pin GPIO_PIN_8
#define Doser_Water_OUT_GPIO_Port GPIOA
#define Doser_Fertilizer_OUT_Pin GPIO_PIN_9
#define Doser_Fertilizer_OUT_GPIO_Port GPIOA
#define Doser_pH_DOWN_OUT_Pin GPIO_PIN_10
#define Doser_pH_DOWN_OUT_GPIO_Port GPIOA
#define Doser_pH_UP_OUT_Pin GPIO_PIN_11
#define Doser_pH_UP_OUT_GPIO_Port GPIOA
#define Level_Switch_IN_Pin GPIO_PIN_12
#define Level_Switch_IN_GPIO_Port GPIOA
#define Light_Sensor_IN_Pin GPIO_PIN_3
#define Light_Sensor_IN_GPIO_Port GPIOB
#define PWM_Motor_Forward_Pin GPIO_PIN_4
#define PWM_Motor_Forward_GPIO_Port GPIOB
#define PWM_Motor_Reverse_Pin GPIO_PIN_5
#define PWM_Motor_Reverse_GPIO_Port GPIOB
#define PWM_Lights_Pin GPIO_PIN_6
#define PWM_Lights_GPIO_Port GPIOB
#define Temp_Hum_OUT_Pin GPIO_PIN_9
#define Temp_Hum_OUT_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
