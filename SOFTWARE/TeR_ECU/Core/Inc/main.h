/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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
#define DOUT1_Pin GPIO_PIN_2
#define DOUT1_GPIO_Port GPIOE
#define DOUT0_Pin GPIO_PIN_3
#define DOUT0_GPIO_Port GPIOE
#define DIN2_Pin GPIO_PIN_4
#define DIN2_GPIO_Port GPIOE
#define DIN1_Pin GPIO_PIN_5
#define DIN1_GPIO_Port GPIOE
#define DIN0_Pin GPIO_PIN_6
#define DIN0_GPIO_Port GPIOE
#define AIN2_Pin GPIO_PIN_0
#define AIN2_GPIO_Port GPIOA
#define AIN3_Pin GPIO_PIN_1
#define AIN3_GPIO_Port GPIOA
#define AIN0_Pin GPIO_PIN_2
#define AIN0_GPIO_Port GPIOA
#define AIN1_Pin GPIO_PIN_3
#define AIN1_GPIO_Port GPIOA
#define SYS_LED_Pin GPIO_PIN_0
#define SYS_LED_GPIO_Port GPIOB
#define ERR_LED_Pin GPIO_PIN_1
#define ERR_LED_GPIO_Port GPIOB
#define IMU_LED_Pin GPIO_PIN_2
#define IMU_LED_GPIO_Port GPIOB
#define DIN3_Pin GPIO_PIN_7
#define DIN3_GPIO_Port GPIOE
#define SC_EN_Pin GPIO_PIN_9
#define SC_EN_GPIO_Port GPIOB
#define DOUT3_Pin GPIO_PIN_0
#define DOUT3_GPIO_Port GPIOE
#define DOUT2_Pin GPIO_PIN_1
#define DOUT2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
