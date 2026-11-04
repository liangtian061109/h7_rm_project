/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define BMI088_ACCEL_SPI_CS_Pin GPIO_PIN_0
#define BMI088_ACCEL_SPI_CS_GPIO_Port GPIOC
#define BMI088_RX_SPI_MOSI_Pin GPIO_PIN_1
#define BMI088_RX_SPI_MOSI_GPIO_Port GPIOC
#define BMI088_TX_SPI__MISO_Pin GPIO_PIN_2
#define BMI088_TX_SPI__MISO_GPIO_Port GPIOC
#define BMI088_GYRO_SPI_CS_Pin GPIO_PIN_3
#define BMI088_GYRO_SPI_CS_GPIO_Port GPIOC
#define BMI088_ACCEL__INTERRUPT_Pin GPIO_PIN_10
#define BMI088_ACCEL__INTERRUPT_GPIO_Port GPIOE
#define BMI088_ACCEL__INTERRUPT_EXTI_IRQn EXTI15_10_IRQn
#define BMI088_GYRO__INTERRUPT_Pin GPIO_PIN_12
#define BMI088_GYRO__INTERRUPT_GPIO_Port GPIOE
#define BMI088_GYRO__INTERRUPT_EXTI_IRQn EXTI15_10_IRQn

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
