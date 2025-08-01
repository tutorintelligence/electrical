/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CS_Pin GPIO_PIN_3
#define CS_GPIO_Port GPIOE
#define HOLD_Pin GPIO_PIN_4
#define HOLD_GPIO_Port GPIOE
#define ETH_INT_Pin GPIO_PIN_15
#define ETH_INT_GPIO_Port GPIOF
#define Input_Curr_INT_Pin GPIO_PIN_0
#define Input_Curr_INT_GPIO_Port GPIOG
#define Temp_ALERT_Pin GPIO_PIN_9
#define Temp_ALERT_GPIO_Port GPIOE
#define TPH_Pin GPIO_PIN_10
#define TPH_GPIO_Port GPIOE
#define TPL_Pin GPIO_PIN_11
#define TPL_GPIO_Port GPIOE
#define BT_Pin GPIO_PIN_12
#define BT_GPIO_Port GPIOE
#define USB_FAULT_Pin GPIO_PIN_12
#define USB_FAULT_GPIO_Port GPIOB
#define USB_EN_Pin GPIO_PIN_8
#define USB_EN_GPIO_Port GPIOD
#define RS485_TX_Pin GPIO_PIN_10
#define RS485_TX_GPIO_Port GPIOC
#define RS485_RX_Pin GPIO_PIN_11
#define RS485_RX_GPIO_Port GPIOC
#define RS232_TX_1_Pin GPIO_PIN_12
#define RS232_TX_1_GPIO_Port GPIOC
#define RS232_RX_1_Pin GPIO_PIN_2
#define RS232_RX_1_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
