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
#include "stm32g0xx_hal.h"

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
#define XSHUT_1_Pin GPIO_PIN_12
#define XSHUT_1_GPIO_Port GPIOC
#define XSHUT_2_Pin GPIO_PIN_14
#define XSHUT_2_GPIO_Port GPIOC
#define DIST_SCL_12_Pin GPIO_PIN_0
#define DIST_SCL_12_GPIO_Port GPIOC
#define DIST_SDA_12_Pin GPIO_PIN_1
#define DIST_SDA_12_GPIO_Port GPIOC
#define LS_INF_LED_MOSI_Pin GPIO_PIN_3
#define LS_INF_LED_MOSI_GPIO_Port GPIOC
#define LS_AD_SCK_Pin GPIO_PIN_1
#define LS_AD_SCK_GPIO_Port GPIOA
#define LS_AD_MOSI_Pin GPIO_PIN_2
#define LS_AD_MOSI_GPIO_Port GPIOA
#define LS_INF_OE_BACK_Pin GPIO_PIN_3
#define LS_INF_OE_BACK_GPIO_Port GPIOA
#define LS_AD_CS4_Pin GPIO_PIN_4
#define LS_AD_CS4_GPIO_Port GPIOA
#define LS_INF_OE_FRONT_Pin GPIO_PIN_5
#define LS_INF_OE_FRONT_GPIO_Port GPIOA
#define LS_AD_MISO_Pin GPIO_PIN_6
#define LS_AD_MISO_GPIO_Port GPIOA
#define LS_AD_CS5_Pin GPIO_PIN_7
#define LS_AD_CS5_GPIO_Port GPIOA
#define LS_LED_LE_FRONT_Pin GPIO_PIN_4
#define LS_LED_LE_FRONT_GPIO_Port GPIOC
#define LS_LED_LE_BACK_Pin GPIO_PIN_5
#define LS_LED_LE_BACK_GPIO_Port GPIOC
#define LS_LED_OE_FRONT_Pin GPIO_PIN_0
#define LS_LED_OE_FRONT_GPIO_Port GPIOB
#define LS_LED_OE_BACK_Pin GPIO_PIN_1
#define LS_LED_OE_BACK_GPIO_Port GPIOB
#define LS_AD_CS6_Pin GPIO_PIN_2
#define LS_AD_CS6_GPIO_Port GPIOB
#define LS_INF_LED_SCK_Pin GPIO_PIN_10
#define LS_INF_LED_SCK_GPIO_Port GPIOB
#define LS_INF_LE_FRONT_Pin GPIO_PIN_11
#define LS_INF_LE_FRONT_GPIO_Port GPIOB
#define LS_INF_LE_BACK_Pin GPIO_PIN_12
#define LS_INF_LE_BACK_GPIO_Port GPIOB
#define LS_AD_CS1_Pin GPIO_PIN_15
#define LS_AD_CS1_GPIO_Port GPIOB
#define LS_AD_CS2_Pin GPIO_PIN_8
#define LS_AD_CS2_GPIO_Port GPIOA
#define LS_AD_CS3_Pin GPIO_PIN_9
#define LS_AD_CS3_GPIO_Port GPIOA
#define TEL_TX_Pin GPIO_PIN_8
#define TEL_TX_GPIO_Port GPIOD
#define TEL_RX_Pin GPIO_PIN_9
#define TEL_RX_GPIO_Port GPIOD
#define LS_AD_CS7_Pin GPIO_PIN_8
#define LS_AD_CS7_GPIO_Port GPIOC
#define LS_AD_CS8_Pin GPIO_PIN_9
#define LS_AD_CS8_GPIO_Port GPIOC
#define STM_RX_Pin GPIO_PIN_2
#define STM_RX_GPIO_Port GPIOD
#define STM_TX_Pin GPIO_PIN_3
#define STM_TX_GPIO_Port GPIOD
#define INERC_CS_Pin GPIO_PIN_4
#define INERC_CS_GPIO_Port GPIOD
#define INERC_SCK_Pin GPIO_PIN_3
#define INERC_SCK_GPIO_Port GPIOB
#define INERC_MISO_Pin GPIO_PIN_4
#define INERC_MISO_GPIO_Port GPIOB
#define INERC_MOSI_Pin GPIO_PIN_5
#define INERC_MOSI_GPIO_Port GPIOB
#define DIST_SCL_34_Pin GPIO_PIN_6
#define DIST_SCL_34_GPIO_Port GPIOB
#define DIST_SDA_34_Pin GPIO_PIN_7
#define DIST_SDA_34_GPIO_Port GPIOB
#define XSHUT_3_Pin GPIO_PIN_8
#define XSHUT_3_GPIO_Port GPIOB
#define XSHUT_4_Pin GPIO_PIN_10
#define XSHUT_4_GPIO_Port GPIOC
/* USER CODE BEGIN Private defines */
#define TICK (__HAL_TIM_GET_COUNTER(&htim2))//Az ütemező timer CNT regiszterének kiolvasása
//#define LS_DEBUG
//#define TOF_DEBUG
#define LS_OE_DUTY_CYCLE 5

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
