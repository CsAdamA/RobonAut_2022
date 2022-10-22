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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define TEL_GPIO7_Pin GPIO_PIN_0
#define TEL_GPIO7_GPIO_Port GPIOC
#define TEL_GPIO6_Pin GPIO_PIN_1
#define TEL_GPIO6_GPIO_Port GPIOC
#define Motor_Curr_Pin GPIO_PIN_2
#define Motor_Curr_GPIO_Port GPIOC
#define Motor_Bat_Pin GPIO_PIN_3
#define Motor_Bat_GPIO_Port GPIOC
#define Servo2_PWM_Pin GPIO_PIN_0
#define Servo2_PWM_GPIO_Port GPIOA
#define TEL_GPIO3_Pin GPIO_PIN_1
#define TEL_GPIO3_GPIO_Port GPIOA
#define STLINK_TX_Pin GPIO_PIN_2
#define STLINK_TX_GPIO_Port GPIOA
#define STLINK_RX_Pin GPIO_PIN_3
#define STLINK_RX_GPIO_Port GPIOA
#define TEL_GPIO4_Pin GPIO_PIN_4
#define TEL_GPIO4_GPIO_Port GPIOA
#define On_Board_LED_Pin GPIO_PIN_5
#define On_Board_LED_GPIO_Port GPIOA
#define MotorPWM1_Pin GPIO_PIN_6
#define MotorPWM1_GPIO_Port GPIOA
#define MotorPWM2_Pin GPIO_PIN_7
#define MotorPWM2_GPIO_Port GPIOA
#define Bat_Meas_Pin GPIO_PIN_0
#define Bat_Meas_GPIO_Port GPIOB
#define LED4_Pin GPIO_PIN_1
#define LED4_GPIO_Port GPIOB
#define SW2_Pin GPIO_PIN_10
#define SW2_GPIO_Port GPIOB
#define LED1_Pin GPIO_PIN_13
#define LED1_GPIO_Port GPIOB
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOB
#define LED3_Pin GPIO_PIN_15
#define LED3_GPIO_Port GPIOB
#define Enkoder_CHA_Pin GPIO_PIN_6
#define Enkoder_CHA_GPIO_Port GPIOC
#define Enkoder_CHB_Pin GPIO_PIN_8
#define Enkoder_CHB_GPIO_Port GPIOC
#define SW1_Pin GPIO_PIN_8
#define SW1_GPIO_Port GPIOA
#define TEL_TX_Pin GPIO_PIN_9
#define TEL_TX_GPIO_Port GPIOA
#define TEL_RX_Pin GPIO_PIN_10
#define TEL_RX_GPIO_Port GPIOA
#define Servo1_PWM_Pin GPIO_PIN_15
#define Servo1_PWM_GPIO_Port GPIOA
#define TB_TX_Pin GPIO_PIN_10
#define TB_TX_GPIO_Port GPIOC
#define TB_RX_Pin GPIO_PIN_11
#define TB_RX_GPIO_Port GPIOC
#define NUCLEO_TX_Pin GPIO_PIN_12
#define NUCLEO_TX_GPIO_Port GPIOC
#define NUCLEO_RX_Pin GPIO_PIN_2
#define NUCLEO_RX_GPIO_Port GPIOD
#define B1B4_Pin GPIO_PIN_4
#define B1B4_GPIO_Port GPIOB
#define B2_Pin GPIO_PIN_5
#define B2_GPIO_Port GPIOB
#define Motor_EN_Pin GPIO_PIN_6
#define Motor_EN_GPIO_Port GPIOB
#define Motor_Feedback_Pin GPIO_PIN_7
#define Motor_Feedback_GPIO_Port GPIOB
#define RC_PWM1_Pin GPIO_PIN_8
#define RC_PWM1_GPIO_Port GPIOB
#define RC_PWM2_Pin GPIO_PIN_9
#define RC_PWM2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
