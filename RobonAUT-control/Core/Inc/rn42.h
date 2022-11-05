/*
 * rn42.h
 *
 *  Created on: Nov 4, 2022
 *      Author: Zsombesz
*/

#ifndef INC_RN42_H_
#define INC_RN42_H_

#include "stm32f4xx_hal.h"
#include "main.h"

#define TEL_3(x) (HAL_GPIO_WritePin(TEL_GPIO3_GPIO_Port,TEL_GPIO3_Pin , x))
#define TEL_4(x) (HAL_GPIO_WritePin(TEL_GPIO4_GPIO_Port,TEL_GPIO4_Pin , x))
#define TEL_6(x) (HAL_GPIO_WritePin(TEL_GPIO6_GPIO_Port,TEL_GPIO6_Pin , x))
#define TEL_7(x) (HAL_GPIO_WritePin(TEL_GPIO7_GPIO_Port,TEL_GPIO7_Pin , x))

#define ZSOMBI_MAC (0xAC1203EA55B4)
#define ADI_MAC (0x70C94ED33AC8)

uint8_t Rn42_Init(UART_HandleTypeDef *uart_rn42, UART_HandleTypeDef *uart_pc);

#endif
