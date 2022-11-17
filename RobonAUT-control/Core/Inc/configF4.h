/*
 * config.h
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */

#ifndef INC_CONFIGF4_H_
#define INC_CONFIGF4_H_

#include "stm32f4xx_hal.h"

#define CMD_READ 255
#define START_BYTE 23
#define STOP_BYTE 18

void Meas_Bat_Task(ADC_HandleTypeDef *hadc,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period);
void F4_Basic_Init(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim,TIM_HandleTypeDef *htim3);
void Read_G0_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug, uint32_t tick, uint32_t period);

#endif /* INC_CONFIGF4_H_ */
