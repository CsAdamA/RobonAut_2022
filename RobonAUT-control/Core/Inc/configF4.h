/*
 * config.h
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */

#ifndef INC_CONFIGF4_H_
#define INC_CONFIGF4_H_

#include "stm32f4xx_hal.h"

void Meas_Bat_Task(ADC_HandleTypeDef *hadc,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period);
void F4_Basic_Init(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim,TIM_HandleTypeDef *htim3,TIM_HandleTypeDef *htim2);
void SW_Read_Task(uint32_t tick, uint32_t period);
void Uart_Receive_From_PC_ISR(UART_HandleTypeDef *huart);

#endif /* INC_CONFIGF4_H_ */
