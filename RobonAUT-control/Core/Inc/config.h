/*
 * config.h
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */

#ifndef INC_CONFIG_H_
#define INC_CONFIG_H_

#include "stm32f4xx_hal.h"

void Meas_Bat_Task(ADC_HandleTypeDef *hadc,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period);
void F4_Basic_Init(UART_HandleTypeDef *huart,ADC_HandleTypeDef *hadc,TIM_HandleTypeDef *htim, uint8_t *buf);

#endif /* INC_CONFIG_H_ */
