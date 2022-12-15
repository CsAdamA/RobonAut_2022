/*
 * dc_driver.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Levi
 */

#ifndef INC_DC_DRIVER_H_
#define INC_DC_DRIVER_H_

#include "stm32f4xx_hal.h"

extern int32_t motorDuty;


void Motor_Drive_Task(TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_encoder, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period) ;//DUTY paramtert kiszedtem -> változtassuk a globális változót
#endif /* INC_DC_DRIVER_H_ */
