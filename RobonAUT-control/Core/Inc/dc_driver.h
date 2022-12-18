/*
 * dc_driver.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Levi
 */

#ifndef INC_DC_DRIVER_H_
#define INC_DC_DRIVER_H_

#include "stm32f4xx_hal.h"


//szabáylozóparaméterek 10ms es mintavételi időhöz és Tcl=0.4s es beálláshoz
#define KC (0.2526)
#define ZD (0.9882)

void Measure_Velocity_Task(TIM_HandleTypeDef *htim_encoder,uint32_t tick, uint32_t period);
void Motor_Drive_Task(TIM_HandleTypeDef *htim_motor, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period); //DUTY paramtert kiszedtem -> változtassuk a globális változót
void Motor_seq(TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_encoder,UART_HandleTypeDef *huart , uint32_t tick, uint32_t period);




#endif /* INC_DC_DRIVER_H_ */
