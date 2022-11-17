/*
 * slave_stm_uart.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Zsombesz
 */

#ifndef INC_CONFIGG0_H_
#define INC_CONFIGG0_H_

#include "stm32g0xx_hal.h"
#include "main.h"

#define START_BYTE 23
#define STOP_BYTE 18
#define CMD_READ 255

extern uint8_t rcvByteG0[];

void G0_Basic_Init(TIM_HandleTypeDef *htim_task,UART_HandleTypeDef *huart_stm, UART_HandleTypeDef *huart_debug);
void Slave_UART_ISR(UART_HandleTypeDef *huart, UART_HandleTypeDef *huart_debug);

#endif /* INC_CONFIGG0_H_ */
