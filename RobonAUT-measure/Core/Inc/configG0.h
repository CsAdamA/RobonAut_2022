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

#define CMD_READ_FAST (42)
#define CMD_READ_SKILL_FORWARD (57)
#define CMD_READ_SKILL_REVERSE (145)
#define CMD_MODE_FAST (63)
#define CMD_MODE_SKILL (82)
#define START_BYTE_FAST (23+CMD_READ_FAST) //65
#define START_BYTE_SKILL_FORWARD (23+CMD_READ_SKILL_FORWARD) //80
#define START_BYTE_SKILL_REVERSE (23+CMD_READ_SKILL_REVERSE) //168
#define STOP_BYTE (18)

#define FAST (63)
#define SKILL (82)

void G0_Basic_Init(TIM_HandleTypeDef *htim_task,UART_HandleTypeDef *huart_stm, UART_HandleTypeDef *huart_debug);
void Slave_UART_RX_ISR(UART_HandleTypeDef *huart, UART_HandleTypeDef *huart_debug);
void Slave_UART_TX_ISR(UART_HandleTypeDef *huart, UART_HandleTypeDef *huart_debug);

#endif /* INC_CONFIGG0_H_ */
