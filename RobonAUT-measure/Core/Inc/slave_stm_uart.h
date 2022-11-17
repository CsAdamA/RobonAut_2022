/*
 * slave_stm_uart.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Zsombesz
 */

#ifndef INC_SLAVE_STM_UART_H_
#define INC_SLAVE_STM_UART_H_

#include "stm32g0xx_hal.h"
#include "line_sensor.h"
#include "main.h"

#define START_BYTE 23
#define STOP_BYTE 18
#define CMD_READ 255

extern uint8_t *rcvByteG0;

void Slave_UART_Init(UART_HandleTypeDef *huart);
void Slave_UART_ISR(UART_HandleTypeDef *huart);

#endif /* INC_SLAVE_STM_UART_H_ */
