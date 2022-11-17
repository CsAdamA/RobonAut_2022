/*
 * slave_stm_uart.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Zsombesz
 */

#include "slave_stm_uart.h""
#include <string.h>

uint8_t *rcvByteG0;

void Slave_UART_Init(UART_HandleTypeDef *huart)
{
	lsData[0]=START_BYTE;
	lsData[4]=STOP_BYTE;
	HAL_UART_Receive_IT(huart,rcvByte,1); //várjuk hogy F4 olvasásni akarjon
}

void Slave_UART_ISR(UART_HandleTypeDef *huart)
{
	if(rcvByteG0[0]==CMD_READ)
	{
		HAL_UART_Transmit(huart, lsData, 5, 1);
	}
	HAL_UART_Receive_IT(huart,rcvByte,1); //várjuk hogy F4 olvasásni akarjon
}

