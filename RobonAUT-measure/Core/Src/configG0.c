/*
 * slave_stm_uart.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Zsombesz
 */

#include "configG0.h"
#include "line_sensor.h"
#include "tof.h"
#include <string.h>


uint8_t rcvByteG0[1];
uint8_t sendByteG0[8];

void G0_Basic_Init(TIM_HandleTypeDef *htim_task,UART_HandleTypeDef *huart_stm, UART_HandleTypeDef *huart_debug)
{
	//bluetoothon kiküldjük az üdvözlő sort
	uint8_t buf[30];
	memset(buf,0,30);
	sprintf(buf,"RobonAUT 2022 Bit Bangers G0\n\r");
	HAL_UART_Transmit(huart_debug, buf, strlen(buf), 20);
	//elindítjuk a task időzítő 32 bites timert
	HAL_TIM_Base_Start(htim_task);
	//várjuk hogy F4 olvasásni akarjon
	rcvByteG0[0]=0;
	sendByteG0[0]=START_BYTE;
	sendByteG0[7]=STOP_BYTE;
	sendByteG0[1]=sendByteG0[2]=sendByteG0[3]=sendByteG0[4]=sendByteG0[5]=sendByteG0[6]=0;
	HAL_UART_Receive_IT(huart_stm,rcvByteG0,1);
}


void Slave_UART_ISR(UART_HandleTypeDef *huart, UART_HandleTypeDef *huart_debug)
{
	if(rcvByteG0[0]==CMD_READ)
	{
		sendByteG0[1]=lsData[0];
		sendByteG0[2]=lsData[1];
		sendByteG0[3]=lsData[2];
		sendByteG0[4]=tofData[0];
		sendByteG0[5]=tofData[1];
		sendByteG0[6]=tofData[2];
		HAL_UART_Transmit(huart, sendByteG0, 8, 2);
	}
	HAL_UART_Receive_IT(huart,rcvByteG0,1); //várjuk hogy F4 olvasásni akarjon
}

