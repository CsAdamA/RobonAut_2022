/*
 * slave_stm_uart.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Zsombesz
 */

#include "configG0.h"
#include <string.h>
#include <stdio.h>


uint8_t rcvByteG0[1];
volatile uint8_t sendByteG0[10];
volatile uint8_t mode;

void G0_Basic_Init(TIM_HandleTypeDef *htim_task,UART_HandleTypeDef *huart_stm, UART_HandleTypeDef *huart_debug)
{
	//bluetoothon kiküldjük az üdvözlő sort
	char buf[40];
	sprintf(buf,"RobonAUT 2023 Bit Bangers G0\n\r");
	HAL_UART_Transmit(huart_debug, (uint8_t*)buf, strlen(buf), 20);
	//elindítjuk a task időzítő 32 bites timert
	HAL_TIM_Base_Start(htim_task);
	//várjuk hogy F4 olvasásni akarjon
	rcvByteG0[0]=0;
	sendByteG0[0]=START_BYTE_SKILL_FORWARD;
	sendByteG0[9]=STOP_BYTE;
	sendByteG0[1]=sendByteG0[2]=sendByteG0[3]=sendByteG0[4]=sendByteG0[5]=sendByteG0[6]=sendByteG0[7]=sendByteG0[8]=0;
	mode = SKILL;
	HAL_UART_Receive_IT(huart_stm,rcvByteG0,1);
	////////////////////////////////////////////////////////////////////////////
}


void Slave_UART_RX_ISR(UART_HandleTypeDef *huart, UART_HandleTypeDef *huart_debug)
{
	if(rcvByteG0[0]==CMD_MODE_FAST)
	{
		mode = FAST;
		sendByteG0[7]=STOP_BYTE;
		HAL_UART_Receive_IT(huart,rcvByteG0,1);

	}
	if(rcvByteG0[0]==CMD_MODE_SKILL)
	{
		mode = SKILL;
		sendByteG0[9]=STOP_BYTE;
		HAL_UART_Receive_IT(huart,rcvByteG0,1);
	}
	else if(rcvByteG0[0]==CMD_READ_FAST)
	{
		if(mode!=FAST)
		{
			mode=FAST;
			sendByteG0[7]=STOP_BYTE;
		}
		sendByteG0[4]=tofData[0];
		sendByteG0[5]=tofData[1];
		sendByteG0[6]=tofData[2];
		HAL_UART_Transmit_IT(huart, (uint8_t*)sendByteG0, 8);
	}
	else if(rcvByteG0[0]==CMD_READ_SKILL_FORWARD || rcvByteG0[0]==CMD_READ_SKILL_REVERSE)
	{
		if(mode!=SKILL)
		{
			mode=SKILL;
			sendByteG0[9]=STOP_BYTE;
		}
		sendByteG0[6]=tofData[0];
		sendByteG0[7]=tofData[1];
		sendByteG0[8]=tofData[2];
		HAL_UART_Transmit_IT(huart, (uint8_t*)sendByteG0, 10);
	}
}

void Slave_UART_TX_ISR(UART_HandleTypeDef *huart, UART_HandleTypeDef *huart_debug)
{
	HAL_UART_Receive_IT(huart,rcvByteG0,1); //várjuk hogy F4 olvasásni akarjon
}

