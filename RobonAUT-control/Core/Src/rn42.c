/*
 * rn42.c
 *
 *  Created on: Nov 4, 2022
 *      Author: Zsombesz
*/

#include "rn42.h"
#include <string.h>
#include <math.h>


uint8_t Rn42_Init(UART_HandleTypeDef *uart_rn42, UART_HandleTypeDef *uart_pc)
{
	uint8_t tx_buf[30];
	uint8_t rx_buf[30];
	memset(tx_buf,0,30);
	memset(rx_buf,0,30);

	TEL_7(0);
	TEL_4(0);
	HAL_Delay(501);
	//command mód
	sprintf(tx_buf,"$$$");
	HAL_UART_Transmit(uart_rn42, tx_buf, strlen(tx_buf), 100); //kikuljuk a comandot
	HAL_UART_Receive(uart_rn42,rx_buf, 5, 200); //fogadjuk a választ (CMD\n\R)
	HAL_UART_Transmit(uart_pc, rx_buf, strlen(rx_buf), 100); //ezt kiiratjuk magunknak
	HAL_Delay(100);
	memset(tx_buf,0,30);
	memset(rx_buf,0,30);

	sprintf(tx_buf,"SF,1\r\n");
	HAL_UART_Transmit(uart_rn42, tx_buf, strlen(tx_buf), 100); //kikuljuk a comandot
	HAL_UART_Receive(uart_rn42,rx_buf, 5, 1000); //fogadjuk a választ (CMD\n\R)
	HAL_UART_Transmit(uart_pc, rx_buf, strlen(rx_buf), 100); //ezt kiiratjuk magunknak
	HAL_Delay(2000);
	memset(tx_buf,0,30);
	memset(rx_buf,0,30);

	sprintf(tx_buf,"R,1\r\n");
	HAL_UART_Transmit(uart_rn42, tx_buf, strlen(tx_buf), 100); //kikuljuk a comandot
	HAL_UART_Receive(uart_rn42,rx_buf, 9, 2000); //fogadjuk a választ (CMD\n\R)
	HAL_UART_Transmit(uart_pc, rx_buf, strlen(rx_buf), 100); //ezt kiiratjuk magunknak
	HAL_Delay(2000);
	memset(tx_buf,0,30);
	memset(rx_buf,0,30);

	sprintf(tx_buf,"$$$");
	HAL_UART_Transmit(uart_rn42, tx_buf, strlen(tx_buf), 100); //kikuljuk a comandot
	HAL_UART_Receive(uart_rn42,rx_buf, 5, 200); //fogadjuk a választ (CMD\n\R)
	HAL_UART_Transmit(uart_pc, rx_buf, strlen(rx_buf), 100); //ezt kiiratjuk magunknak
	HAL_Delay(100);
	memset(tx_buf,0,30);
	memset(rx_buf,0,30);

	sprintf(tx_buf,"SM,0\r\n");
	HAL_UART_Transmit(uart_rn42, tx_buf, strlen(tx_buf), 100); //kikuljuk a comandot
	HAL_UART_Receive(uart_rn42,rx_buf, 5, 200); //fogadjuk a választ (CMD\n\R)
	HAL_UART_Transmit(uart_pc, rx_buf, strlen(rx_buf), 100); //ezt kiiratjuk magunknak
	HAL_Delay(100);
	memset(tx_buf,0,30);
	memset(rx_buf,0,30);
/**/
	//connect a megfelelő fizikai cimre
	sprintf(tx_buf,"C,70C94ED33AC8\r");
	HAL_UART_Transmit(uart_rn42, tx_buf, strlen(tx_buf), 300);
	HAL_UART_Receive(uart_rn42,rx_buf, 8, 200);
	HAL_UART_Transmit(uart_pc, rx_buf, strlen(rx_buf), 100);
	HAL_Delay(500);
	memset(tx_buf,0,30);
	memset(rx_buf,0,30);

	//connection status olvasás
	sprintf(tx_buf,"GK\r");
	HAL_UART_Transmit(uart_rn42, tx_buf, strlen(tx_buf), 100);
	HAL_UART_Receive(uart_rn42,rx_buf, 8, 200);
	HAL_UART_Transmit(uart_pc, rx_buf, strlen(rx_buf), 100);
	HAL_Delay(100);
	memset(tx_buf,0,30);
	memset(rx_buf,0,30);

	//megint data mód
	sprintf(tx_buf,"---\r");
	HAL_UART_Transmit(uart_rn42, tx_buf, strlen(tx_buf), 100); //kikuljuk a comandot
	HAL_UART_Receive(uart_rn42,rx_buf, 5, 200); //fogadjuk a választ (CMD\n\R)
	HAL_UART_Transmit(uart_pc, rx_buf, strlen(rx_buf), 100); //ezt kiiratjuk magunknak

	return 1;
}

