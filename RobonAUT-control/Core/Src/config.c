/*
 * config.c
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */


#include "config.h"
#include "main.h"
#include <string.h>
#include<stdio.h>

// hadc2;huart

void F4_Basic_Init(UART_HandleTypeDef *huart,ADC_HandleTypeDef *hadc,TIM_HandleTypeDef *htim,uint8_t *buf)
{
	LED_R(0);
	LED_B(0);
	LED_G(0);
	LED_Y(0);
	memset(buf,0,32); //a buf tömböt feltöltöm 0-kkal
	sprintf(buf,"RobonAUT 2022 Bit Bangers\r\n");// a buff tömb-be beleírom (stringprint) a string-emet. 1 karakter = 1 byte = 1 tömbelem
	HAL_UART_Transmit(huart, buf, strlen(buf), 100);// A UART2-őn (ide van kötve a programozó) kiküldöm a buf karaktertömböt (string) és maximum 10-ms -ot várok hogy ezt elvégezze a periféria
	HAL_TIM_Base_Start(htim);//heart beat timer tick start



}


void Meas_Bat_Task(ADC_HandleTypeDef *hadc,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{
	uint16_t raw;
	float bat;
	char msg[30];
	static uint32_t meas_bat_tick=0;


	if(meas_bat_tick>tick) return;
	meas_bat_tick= tick + period;

	//get ADC value
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 1);
	raw = HAL_ADC_GetValue(hadc);

	//Raw to Volt
	bat=(float)raw/2091*8.05;

	//Print Value
	sprintf(msg,"NI-MH feszultsege: %3.2f [V] (%d)\r\n",bat,raw);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),10);


	memset(msg,0,30);
	if (raw<1600)
	{
		meas_bat_tick= tick + period/10;
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		sprintf(msg,"Toltes szukseges\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),10);

	}
	else LED_Y(0);


}
