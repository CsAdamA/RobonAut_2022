/*
 * remote_control.c
 *
 *  Created on: 2022. okt. 24.
 *      Author: Zsombesz
 */

#include "remote_control.h"
#include "main.h"
#include <string.h>
#include <math.h>

uint32_t tEdge[3];

void Remote_Control_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
	tEdge[0] = 0;
	tEdge[1] = 0;
	tEdge[2] = 0;
	HAL_TIM_IC_Start_DMA(htim, channel, tEdge, 3);
}
void Remote_Control_Task(TIM_HandleTypeDef *htim, uint32_t channel,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{
	static int32_t dt0=0;
	static int32_t dt1=0;
	static int32_t dt2=0;
	static uint32_t tLow=0;
	static char string[30];//kiiratáshoz
	static uint32_t remote_control_tick=0;

	if(remote_control_tick>tick) return;
	remote_control_tick = tick + period;

	//a három időkülönbésgből egy a T_s, egy a D*T_s és egy az (1-D)*T_s, de nem tudjuk melyik melyik a cirkuláris buffer miatt
	HAL_NVIC_DisableIRQ(TIM4_IRQn);  //atomivá tesszük ezt a két műveletet
	dt0=abs(tEdge[1]-tEdge[0]);
	dt1=abs(tEdge[2]-tEdge[1]);
	dt2=abs(tEdge[0]-tEdge[2]);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);   // mostmár fogadhatjuk az új pwm periodusokat

	//lehet hogy a kövi fűrészjelen vagyunk, ezt ki kell kompenzálni
	if(dt0>5000)dt0=0xffff-dt0;
	if(dt1>5000)dt1=0xffff-dt1;
	if(dt2>5000)dt2=0xffff-dt2;

	//mostmár tényleges időkülönbségeink vannak
	//a három különbéség közül a legkisebb kell nekünk ->
	if(dt0<dt1 && dt0<dt2) //a 4 különbésg közül a legkisebb adja a magasan töltött időt
	{
		tLow=dt0;
	}
	else if(dt1<dt0 && dt1<dt2)
	{
		tLow=dt1;
	}
	else
	{
		tLow=dt2;
	}
/*
	sprintf(string,"%d\n\n\r",tLow);
	HAL_UART_Transmit(huart, string, strlen(string), 100);
*/
	if(tLow<70 || tLow>100) LED_R(1); //ha nincs meghuzva a ravasz tLow kb 87, ha meg van huzva kb 55, ha előre van nyomva kb 118
	else LED_R(0);
}




