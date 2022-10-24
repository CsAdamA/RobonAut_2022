/*
 * remote_control.c
 *
 *  Created on: 2022. okt. 24.
 *      Author: Zsombesz
 */

#include "remote_control.h"
#include "main.h"
#include <string.h>

float freqRC=0; //RC pwm frekvenciája
volatile int32_t tHighCnt;
volatile int32_t tLowCnt;
volatile uint8_t flagRC;



void Reset_Remote_Control_Flag(void)
{
	HAL_NVIC_DisableIRQ(TIM4_IRQn); //atomivá tesszük ezt a flagbillentést
	flagRC=0;
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
}

uint8_t Read_Control_Flag(void)
{
	uint8_t tmp=0;
	HAL_NVIC_DisableIRQ(TIM4_IRQn); //atomivá tesszük ezt a flagbillentést
	tmp=flagRC;
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	return tmp;
}

void Remote_Control_Init(TIM_HandleTypeDef *htim, uint32_t channel)
{
	  tHighCnt = 0;
	  tLowCnt = 0;
	  Reset_Remote_Control_Flag();
	  HAL_TIM_IC_Start_IT(htim, channel); //a 4-es timert elindítom interrupt capture modban a 3 as channeljén
}
void Remote_Control_Task(TIM_HandleTypeDef *htim, uint32_t channel,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{
	static float tHighMs=0; // a PWM periódus aktív része milisecundumban
	static float tLowMs=0; // a PWM periódus inaktív része milisecundumban
	static float duty_cycle=0; //a PWM kitölrési tényezője
	static float freq=0; //periódusideje
	static char string[30];//kiiratáshoz
	static uint32_t remote_control_tick=0;

	if(remote_control_tick>tick) return;
	remote_control_tick = tick + period;

	if(Read_Control_Flag()) //ha az előző futás óta érkeztek új élek
	{

	  HAL_NVIC_DisableIRQ(TIM4_IRQn);  //atomivá tesszük ezt a két műveletet

	  tHighMs = 1000.0*tHighCnt*(htim->Init.Prescaler+1)/45000000.0; //átváltjuk az értékeinket ms-ba
	  tLowMs = 1000.0*tLowCnt*(htim->Init.Prescaler+1)/45000000.0;

	  HAL_NVIC_EnableIRQ(TIM4_IRQn);   // mostmár fogadhatjuk az új pwm periodusokat

	  duty_cycle = tHighMs/(tHighMs+tLowMs); //a kitöltési tényező a magas és alacsony idők arányából következik
	  freq = 1000.0/(tHighMs+tLowMs); //a PWM preiódusideje is ezekből számítható
	  Reset_Remote_Control_Flag();//várjuk a következő PWM preiódus adatait
	}
	else //ha az előző futás óta nem érkeztek új élek, akkor mindegyik érték 0
	{
	  tHighMs=0;
	  tLowMs=0;
	  duty_cycle=0;
	}
	sprintf(string,"High time: %.3f ms\r\n",tHighMs);
	HAL_UART_Transmit(huart, string, strlen(string), 5);
	sprintf(string,"Low time: %.3f ms\r\n",tLowMs);
	HAL_UART_Transmit(huart, string, strlen(string), 5);
	sprintf(string,"Duty cycle: %.3f\r\n",duty_cycle);
	HAL_UART_Transmit(huart, string, strlen(string), 5);
	sprintf(string,"Frequency: %.3f Hz\n\n\r",freq);
	HAL_UART_Transmit(huart, string, strlen(string), 5);
}


void Remote_Control_ISR(TIM_HandleTypeDef *htim, uint32_t channel)
{
	static uint32_t t_stamp=0; //élváltás beköevtkezésének időpontja
	static uint32_t t_stamp_prev = 0; //előző élváltás bekövetkezésének időpontja
	t_stamp = __HAL_TIM_GET_COMPARE(htim,channel);

	if(READ_RC1)//ha a láb magas az élváltás után, akkor rising edge volt
	{
		tHighCnt = t_stamp-t_stamp_prev; //a PWM magasan van ennyi ideig (ezt még majd át kell váltani valós ms-be->mainben)
		if(tHighCnt<0) tHighCnt += 0xffff; //másik fűrészfogon van a két időpont -> egy egyszerű 16 bites eltolással kompenzálható
	}
	else //ha a láb alacsony az élváltás után, akkor falling edge volt
	{
		tLowCnt=t_stamp-t_stamp_prev; //a PWM alacsonyan van ennyi ideig (ezt még majd át kell váltani valós ms-be->mainben)
		if(tLowCnt<0) tHighCnt += 0xffff; //másik fűrészfogon van a két időpont -> egy egyszerű 16 bites eltolással kompenzálható
	}
	t_stamp_prev = t_stamp;
	flagRC=1; //jelzünk a main-ben lévő tasknak (kiiratás), hogy fel lehet dolgozni az adatokat, mert érkezett új él
}


