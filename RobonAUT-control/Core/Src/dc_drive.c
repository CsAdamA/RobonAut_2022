/*
 * dc_drive.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Levi
 */


#include "dc_driver.h"
#include "main.h"
#include "configF4.h"
#include <string.h>


int32_t motorDuty=200;//(-1000)-től (1000)-ig változhasson elméletben (gykorlatban -950 től 950 ig és a [-50,50] sáv is tiltott)
//ha 1000 akkor a motor full csutkán megy előre
//ha -1000 akkor a motor full csutkán megy hátra

void Motor_Drive_Task(TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_encoder, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period) //DUTY paramtert kiszedtem -> változtassuk a globális változót
{
	static uint32_t motorDutyPrev=0;
	static uint32_t motor_drive_task_tick=0;
	static int32_t tick_prev=0;


	int32_t ccr1;
	int32_t ccr2;
	if(motor_drive_task_tick>tick) return;
	motor_drive_task_tick= tick + period;

	/*MOTOR DUTY BLUETOOTH-RÓL MÓDOSÍTHATÓ
	motorDuty=2*fromPC[0];
	if (motorDuty<50)
	{
		motorDuty=50;
	}
*/
	v =( __HAL_TIM_GET_COUNTER(htim_encoder)-0x8000)/(tick-tick_prev)*7490;
	TIM8->CNT=0x8000;
	tick_prev=tick;


#ifdef MOTOR_DEBUG
	static int32_t i=0;
	uint8_t str[10];
	if(i>100){
		i=0;
		sprintf(str,"%d\n\r",v);
		HAL_UART_Transmit(huart, str, strlen(str), 2);
	}else i++;
#endif

	if(motorEnBattOk && motorEnRemote && motorEnLineOk && motorDuty>50) MOTOR_EN(1);//ha nem nyomtunk vészstopot és az akkuk is rendben vannak akkor pöröghet a motor
	else MOTOR_EN(0); //amugy stop
	//A két érték amit irogatsz (TIM3->CCR1,CCR2) konkrét timer periféria regiszterek, nem feltétlen jó őket folyamatosan újraírni
	if(motorDuty!=motorDutyPrev)//csak akkor írjuk át őket ha tényleg muszáj (ha változtak az előző taskhívás óta)
	{
		ccr2 = (motorDuty + 1000)/2-1;
		if(ccr2>950)ccr2=950;
		if(ccr2<-950)ccr2=-950;
		ccr1= 1000-ccr2-2;

		//2 Referencia megadása
		//Ezeket a loopba kéne változtatni folyamatosan, pwm-elinditas mashova kell majd
		TIM3->CCR1=ccr1;
		TIM3->CCR2=ccr2;
	}
	motorDutyPrev=motorDuty;
}

