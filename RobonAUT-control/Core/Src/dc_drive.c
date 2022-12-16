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
int32_t v=0;
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
*/
	v =(0x8000- __HAL_TIM_GET_COUNTER(htim_encoder));
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

	if(motorEnBattOk && motorEnRemote && motorEnLineOk) MOTOR_EN(1);//ha nem nyomtunk vészstopot és az akkuk is rendben vannak akkor pöröghet a motor
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


void Motor_seq(TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_encoder,UART_HandleTypeDef *huart , uint32_t tick, uint32_t period){

	static uint32_t motorDutyPrev=0;
	static uint8_t flag=0;
	int32_t ccr1;
	int32_t ccr2;
	static int32_t cntr=0;
	static int32_t prev=100;
	uint8_t str[4];
	uint8_t string[20];
	static uint32_t motor_seq_tick=0;


	if(motor_seq_tick>tick) return;
	motor_seq_tick= tick + period;

	v =(0x8000 - __HAL_TIM_GET_COUNTER(htim_encoder));
	TIM8->CNT=0x8000;
	if(v<0)v=0;

	if(motorEnRemote==2)//vészstop
	{
		MOTOR_EN(1);
		motorDuty=-50;
		flag=0;
	}

	if (flag==0 && motorEnRemote==1)//egységugrás indítása
	{
		MOTOR_EN(1);
		motorDuty=prev;
		flag=1;
		LED_Y(0);
	}

	if(cntr<142 && flag==1)
	{

		str[0]=(uint8_t)((motorDuty& 0x0000ff00)>>8);
		str[1]=(uint8_t)(motorDuty & 0x000000ff);
		str[2]=(uint8_t)((v & 0x0000ff00)>>8);
		str[3]=(uint8_t)(v & 0x000000ff);
		HAL_UART_Transmit(huart, str, 4, 10);
		/*
		sprintf(string, "%d\n\r",v);
		HAL_UART_Transmit(huart, string, strlen(string), 10);
		*/
		cntr++;
	}
	if (cntr>=142 && flag==1){

		str[0]=(uint8_t)((motorDuty& 0x0000ff00)>>8);
		str[1]=(uint8_t)(motorDuty & 0x000000ff);
		str[2]=(uint8_t)((v & 0x0000ff00)>>8);
		str[3]=(uint8_t)(v & 0x000000ff);
		HAL_UART_Transmit(huart, str, 4, 10);

		/*
		sprintf(string, "%d\n\r",v);
		HAL_UART_Transmit(huart, string, strlen(string), 10);
		*/
		prev = motorDuty + 50;
		motorDuty = 100;
		flag=2;
	}
	if(flag==2)
	{
		str[0]=(uint8_t)((motorDuty& 0x0000ff00)>>8);
		str[1]=(uint8_t)(motorDuty & 0x000000ff);
		str[2]=(uint8_t)((v & 0x0000ff00)>>8);
		str[3]=(uint8_t)(v & 0x000000ff);
		HAL_UART_Transmit(huart, str, 4, 10);

		/*
		sprintf(string, "%d\n\r",v);
		HAL_UART_Transmit(huart, string, strlen(string), 10);
		*/
	}
	//asd
	if(v==0 && flag == 2)
	{
		flag=0;
		cntr=0;
		MOTOR_EN(0);
		LED_Y(1);
	}

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

