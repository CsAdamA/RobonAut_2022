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


int32_t v_ref=1000; //mm/s
float v=0.0;
//ha 1000 akkor a motor full csutkán megy előre
//ha -1000 akkor a motor full csutkán megy hátra

void Measure_Velocity_Task(TIM_HandleTypeDef *htim_encoder,uint32_t tick, uint32_t period)
{
	static uint32_t tick_prev=0;
	static uint32_t measure_v_task_tick=0;
	static float alpha=0.25;
	float v_uj;

	if(measure_v_task_tick>tick) return;
	measure_v_task_tick= tick + period;
	v_uj =(float)(0x8000 - __HAL_TIM_GET_COUNTER(htim_encoder))*7.49/(tick-tick_prev); //mm/s dimenzió
	TIM8->CNT=0x8000;
	tick_prev=tick;
	//exponenciális szűrés
	v = alpha*v_uj + (1-alpha)*v;
}

void Motor_Drive_Task(TIM_HandleTypeDef *htim_motor, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period) //DUTY paramtert kiszedtem -> változtassuk a globális változót
{
	static int16_t motorDuty=0;
	static uint32_t motorDutyPrev=0;
	static uint32_t motor_drive_task_tick=5;
	static float u2,u2_prev,u1,u,u_prev=0;

	int32_t ccr1;
	int32_t ccr2;
	if(motor_drive_task_tick>tick) return;
	motor_drive_task_tick= tick + period;

	//az u paraméter a bevatkozó jel minusz holtásávot adja meg
	u2 = ZD*u2_prev+(1-ZD)*u_prev;
	u1= KC * (float)(v_ref-v);
	u= u1+u2;
	if(u>880)u=880;
	else if(u<(-880))u=(-880);
	u_prev=u;
	u2_prev=u2;
	//ez alapján a kiadandó kitöltési tényező
	if(u>0) motorDuty=u+70;
	else if(u<0) motorDuty=u-70;
	else motorDuty=u;

	if(motorEnBattOk && motorEnRemote && motorEnLineOk) MOTOR_EN(1);//ha nem nyomtunk vészstopot és az akkuk is rendben vannak akkor pöröghet a motor
	//else motorDuty=-50;
	else MOTOR_EN(0); //amugy stop
	//A két érték amit irogatsz (TIM3->CCR1,CCR2) konkrét timer periféria regiszterek, nem feltétlen jó őket folyamatosan újraírni 10ms enként
	if(motorDuty!=motorDutyPrev)//csak akkor írjuk át őket ha tényleg muszáj (ha változtak az előző taskhívás óta)
	{
		ccr2 = (motorDuty + 1000)/2-1;
		if(ccr2>950)ccr2=950;
		if(ccr2<-950)ccr2=-950;
		ccr1= 998-ccr2;

		//2 Referencia megadása
		//Ezeket a loopba kéne változtatni folyamatosan, pwm-elinditas mashova kell majd
		TIM3->CCR1=ccr1;
		TIM3->CCR2=ccr2;
	}
	motorDutyPrev=motorDuty;
}


void Motor_seq(TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_encoder,UART_HandleTypeDef *huart , uint32_t tick, uint32_t period){

	static int16_t motorDuty=200;
	static int32_t motorDutyPrev=0;
	static uint8_t flag=0;
	int32_t ccr1;
	int32_t ccr2;
	static int32_t cntr=0;
	static int32_t prev=70;
	uint8_t str[4];
	uint8_t string[20];
	static uint32_t motor_seq_tick=0;


	if(motor_seq_tick>tick) return;
	motor_seq_tick= tick + period;

	v =(0x8000 - __HAL_TIM_GET_COUNTER(htim_encoder));
	TIM8->CNT=0x8000;
	if(v<0)v=1;

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
		str[2]=(uint8_t)(((int)v & 0x0000ff00)>>8);
		str[3]=(uint8_t)((int)v & 0x000000ff);
		HAL_UART_Transmit(huart, str, 4, 3);
		/*
		sprintf(string, "%d\n\r",v);
		HAL_UART_Transmit(huart, string, strlen(string), 10);
*/
		cntr++;
	}
	if (cntr>=142 && flag==1){

		str[0]=(uint8_t)((motorDuty& 0x0000ff00)>>8);
		str[1]=(uint8_t)(motorDuty & 0x000000ff);
		str[2]=(uint8_t)(((int)v & 0x0000ff00)>>8);
		str[3]=(uint8_t)((int)v & 0x000000ff);
		HAL_UART_Transmit(huart, str, 4, 3);

/*
		sprintf(string, "%d\n\r",v);
		HAL_UART_Transmit(huart, string, strlen(string), 10);
*/
		prev = 420;
		motorDuty = 70;
		flag=2;
	}
	if(flag==2)
	{
		str[0]=(uint8_t)((motorDuty& 0x0000ff00)>>8);
		str[1]=(uint8_t)(motorDuty & 0x000000ff);
		str[2]=(uint8_t)(((int)v & 0x0000ff00)>>8);
		str[3]=(uint8_t)((int)v & 0x000000ff);
		HAL_UART_Transmit(huart, str, 4, 3);

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

