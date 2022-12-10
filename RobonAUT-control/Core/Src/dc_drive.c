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

void Motor_Drive_Task(TIM_HandleTypeDef *htim, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period) //DUTY paramtert kiszedtem -> változtassuk a globális változót
{
	static uint32_t motorDutyPrev=0;
	static uint32_t motor_drive_task_tick=0;
	int32_t ccr1;
	int32_t ccr2;
	if(motor_drive_task_tick>tick) return;
	motor_drive_task_tick= tick + period;

#ifdef MOTOR_DEBUG
	uint8_t buf[20];
	memset(buf,0,20);
	sprintf(buf,"Kitoltesi tenyezo: %d \r\n",DUTY);
	HAL_UART_Transmit(huart, buf, strlen(buf), 10);
	motor_drive_task_tick= tick + 2000;
#endif

	if(motorEnBattOk && motorEnRemote && motorEnLineOk) MOTOR_EN(1);//ha nem nyomtunk vészstopot és az akkuk is rendben vannak akkor pöröghet a motor
	else MOTOR_EN(0); //amugy stop
	//A két érték amit irogatsz (TIM3->CCR1,CCR2) konkrét timer periféria regiszterek, nem feltétlen jó őket folyamatosan újraírni
	if(motorDuty!=motorDutyPrev)//csak akkor írjuk át őket ha tényleg muszáj (ha változtak az előző taskhívás óta)
	{
		ccr2 = (motorDuty + 1000)/2-1;
		ccr1= 1000-ccr2-2;
		//2 Referencia megadása
		//Ezeket a loopba kéne változtatni folyamatosan, pwm-elinditas mashova kell majd
		TIM3->CCR1=ccr1;
		TIM3->CCR2=ccr2;
	}
	motorDutyPrev=motorDuty;


}
