/*
 * line_track.c
 *
 *  Created on: 2022. nov. 29.
 *      Author: Zsombesz
 */

#include "line_track.h"
#include "configF4.h"
#include "dc_driver.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint8_t txBuf[]={CMD_READ};
uint8_t rxBuf[]={0,0,0,0,0,0,0,0};


uint8_t G0_Read(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug)
{

	uint8_t state=0;
	HAL_UART_Transmit(huart_stm, txBuf,1, 1);
	state=HAL_UART_Receive(huart_stm, rxBuf, 8, 2);
	if((state==0)&&(rxBuf[0]==START_BYTE) && (rxBuf[7]==STOP_BYTE))//jöt adat a G0 tól és a keret is megfelelő
	{
		motorEnLineOk = 1;
		return 0;
	}
	else //nem jött szabályos adat a G0-tól
	{
		//motorEnLineOk=0;
		return 1;
	}
}


void Line_Track_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug, uint32_t tick, uint32_t period)
{

	static uint32_t cnt=0;
	uint32_t dist=0;
	static uint32_t line_track_task_tick=0;
	static uint32_t dt[]={1000,1000,1000,1000,1000};
	static uint32_t tick_prev=0;
	static uint8_t lineCnt_prev=1;
	static uint32_t start3time=0;
	static uint8_t index=0;
	static uint8_t str[20];
	static uint32_t breakCnt=0;
	static uint8_t startBreak=0;

	static float x_elso=0;
	static float x_elso_prev=0;
	static float I=0;
	static float x_hatso;
	static float PHI;
	static float delta;
	static float gamma;


	static float m = M_200;
	static float k_p = K_P_200;
	static float k_delta = K_DELTA_200;

	static uint8_t speed = GO_FAST;
	static int32_t ccr = SERVO_CCR_MIDDLE;

	if(line_track_task_tick>tick) return;
	line_track_task_tick = tick + period;

	if(G0_Read(huart_stm, huart_debug)) return; //ha sikertelen az olvasás a G0 ból akkor nincs értelme az egésznek

	if (LINE_CNT<1) //ha nincs vonal a kocsi alatt
	{
		//motorEnLineOk=0; //áljunk meg
		return;
	}
	motorEnLineOk=1; //ha van akkor mehet a szabályozás

	/*****Gyorsasági pálya üzemmód******/
	if(swState[0]==FAST_MODE)
	{
		/*****Gyorsító jelölés figyelése (szaggatott 3 vonal)*****/
		if(LINE_CNT != lineCnt_prev && startBreak!=1) //ha változik az alattunk lévő vonalak száma
		{
			dt[index] = tick - tick_prev;
			uint32_t sum=(dt[0] + dt[1] + dt[2] + dt[3]+ dt[4]);
			if((sum > 300) && (sum < 700))
			{
				v_ref=4000;
				LED_B(1);
				startBreak=0;
			}

			index++;
			if(index>4) index=0;
			tick_prev = tick;
		}
		/* A memóriajellegű statikus változók segítségével vizsgáljuk a szaggatott vonalat*/
		lineCnt_prev = LINE_CNT; //az előző értéket a jelenlegihez hangoljuk

		/*****Lassító jelölés figyelése (folytonos 3 vonal)*****/
		if(LINE_CNT > 1 && startBreak==0) //ha 3 vonalat érzékelünk
		{
			if(tick > (start3time + BREAK_TIME_MS)) //ha már legalább BREAK_TIME_MS -idő óta folyamatosan 3 vonal van alattunk
			{
				v_ref = 1000;
				startBreak=1;
				LED_B(0);
			}
		}
		else //ha 1 vonalat érzékelünk
		{
			start3time = tick;
		}
		/*****FÉKEZÉS NEGATÍV PWM-EL*******/
		if(startBreak==1)
		{
			//motorDuty=BREAK_DUTY-250/BREAK_PERIOD*breakCnt;
			if(breakCnt>BREAK_PERIOD)
			{
				//v_ref = 1000;
				breakCnt=0;
				startBreak=2;
			}
			else breakCnt++;

		}
	}

	/*****SC üzemmód******/
	else if(swState[0]==SC_MODE)
	{
		dist=(((uint16_t)rxBuf[5])<<8) | ((uint16_t)rxBuf[6]);
		v_ref=4*dist-1000;
	}

	x_elso=(float)rxBuf[2]*204/248.0-102;;
	x_hatso=(float)rxBuf[3]*204/244.0-102;
	delta=atan((float)(x_elso-x_hatso)/L_SENSOR);
	/**/
	//szabályozóparaméterek ujraszámolása az aktuális sebesség alapján
	if((v>200 || v<-200))
	{
		k_p = -L/(v*v)*S1MULTS2;
		k_delta = L/v*(S1ADDS2-v*k_p);
		I += K_I*x_elso;
	}

	gamma = -k_p * x_elso -k_delta * delta -K_D*(x_elso-x_elso_prev)- I;
	x_elso_prev=x_elso;
	PHI = atan((L/(L+D))*tan(gamma));//*180.0/3.1415;0

	//1,2424
	if(PHI<0) ccr = (uint16_t)(-SERVO_M * PHI + SERVO_CCR_MIDDLE);//balra kanyarodás
	else ccr = (uint16_t) (-SERVO_M */*1.2*/ PHI + SERVO_CCR_MIDDLE); //jobbra kanyrodás
	/*
	if(cnt>100)
	{
		sprintf(str,"%3.2f    %3.2f, %d\n\r",x_elso, x_hatso, ccr);
		HAL_UART_Transmit(huart_debug, str, strlen(str), 3);
		cnt=0;
	}
	else cnt++;
	 */
	if(ccr > CCR_MAX)//ne feszítsük neki a mechanikai határnak a szervót
	{
		ccr = CCR_MAX;
	}
	else if(ccr < CCR_MIN)//egyik irányba se
	{
		ccr = CCR_MIN;
	}
	TIM2->CCR1 = ccr;
}




