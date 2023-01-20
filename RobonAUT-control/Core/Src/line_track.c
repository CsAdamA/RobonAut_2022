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
#include "control.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint8_t txBuf[]={CMD_READ_SKILL_FORWARD};
uint8_t rxBuf[10];


uint8_t G0_Read_Fast(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg)
{

	uint8_t state=0;
	txBuf[0]=CMD_READ_FAST;
	HAL_UART_Transmit(huart_stm, txBuf,1, 1);
	state=HAL_UART_Receive(huart_stm, rxBuf, 8, 2);
	motorEnLineOk=1; //ha van akkor mehet a szabályozás
	if((state==0)&&(rxBuf[0]==START_BYTE) && (rxBuf[7]==STOP_BYTE))//jöt adat a G0 tól és a keret is megfelelő
	{
		return 0;
	}
	else //nem jött szabályos adat a G0-tól
	{
		return 1;
	}
}

uint8_t G0_Read_Skill(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg, uint8_t command)
{
	uint8_t state=1;
	txBuf[0]=command;
	HAL_UART_Transmit(huart_stm, txBuf,1, 1);
	state=HAL_UART_Receive(huart_stm, rxBuf, 10, 2);
	motorEnLineOk=1; //ha van akkor mehet a szabályozás
	if((state==0)&&(rxBuf[0]==START_BYTE) && (rxBuf[9]==STOP_BYTE))//jöt adat a G0 tól és a keret is megfelelő
	{
		return 0;
	}
	else //nem jött szabályos adat a G0-tól
	{
		return 1;
	}
}

void Line_Track_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg, uint32_t tick, uint32_t period)
{
	static uint32_t line_track_task_tick=0;
	static int32_t ccr = SERVO_FRONT_CCR_MIDDLE;
	static float PHI;
	static float gamma=0;
	static uint8_t reverse=1;

	if(line_track_task_tick>tick) return;
	line_track_task_tick = tick + period;

	if(mode == SKILL)
	{
		if(swState[1])
		{
			if(G0_Read_Skill(huart_stm, huart_debugg,CMD_READ_SKILL_FORWARD)) return;
			v_ref=1400;
			Detect_Node2(huart_debugg, tick);
			if (LINE_CNT<1 || LINE_CNT > 4) return;//ha nincs vonal a kocsi alatt
			gamma = Skill_Mode(huart_debugg);
			PHI = atan((L/(L+D_FRONT))*tan(gamma));
			ccr = (uint16_t)(-SERVO_M * PHI + SERVO_FRONT_CCR_MIDDLE);//balra kanyarodás

			//if(PHI<0) ccr = (uint16_t)(-SERVO_M * PHI + SERVO_CCR_MIDDLE);//balra kanyarodás
			//else ccr = (uint16_t) (-SERVO_M  * PHI + SERVO_CCR_MIDDLE); //jobbra kanyrodás

			if(ccr > CCR_FRONT_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_FRONT_MAX;
			}
			else if(ccr < CCR_FRONT_MIN)//egyik irányba se
			{
				ccr = CCR_FRONT_MIN;
			}
			TIM2->CCR1 = ccr;
			TIM1->CCR4 = SERVO_REAR_CCR_MIDDLE;
		}
		else
		{
			if(G0_Read_Skill(huart_stm, huart_debugg,CMD_READ_SKILL_REVERSE)) return;
			v_ref=-1000;
			Detect_Node2(huart_debugg, tick);
			if (LINE_CNT<1 || LINE_CNT > 4) return;//ha nincs vonal a kocsi alatt
			gamma = Skill_Mode(huart_debugg);
			PHI = -atan((L/(L+D_REAR))*tan(gamma));////////////////////kiszámolni kézzel
			ccr = (uint16_t)(-700 * PHI + SERVO_REAR_CCR_MIDDLE);
			//ccr = (uint16_t)(-SERVO_M * PHI + SERVO_REAR_CCR_MIDDLE);
			//HÁTSÓ SZERVÓ
			if(ccr > CCR_REAR_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_REAR_MAX;
			}
			else if(ccr < CCR_REAR_MIN)//egyik irányba se
			{
				ccr = CCR_REAR_MIN;
			}
			TIM1->CCR4= ccr;
			/**/
			//ELSŐ SEZRVÓ
			PHI = -PHI/10;
			ccr = (uint16_t)(-SERVO_M * PHI + SERVO_FRONT_CCR_MIDDLE);
			if(ccr > CCR_FRONT_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_FRONT_MAX;
			}
			else if(ccr < CCR_FRONT_MIN)//egyik irányba se
			{
				ccr = CCR_FRONT_MIN;
			}

			TIM2->CCR1 =ccr;

			//TIM2->CCR1 = SERVO_FRONT_CCR_MIDDLE;
		}

	}
	/*****Gyorsasági pálya üzemmód******/
	else if(mode == FAST)
	{
		if(G0_Read_Fast(huart_stm, huart_debugg)) return; //ha sikertelen az olvasás a G0 ból akkor nincs értelme az egésznek
		if (LINE_CNT<1 || LINE_CNT > 4) return;//ha nincs vonal a kocsi alatt
		gamma = Fast_Mode(huart_debugg,tick);
		PHI = atan((L/(L+D_FRONT))*tan(gamma));
		ccr = (uint16_t)(-SERVO_M * PHI + SERVO_FRONT_CCR_MIDDLE);

		//if(PHI<0) ccr = (uint16_t)(-SERVO_M * PHI + SERVO_CCR_MIDDLE);//balra kanyarodás
		//else ccr = (uint16_t) (-SERVO_M  * PHI + SERVO_CCR_MIDDLE); //jobbra kanyrodás

		if(ccr > CCR_FRONT_MAX)//ne feszítsük neki a mechanikai határnak a szervót
		{
			ccr = CCR_FRONT_MAX;
		}
		else if(ccr < CCR_FRONT_MIN)//egyik irányba se
		{
			ccr = CCR_FRONT_MIN;
		}
		TIM2->CCR1 = ccr;
		TIM1->CCR4 = SERVO_REAR_CCR_MIDDLE;
	}

}

float Fast_Mode(UART_HandleTypeDef *huart_debugg, uint32_t t)
{
	static uint32_t dt[]={1000,1000,1000,1000,1000};
	static uint32_t t_prev=0;
	static uint8_t lineCnt_prev=1;
	static uint32_t start3time=0;
	static uint8_t index=0;
	static uint8_t Free_Run_State = 0;

	static float k_p = K_P_200;
	static float k_delta = K_DELTA_200;
	static float x_elso=0;
	static float x_elso_prev=0;
	static float x_hatso;
	static float delta;
	static float gamma;

	uint32_t sum=0;
	uint32_t dist=0;


	if(swState[0] == FREERUN_MODE)
	{
		/*****Gyorsító jelölés figyelése (szaggatott 3 vonal)*****/
		if(LINE_CNT != lineCnt_prev && (!Free_Run_State || Free_Run_State==2)) //ha változik az alattunk lévő vonalak száma
		{
			dt[index] = t - t_prev;
			sum=dt[0] + dt[1] + dt[2] + dt[3]+ dt[4];
			if((sum > 300) && (sum < 700))
			{
				v_ref=4200;
				LED_B(1);
				Free_Run_State=1;
			}
			index++;
			if(index>4) index=0;
			t_prev = t;
		}
		/* A memóriajellegű statikus változók segítségével vizsgáljuk a szaggatott vonalat*/
		lineCnt_prev = LINE_CNT; //az előző értéket a jelenlegihez hangoljuk

		/*****Lassító jelölés figyelése (folytonos 3 vonal)*****/
		if(LINE_CNT > 1 && (!Free_Run_State || Free_Run_State==1)) //ha 3 vonalat érzékelünk
		{
			if(t > (start3time + BREAK_TIME_MS)) //ha már legalább BREAK_TIME_MS -idő óta folyamatosan 3 vonal van alattunk
			{
				v_ref = 1100;
				Free_Run_State=2;
				LED_B(0);
			}
		}
		else //ha 1 vonalat érzékelünk
		{
			start3time = t;
		}
		/*****FÉKEZÉS NEGATÍV PWM-EL*******/
	}

	/*****SC üzemmód******/
	else if(swState[0]==SC_MODE)
	{
		dist=(((uint16_t)rxBuf[5])<<8) | ((uint16_t)rxBuf[6]);
		if(dist>1000 || rxBuf[4]) v_ref=1500; //ha tul messze vana  SC vagy érvénytelen az olvasás
		else v_ref = 2*(float)dist-500;
	}

	x_elso=(float)rxBuf[2]*204/248.0-102;;
	x_hatso=(float)rxBuf[3]*204/244.0-102;
	delta=atan((float)(x_elso-x_hatso)/L_SENSOR);
	/**/
	//szabályozóparaméterek ujraszámolása az aktuális sebesség alapján
	if(v>100)
	{
		if(v<2000)
		{
			k_p = -L/(v*v)*S1MULTS2_SLOW;
			k_delta = L/v*(S1ADDS2_SLOW-v*k_p);
		}
		else
		{
			k_p = -L/(v*v)*S1MULTS2_FAST;
			k_delta = L/v*(S1ADDS2_FAST-v*k_p);
		}
	}

	gamma = -k_p * x_elso -k_delta * delta - K_D * (x_elso-x_elso_prev);
	x_elso_prev = x_elso;

	return gamma;
}

