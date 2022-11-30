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
		//LED_B(0);
		motorEnLineOk = 1;
		return 0;
	}
	else //nem jött szabályos adat a G0-tól
	{
		motorEnLineOk=0;
		//LED_B(1);
		return 1;
	}
}


void Line_Track_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug, uint32_t tick, uint32_t period)
{

	uint32_t dist=0;
	static uint32_t read_g0_task_tick=0;
	static uint32_t dt[]={1000,1000,1000,1000,1000,1000,1000,1000};
	static uint32_t tick_prev=0;
	static uint8_t lineCnt_prev=1;
	static uint32_t start3time=0;
	static uint8_t index=0;

	static float x_elso;
	static float x_hatso;
	static float PHI;
	static float delta;
	static float gamma;

	static float k_p = K_P_400;
	static float k_delta = K_DELTA_400;
	static uint8_t speed = GO_FAST;
	static int32_t ccr = SERVO_CCR_MIDDLE;

	if(read_g0_task_tick>tick) return;
	read_g0_task_tick = tick + period;

	if(G0_Read(huart_stm, huart_debug)) return; //ha sikertelen az olvasás a G0 ból akkor nincs értelme az egésznek

	if (LINE_CNT<1) //ha nincs vonal a kocsi alatt
	{
		motorEnLineOk=0; //áljunk meg
		return;
	}
	motorEnLineOk=1; //ha van akkor mehet a szabályozás

	/*****Gyorsasági pálya üzemmód******/
	if(swState[0]==FAST_MODE)
	{
		/*****Gyorsító jelölés figyelése (szaggatott 3 vonal)*****/
		if(LINE_CNT != lineCnt_prev) //ha változik az alattunk lévő vonalak száma
		{
			dt[index] = tick - tick_prev;
			uint32_t sum=(dt[0] + dt[1] + dt[2] + dt[3] + dt[4] +dt[5] + dt[6] + dt[7]);
			if((sum > 300) && (sum < 3000))
			{
				motorDuty=400;
				k_p = K_P_400;
				k_delta = K_DELTA_400;
				//LED_B(1);
				HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			}

			index++;
			if(index>7) index=0;
		}
		/* A memóriajellegű statikus változók segítségével vizsgáljuk a szaggatott vonalat*/
		if(LINE_CNT != 2) //a kettőre váltás nem érdekel minket csak az 1 be vagy 3 ba váltás
		{
			lineCnt_prev = LINE_CNT; //az előző értéket a jelenlegihez hangoljuk
			tick_prev = tick;
		}

		/*****Lassító jelölés figyelése (folytonos 3 vonal)*****/
		if(LINE_CNT > 1) //ha 3 vonalat érzékelünk
		{
			if(tick > (start3time + BREAK_TIME_MS)) //ha már legalább BREAK_TIME_MS -idő óta folyamatosan 3 vonal van alattunk
			{
				motorDuty = 150;
				k_p = K_P_150;
				k_delta = K_DELTA_150;
				//HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
			}
		}
		else //ha 1 vonalat érzékelünk
		{
			start3time = tick;
		}
	}

	/*****SC üzemmód******/
	else if(swState[0]==SC_MODE)
	{
		dist=(((uint16_t)rxBuf[5])<<8) | ((uint16_t)rxBuf[6]);

		if(speed==GO_FAST) //Ha túl messze vagyunk a SC -tól
		{
			motorDuty = 250;
			k_p = K_P_250;
			k_delta = K_DELTA_250;

			if((dist < DIST_SLOW_MM) && rxBuf[4]) speed = GO_SLOW;
			if((dist < DIST_STOP_MM) && rxBuf[4]) speed = STOP;
		}
		else if(speed==GO_SLOW) //Ha túl közel vagyunk a SC-hoz
		{
			motorDuty = 150;
			k_p = K_P_150;
			k_delta = K_DELTA_150;

			if((dist > DIST_FAST_MM) && rxBuf[4]) speed = GO_FAST;
			if((dist < DIST_STOP_MM) && rxBuf[4]) speed = STOP;
		}
		else if(speed==STOP) //Ha a SC megállt
		{
			if((dist > DIST_STOP_MM) && rxBuf[4]) speed = GO_SLOW;
			motorDuty = 50; //ezzel már meg kell hogy álljon
		}


	}

	x_elso=((float)rxBuf[2]-123.5)*0.8158995;//*195/239;
	x_hatso=((float)rxBuf[3]-122.5)*0.8227848;//*195/237;
	delta=atan((float)(x_elso-x_hatso)/L_SENSOR);

	gamma = -k_p * x_elso -k_delta * delta;
	PHI = atan(((float)L/(L+D))*tan(gamma))*57.29578;//*180.0/3.1415;

	if(PHI<0) ccr = (uint16_t)(-SERVO_M * PHI + 593);//más a két irányba a szervóérzékenység
	else ccr = (uint16_t) (-0.6425*SERVO_M * PHI + 593); //különboző meredekséű egyenesek illesztünk
	//LED_B(1);
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
