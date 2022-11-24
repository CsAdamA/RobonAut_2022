/*
 * config.c
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */


#include <configF4.h>
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// hadc2;huart

void F4_Basic_Init(UART_HandleTypeDef *huart,TIM_HandleTypeDef *htim,TIM_HandleTypeDef *htim3,TIM_HandleTypeDef *htim2)
{
	uint8_t buf[30];
	LED_R(0);
	LED_B(0);
	LED_G(0);
	LED_Y(0);
	memset(buf,0,30); //a buf tömböt feltöltöm 0-kkal
	sprintf(buf,"RobonAUT 2022 Bit Bangers\r\n");// a buff tömb-be beleírom (stringprint) a string-emet. 1 karakter = 1 byte = 1 tömbelem
	HAL_UART_Transmit(huart, buf, strlen(buf), 100);// A UART2-őn (ide van kötve a programozó) kiküldöm a buf karaktertömböt (string) és maximum 10-ms -ot várok hogy ezt elvégezze a periféria
	HAL_TIM_Base_Start(htim);//heart beat timer tick start

	//MotorEnable engedélyezése
	motorEnRemote=1;
	motorEnBattOk=1;
	motorEnLineOk=1;

	//kezdeti pwm kitoltes megadasa->0 hiszen nem akarjuk h forogjon
	TIM3->CCR1=0;
	TIM3->CCR2=0;
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim2, TIM_CHANNEL_1);
	TIM2->CCR1=600;
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
	if (raw<1600 && EN_FB) //ha be van kapcolva a motorvezérlő és az akkuja feszültsége alacsony
	{
		meas_bat_tick= tick + period/10;
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		sprintf(msg,"Toltes szukseges\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),10);

		//MotorEnable kikapcsolása ha akksi fesz beesik.
		motorEnBattOk=0;

	}
	else
	{
		LED_Y(0);
		motorEnBattOk=1;
	}


}


void Read_G0_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug, uint32_t tick, uint32_t period)
{

#ifdef G0_DEBUG
	//uint8_t str[30];
#endif
	uint8_t str[40];
	uint32_t dist=0;
	uint8_t txBuf[]={CMD_READ};
	uint8_t rxBuf[]={0,0,0,0,0,0,0,0};
	static uint32_t read_g0_task_tick=0;
	static float d=85;
	static float x_elso;
	static float x_hatso;
	static float p;
	static float L_sensor=250;
	static float L=272;
	static  float k_p=-0.00054389;
	static  float k_delta=-0.3961;
	static float PHI;
	static float delta;
	static float gamma;



	if(read_g0_task_tick>tick) return;
	read_g0_task_tick = tick + period;


	HAL_UART_Transmit(huart_stm, txBuf,1, 1);
	HAL_UART_Receive(huart_stm, rxBuf, 8, 2);
	if((rxBuf[0]==START_BYTE && rxBuf[7]==STOP_BYTE)) //jöt adat a G0 tól és a keret is megfelelő
	{
		LED_G(1);
		dist=(((uint16_t)rxBuf[5])<<8) | ((uint16_t)rxBuf[6]);
#ifdef G0_DEBUG
		sprintf(str,"LS: %1d, %3d, %3d   -   TOF1: %1d, %4d \n\r", rxBuf[1],rxBuf[2],rxBuf[3], rxBuf[4],dist);
		HAL_UART_Transmit(huart_debug, str, strlen(str), 10);
		read_g0_task_tick+=1000;
#endif
		if (rxBuf[1]<1)
		{
			motorEnLineOk=0;
			return;
		}
		else
		motorEnLineOk=1;

		x_elso=((float)rxBuf[2]-123.5)*195/239;
		x_hatso=((float)rxBuf[3]-122.5)*195/237;
		p=x_elso;

		delta=atan((float)(x_elso-x_hatso)/L_sensor);
		gamma = -k_p*p -k_delta*delta;

		PHI=atan(((float)L/(L+d))*tan(gamma))*180.0/3.1415;
		TIM2->CCR1= (uint16_t)(-30 * PHI + 593);


/*		sprintf(str,"Phi értéke: %.2f  ,p:%.2f Delta erteke: %.2f \n\r", PHI*180.0/3.1415, p, delta*180.0/3.1415);
		HAL_UART_Transmit(huart_debug, str, strlen(str), 50);
		read_g0_task_tick+=1000;
*/

	}
	else
	{
		LED_G(0);
#ifdef G0_DEBUG
		sprintf(str,"G0 read error!\n\r");
		HAL_UART_Transmit(huart_debug, str, strlen(str), 10);
		read_g0_task_tick+=1000;
#endif
	}




}
