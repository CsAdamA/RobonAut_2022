/*
 * config.c
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */


#include "configF4.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// hadc2;huart


uint8_t swState[]={0,0};
uint8_t fromPC[]={150};

void F4_Basic_Init(UART_HandleTypeDef *huart_debugg,TIM_HandleTypeDef *htim_scheduler,TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_servo, TIM_HandleTypeDef *htim_encoder)
{
	uint8_t buf[30];
	LED_R(0);
	LED_B(0);
	LED_G(0);
	LED_Y(0);
	memset(buf,0,30); //a buf tömböt feltöltöm 0-kkal
	sprintf(buf,"RobonAUT 2022 Bit Bangers F4\r\n");// a buff tömb-be beleírom (stringprint) a string-emet. 1 karakter = 1 byte = 1 tömbelem
	HAL_UART_Transmit(huart_debugg, buf, strlen(buf), 100);// A UART2-őn (ide van kötve a programozó) kiküldöm a buf karaktertömböt (string) és maximum 10-ms -ot várok hogy ezt elvégezze a periféria

	//MotorEnable engedélyezése
	motorEnRemote=0;//csak akkor ha megnyomtuk a ravaszt
	motorEnLineOk=1;

	//timerek elindítása
	TIM2->CCR1=684; //servot középre
	TIM3->CCR1=499; //0 kitöltési tényező a motorra
	TIM3->CCR2=499;
	HAL_TIM_Base_Start(htim_scheduler);//heart beat timer tick start
	HAL_TIM_PWM_Start(htim_motor, TIM_CHANNEL_1);//motor PWM-ek elindítása
	HAL_TIM_PWM_Start(htim_motor, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim_servo, TIM_CHANNEL_1); //servo RC pwm elindítása
	HAL_TIM_Encoder_Start(htim_encoder,TIM_CHANNEL_ALL);

	//Ha a PC-ről küldünk vmit azt fogadjuk
	HAL_UART_Receive_IT(huart_debugg, fromPC, 1);

}


void Meas_Bat_Task(ADC_HandleTypeDef *hadc,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{
	uint16_t raw;
	float bat;
	static char msg[20];
	static uint32_t meas_bat_tick=0;


	if(meas_bat_tick>tick) return;
	meas_bat_tick= tick + period;

	//get ADC value
	HAL_ADC_Start(hadc);
	HAL_ADC_PollForConversion(hadc, 3);
	raw = HAL_ADC_GetValue(hadc);

	//Raw to Volt
	bat=(float)raw*0.00458953168044077134986225895317;

	//Print Value
	sprintf(msg,"NI-MH feszultsege: %3.2f [V] (%d)\r\n",bat,raw);
	HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),3);


	memset(msg,0,30);
	if (raw<1612 && EN_FB) //ha be van kapcolva a motorvezérlő és az akkuja feszültsége alacsony
	{
		meas_bat_tick= tick + period/10;
		HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
		sprintf(msg,"Toltes szukseges\r\n");
		HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg),10);

		//MotorEnable kikapcsolása ha akksi fesz beesik.
		//motorEnBattOk=0;

	}
	else
	{
		LED_Y(0);
		//motorEnBattOk=1;
	}


}


void SW_Read_Task(uint32_t tick, uint32_t period)
{
	static uint32_t sw_read_task_tick=0;

	if(sw_read_task_tick>tick) return;
	sw_read_task_tick = tick + period;

	swState[0]=SW1;
	swState[1]=SW2;

	if(swState[0]) LED_G(1);
	else LED_G(0);
	/*if(swState[1]) LED_B(1);
	else LED_B(0);
	*/
}

void Uart_Receive_From_PC_ISR(UART_HandleTypeDef *huart)
{
	HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
	HAL_UART_Receive_IT(huart, fromPC, 1);
	TIM2->CCR1 = 4*fromPC[0];
}



