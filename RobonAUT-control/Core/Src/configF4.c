/*
 * config.c
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */


#include "configF4.h"
#include "main.h"
#include "control.h"
#include "line_track.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// hadc2;huart


uint8_t swState[2];
volatile uint8_t bFlag[3];
uint8_t fromPC[2];
uint8_t mode;
float v_ref; //mm/s
uint8_t leaveLineEnabled;

void F4_Basic_Init(UART_HandleTypeDef *huart_debugg,TIM_HandleTypeDef *htim_scheduler,TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_servo1,TIM_HandleTypeDef *htim_servo2, TIM_HandleTypeDef *htim_encoder,TIM_HandleTypeDef *htim_delay,TIM_HandleTypeDef *htim_rand)
{
	char buf[40];
	LED_R(0);
	LED_B(0);
	LED_G(0);
	LED_Y(0);
	LED_NUCLEO(0);
	sprintf(buf,"\r\nRobonAUT 2023 Bit Bangers F4\r\n");// a buff tömb-be beleírom (stringprint) a string-emet. 1 karakter = 1 byte = 1 tömbelem
	HAL_UART_Transmit(huart_debugg,(uint8_t*) buf, strlen(buf), 100);// A UART2-őn (ide van kötve a programozó) kiküldöm a buf karaktertömböt (string) és maximum 10-ms -ot várok hogy ezt elvégezze a periféria

	//MotorEnable engedélyezése
	motorEnRemote=0;//csak akkor ha megnyomtuk a ravaszt
	motorEnLineOk=1;
	leaveLineEnabled=0;

	swState[0] = SW1;
	swState[1] = SW2;
	if(SW1)
	{
		LED_G(1);
		boostCnt=11;
	}
	else
	{
		LED_G(0);
		boostCnt=0;
	}
	if(SW2)LED_R(1);
	else LED_R(0);

	bFlag[0] = bFlag[1] = bFlag[2] = 0;
	fromPC[0]=150;
	fromPC[1]=150;
	mode=SKILL;
	v_ref = 1000;
	v=0;



	//timerek elindítása
	TIM1->CCR4=SERVO_REAR_CCR_MIDDLE;
	TIM2->CCR1=SERVO_FRONT_CCR_MIDDLE;
	//TIM2->CCR1=684; //servot középre
	//TIM3->CCR1=499; //0 kitöltési tényező a motorra
	//TIM3->CCR2=499;
	HAL_TIM_Base_Start(htim_scheduler);//heart beat timer tick start
	HAL_TIM_Base_Start(htim_delay);//heart beat timer tick start
	HAL_TIM_Base_Start(htim_rand);//heart beat timer tick start
	HAL_TIM_PWM_Start(htim_motor, TIM_CHANNEL_1);//motor PWM-ek elindítása
	HAL_TIM_PWM_Start(htim_motor, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(htim_servo1, TIM_CHANNEL_1); //servo RC pwm elindítása
	HAL_TIM_PWM_Start(htim_servo2, TIM_CHANNEL_4); //servo RC pwm elindítása
	HAL_TIM_Encoder_Start(htim_encoder,TIM_CHANNEL_ALL);

	//Ha a PC-ről küldünk vmit azt fogadjuk
	//HAL_UART_Receive_IT(huart_debugg, fromPC, 2);
}


void HDI_Read_Task(UART_HandleTypeDef *huart_debugg,TIM_HandleTypeDef *htim_servo,uint32_t tick, uint32_t period)
{
	static uint32_t hdi_read_task_tick=0;

	if(hdi_read_task_tick>tick) return;
	hdi_read_task_tick = tick + period;

	if(SW1)
	{
		LED_G(1);
		swState[0]=1;
	}
	else
	{
		LED_G(0);
		swState[0]=0;
	}
	if(SW2)
	{
		LED_R(1);
		swState[1]=1;
	}
	else
	{
		LED_R(0);
		swState[1]=0;
	}
}

void Uart_Receive_From_PC_ISR(UART_HandleTypeDef *huart)
{
	LED_Y_TOGGLE;
	HAL_UART_Receive_IT(huart, (uint8_t*)fromPC, 2);
	TIM2->CCR1 = 4*fromPC[0];
	TIM1->CCR4 = 4*fromPC[1];
}

void B1_ISR(UART_HandleTypeDef *huart_debugg)
{
		HAL_FLASH_Unlock();
		Delay(50);
		FLASH_Erase_Sector(6, FLASH_VOLTAGE_RANGE_3);
		Delay(50);
		HAL_FLASH_Lock();

		HAL_FLASH_Unlock();
		Delay(50);
		int i;
		for(i=0;i<22;i++)
		{
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_NODEWORTH+i, Nodes[i].worth);
		}
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_NODEWORTH+22, collectedPoints);
		Delay(50);
		HAL_FLASH_Lock();
		HAL_UART_Transmit(huart_debugg,(uint8_t*) "\n\rBackup save!\n\r", 16, 10);
}

void B_NUCLEO_ISR(UART_HandleTypeDef *huart_debugg)
{
	//Milyen módban voltunk eddig?
	uint8_t tmp= *(__IO uint8_t *) FLASH_ADDRESS_MODESELECTOR;
	if(tmp==SKILL || tmp==FAST)mode=tmp;
	else mode=SKILL;

	//section 7 törlése, hogy újraírhassuk a módot jelző bytot
	HAL_FLASH_Unlock();
	Delay(50);
	FLASH_Erase_Sector(7, FLASH_VOLTAGE_RANGE_3);
	Delay(50);
	HAL_FLASH_Lock();

	LED_NUCLEO(0);
	LED_Y(0);
	LED_G(0);
	LED_B(0);
	LED_R(0);
	int i;
	for(i=0;i<8;i++)
	{
		LED_NUCLEO_TOGGLE;
		LED_Y_TOGGLE;
		LED_G_TOGGLE;
		LED_B_TOGGLE;
		LED_R_TOGGLE;
		Delay(150);
	}

	//Állítsuk át a módot
	HAL_FLASH_Unlock();
	Delay(50);
	if(mode==SKILL) HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_MODESELECTOR, FAST); //ha eddig skill mód volt akor msot gyors lesz
	else HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_MODESELECTOR, SKILL); //ha eddig gyors mód vagy memóriaszemét volt akkor msot skil lesz
	Delay(50);
	HAL_FLASH_Lock();
	Delay(50);
	HAL_UART_Transmit(huart_debugg, (uint8_t*)"\n\rMode change!\n\r", 16, 10);
	NVIC_SystemReset(); //SW reseteljük a mikorvezérlőt
}

