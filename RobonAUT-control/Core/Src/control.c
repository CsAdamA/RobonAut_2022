/*
 * control.c
 *
 *  Created on: Jan 1, 2023
 *      Author: Zsombesz
 */

#include "control.h"
#include "main.h"
#include "configF4.h"
#include "line_track.h"
#include <stdio.h>
#include <string.h>
/**/

uint32_t nodeDetect=0;

////LEVI globals
uint8_t readytorace;
uint8_t pirate_pos[6];
volatile uint8_t uartThunder[6];
volatile uint8_t thunderboardFlag=0;

void Mode_Selector(UART_HandleTypeDef *huart_debugg, UART_HandleTypeDef *huart_stm)
{
	//Milyen módban kell működni?
	uint8_t buffer[30];
	uint32_t tmp=0;

	HAL_FLASH_Unlock();
	tmp= *(__IO uint32_t *) FLASH_ADDRESS_SECTOR7; //FLASH-ből kiolvassuk, hogy milyen módban vagyunk
	HAL_FLASH_Lock();
	mode = (uint8_t)tmp;

	if(mode==SKILL)
	{
		buffer[0] = CMD_MODE_SKILL; //szólunk a g0-nak, hogy ügyességi módban vagyunk
		HAL_UART_Transmit(huart_stm, buffer,1, 10);
		HAL_Delay(10);
		HAL_UART_Transmit(huart_stm, buffer,1, 10);//3 szór is szólunk neki, hogy tuti megkapja a módváltásról az üzenetet
		HAL_Delay(10);
		HAL_UART_Transmit(huart_stm, buffer,1, 10);

		sprintf(buffer,"Skill mode!\n\r"); //Debugg uart-ra is kiküldjük, hogy milyen módban vagyunk
		HAL_UART_Transmit(huart_debugg, buffer, strlen(buffer), 100);
		LED_NUCLEO(1); //A NUCLEO zöld LED-je világít, ha ügyeségi üzemmódban vagyunk
	}
	else if(mode==FAST)
	{
		buffer[0] = CMD_MODE_FAST;
		HAL_UART_Transmit(huart_stm, buffer,1, 10);
		HAL_Delay(10);
		HAL_UART_Transmit(huart_stm, buffer,1, 10);
		HAL_Delay(10);
		HAL_UART_Transmit(huart_stm, buffer,1, 10);

		sprintf(buffer,"Fast mode!\n\r");
		HAL_UART_Transmit(huart_debugg, buffer, strlen(buffer), 100);
		LED_NUCLEO(0);
	}
	else
	{
		sprintf(buffer,"Flash error! Press blue button\n\r");
		HAL_UART_Transmit(huart_debugg, buffer, strlen(buffer), 100);
	}


}

//bemenet detect, kalozrobpoz; kimenet direction
float Skill_Mode(UART_HandleTypeDef *huart_debugg)
{
	static float k_p = K_P_200;
	static float x_elso=0;
	static float x_elso_prev=0;
	static float gamma;
	int i;

	x_elso=0;
	for(i=0;i<rxBuf[1];i++)
	{
		x_elso += (float)rxBuf[i+2];
	}

	if(rxBuf[1]) x_elso /= rxBuf[1];

	x_elso = x_elso * 204/248.0-102;
	k_p =  -L/(v*v)*S1MULTS2_SLOW;
	gamma = -k_p * x_elso  - K_D*(x_elso-x_elso_prev);
	x_elso_prev = x_elso;

	return gamma;
}

void Detect_Node(UART_HandleTypeDef *huart_debugg, uint32_t t)
{
	static uint8_t detect_node_state=0;
	static uint32_t dt=0;
	static uint32_t t_prev=0;

	switch(detect_node_state)
	{
	case STEADY: //többször futó állapot
		if(rxBuf[1]==4) dt = t-t_prev;//mennyi ideje van alattunk 4 vonal
		else
		{
			t_prev=t;
			if(dt > TH_MIN(70))detect_node_state=QUAD_LINE_DETECTED;
		}
		break;

	case QUAD_LINE_DETECTED: //egyszer futó állapot
		if(dt > TH_MIN(70) && dt < TH_MAX(70) && rxBuf[1]==2) detect_node_state=MAYBE_HORIZONTAL_NODE_1;
		else if(dt > TH_MIN(200) && dt < TH_MAX(200) && rxBuf[1]==1) detect_node_state=VERTICAL_NODE_DETECTED;
		else detect_node_state=STEADY;
		break;

	case MAYBE_HORIZONTAL_NODE_1: //többször futó állapot
		if(rxBuf[1]==2) dt = t - t_prev;//mennyi ideje van alattunk 4 vonal
		else
		{
			t_prev=t;
			if(dt > TH_MIN(60))detect_node_state=MAYBE_HORIZONTAL_NODE_2;
			else detect_node_state=STEADY;
		}
		break;

	case MAYBE_HORIZONTAL_NODE_2: //egyszer futó állapot
		if(dt > TH_MIN(60) && dt < TH_MAX(60) && rxBuf[1]==4) detect_node_state=MAYBE_HORIZONTAL_NODE_3;
		else detect_node_state=STEADY;
		break;

	case MAYBE_HORIZONTAL_NODE_3:
		if(rxBuf[1]==4) dt = t - t_prev;//mennyi ideje van alattunk 4 vonal
		else
		{
			t_prev=t;
			if(dt > TH_MIN(70))detect_node_state=HORIZONTAL_NODE_DETECTED;
			else detect_node_state=STEADY;
		}
		break;

	case HORIZONTAL_NODE_DETECTED: //egyszer futó állapot
		if(rxBuf[1]==1) LED_B(1); //vízintes csomópont
		detect_node_state=STEADY;
		t_prev=t;
		break;

	case VERTICAL_NODE_DETECTED: //egyszer futó állapot
		if(rxBuf[1]==1) LED_B(1); //függőleges csomópont
		detect_node_state=STEADY;
		t_prev=t;
		break;

	}
}

uint32_t TH_MIN(uint32_t mm)
{
	return mm*900/(int32_t)v; //1000 nek kéne lenni a névleges időtartamhoz
}

uint32_t TH_MAX(uint32_t mm)
{
	return mm*2000/(int32_t)v;
}

void Monitoring_Task(UART_HandleTypeDef *huart_monitoring, int16_t sebesseg, uint8_t vonalszam, int32_t CCR, uint16_t tavolsag, uint32_t tick, uint32_t period)//csekkolni kell majd a typeokat!!!
{
	static uint32_t monitoring_tick=0;
	uint8_t data[11];
	uint8_t lower_data;
	uint8_t upper_data;

	//header
	data[0]=255;
	data[1]=255;
	data[2]=255;

	if(monitoring_tick>tick) return;
	monitoring_tick = tick + period;


	sebesseg=sebesseg+10000;
	lower_data= (uint8_t)sebesseg;
	upper_data= (uint8_t)(sebesseg>>8);

	data[3]=upper_data;
	data[4]=lower_data;

	lower_data= (uint8_t)vonalszam;
	upper_data= (uint8_t)(vonalszam>>8);

	data[5]=upper_data;
	data[6]=lower_data;

	lower_data= (uint8_t)CCR;
	upper_data= (uint8_t)(CCR>>8);

	data[7]=upper_data;
	data[8]=lower_data;

	lower_data= (uint8_t)tavolsag;
	upper_data= (uint8_t)(tavolsag>>8);

	data[9]=upper_data;
	data[10]=lower_data;


	HAL_UART_Transmit(huart_monitoring, data, 11, 4);
}

void GetBoardValue(UART_HandleTypeDef *huart_TB,UART_HandleTypeDef *huart_DEBUGG, uint32_t tick, uint32_t period){

	static uint8_t whichState=0;
	static uint32_t th_board_tick=0;
	static uint8_t str[]={'\n','\r'};

	if(th_board_tick>tick) return;
	th_board_tick= tick + period;


	if(thunderboardFlag==1){								//jott-e uzenet
		thunderboardFlag=0;
		if (whichState==0){									// visszaszamlalas												//ha itt elinditom ujra, uart th valtozo valtozik? vagy marad uyganez szoval jo mogotte is olvasni?(gondolom marad)
			if(uartThunder[0]=='0'){					//48-az a 0 hoz tartozo ascii		//start
				readytorace=1;							//extern valtozo, kulsoleg felhasználni
				whichState=1;
				HAL_UART_Receive_IT(huart_TB, uartThunder, 6);
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			}else HAL_UART_Receive_IT(huart_TB, uartThunder, 1);
		}																					//extern volatile uint8_t flag,buffer

		else if (whichState==1){							// pozicio lekérés

			pirate_pos[0]=uartThunder[0];//honnan
			pirate_pos[1]=uartThunder[1];//hová
			pirate_pos[2]=uartThunder[2];//kovi cel
			pirate_pos[3]=uartThunder[3];//elso szj
			pirate_pos[4]=uartThunder[4];//masodik szj
			pirate_pos[5]=uartThunder[5];//harmadik szj

			HAL_UART_Receive_IT(huart_TB, uartThunder, 6);

			HAL_UART_Transmit(huart_DEBUGG, pirate_pos, 6, 10);
			HAL_UART_Transmit(huart_DEBUGG, str, 2, 10);
		}
	}
}

void Uart_Receive_Thunderboard_ISR(UART_HandleTypeDef *huart)
{
	//LED_Y(1);	//Togglezzunk egy LED1-et
	thunderboardFlag=1;											//Globalis valtozo flag-et tegyuk 1-be, majd ha fogadtuk th.c-ben a uart-ot, vissza 0-ba
													//es inditsuk ujra a varakozast: HAL_UART_Receive_IT(huart, fromPC, 1)
}
