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
#include <math.h>
/**/

uint8_t orientation=FORWARD;
uint8_t nodeDetected=0; //érzékeltünk a node jelölőt
uint8_t path=LEFT; //3-as utelágazásnál melyik irányba menjünk?
uint8_t collectedPoints=0;
uint8_t laneChange=0;

volatile uint8_t thunderboardFlag=0;
uint8_t tb_msg[6];
uint8_t piratePos[4];


////LEVI globals
uint8_t readytorace=0;

volatile uint8_t uartThunder[6];

node Nodes[25];

void Create_Nodes(UART_HandleTypeDef *huart_debugg)
{
	int i;
	orientation=FORWARD;
	nodeDetected=1;
	collectedPoints=0;
	laneChange=0;

	for(i=0;i<25;i++)
	{
		Nodes[i].id=65+i;
		Nodes[i].worth=0;
		Nodes[i].type=0;
		VALUE(Nodes[i].neighbours,0,0,0,0);
		VALUE(Nodes[i].directions,0,0,0,0);
		VALUE(Nodes[i].distance,0,0,0,0);
	}
	//A node
	N('A').worth=0;
	N('A').type=1;
	VALUE(N('A').neighbours,0,0,0,'C');
	VALUE(N('A').directions,0,0,0,2);
	VALUE(N('A').distance,0,0,0,365);

	//B node
	N('B').worth=2;
	N('B').type=2;
	VALUE(N('B').neighbours,'D',0,'C',0);
	VALUE(N('B').directions,2,0,2,0);
	VALUE(N('B').distance,452,0,168,0);

	//C node
	N('C').worth=2; //beragadás ellen
	N('C').type=3;
	VALUE(N('C').neighbours,0,'B','E',0);
	VALUE(N('C').directions,0,1,2,0);
	VALUE(N('C').distance,0,218,130,0);

	//D node
	N('D').worth=2;
	N('D').type=1;
	VALUE(N('D').neighbours,'B',0,'F',0);
	VALUE(N('D').directions,2,0,2,0);
	VALUE(N('D').distance,452,0,316,0);

	//E node
	N('E').worth=2; //beragadás ellen
	N('E').type=3;
	VALUE(N('E').neighbours,'C',0,'F','G');
	VALUE(N('E').directions,1,0,2,2);
	VALUE(N('E').distance,130,0,428,385);

	//F node
	N('F').worth=2;
	N('F').type=1;
	VALUE(N('F').neighbours,'E','D','H','I');
	VALUE(N('F').directions,1,1,2,2);
	VALUE(N('F').distance,428,316,284,335);

	//G node
	N('G').worth=2;
	N('G').type=1;
	VALUE(N('G').neighbours,'E',0,'H','I');
	VALUE(N('G').directions,1,0,2,2);
	VALUE(N('G').distance,385,0,336,284);

	//H node
	N('H').worth=2;
	N('H').type=1;
	VALUE(N('H').neighbours,'G','F','K','J');
	VALUE(N('H').directions,1,1,2,2);
	VALUE(N('H').distance,336,284,407,230);

	//I node
	N('I').worth=0;
	N('I').type=3;
	VALUE(N('I').neighbours,'G','F',0,'L');
	VALUE(N('I').directions,1,1,0,2);
	VALUE(N('I').distance,284,335,0,418);

	//J node
	N('J').worth=0;
	N('J').type=3;
	VALUE(N('J').neighbours,'H',0,'K','L');
	VALUE(N('J').directions,1,0,2,2);
	VALUE(N('J').distance,230,0,204,229);

	//K node
	N('K').worth=2;
	N('K').type=1;
	VALUE(N('K').neighbours,'J','H','M','N');
	VALUE(N('K').directions,1,1,2,2);
	VALUE(N('K').distance,204,407,288,319);

	//L node
	N('L').worth=2;
	N('L').type=1;
	VALUE(N('L').neighbours,'I','J','M','N');
	VALUE(N('L').directions,1,1,2,2);
	VALUE(N('L').distance,418,229,329,258);

	//M node
	N('M').worth=2;
	N('M').type=1;
	VALUE(N('M').neighbours,'L','K','P','O');
	VALUE(N('M').directions,1,1,2,2);
	VALUE(N('M').distance,329,288,416,198);

	//N node
	N('N').worth=0;
	N('N').type=3;
	VALUE(N('N').neighbours,'L','K','O','Q');
	VALUE(N('N').directions,1,1,2,2);
	VALUE(N('N').distance,258,318,228,447);

	//O node
	N('O').worth=2;
	N('O').type=1;
	VALUE(N('O').neighbours,'N','M','P',0);
	VALUE(N('O').directions,1,1,2,0);
	VALUE(N('O').distance,228,198,248,0);

	//P node
	N('P').worth=2;
	N('P').type=1;
	VALUE(N('P').neighbours,'O','M','R','S');
	VALUE(N('P').directions,1,1,2,2);
	VALUE(N('P').distance,248,416,305,346);

	//Q node
	N('Q').worth=2;
	N('Q').type=1;
	VALUE(N('Q').neighbours,'N',0,'R','S');
	VALUE(N('Q').directions,1,0,2,2);
	VALUE(N('Q').distance,447,0,346,284);

	//R node
	N('R').worth=2;
	N('R').type=1;
	VALUE(N('R').neighbours,'Q','P','U','T');
	VALUE(N('R').directions,1,1,2,2);
	VALUE(N('R').distance,346,305,366,204);

	//S node
	N('S').worth=2;
	N('S').type=1;
	VALUE(N('S').neighbours,'Q','P','T','V');
	VALUE(N('S').directions,1,1,2,2);
	VALUE(N('S').distance,284,346,223,406);

	//T node
	N('T').worth=2;
	N('T').type=1;
	VALUE(N('T').neighbours,'S','R','U','V');
	VALUE(N('T').directions,1,1,2,2);
	VALUE(N('T').distance,223,204,192,233);

	//U node
	N('U').worth=2;
	N('U').type=1;
	VALUE(N('U').neighbours,'T','R','X',0);
	VALUE(N('U').directions,1,1,2,0);
	VALUE(N('U').distance,192,366,371,0);

	//V node
	N('V').worth=2;
	N('V').type=1;
	VALUE(N('V').neighbours,'S','T','W',0);
	VALUE(N('V').directions,1,1,2,0);
	VALUE(N('V').distance,406,233,149,0);

	//W node
	N('W').worth=0;
	N('W').type=3;
	VALUE(N('W').neighbours,'V',0,'X',0);
	VALUE(N('W').directions,1,0,1,0);
	VALUE(N('W').distance,149,0,189,0);

	//X node
	N('X').worth=2;
	N('X').type=2;
	VALUE(N('X').neighbours,'U',0,0,'W');
	VALUE(N('X').directions,1,0,0,1);
	VALUE(N('X').distance,371,0,0,189);

	//Y node
	/**/N('Y').worth=0;
	N('Y').type=1;
	VALUE(N('Y').neighbours,'W',0,0,0);
	VALUE(N('Y').directions,1,0,0,0);
	VALUE(N('Y').distance,351,0,0,0);

	//Nodeértékek backup mentésből való visszatöltése
	if(SW2)//ha a kacsapoló2 a megfelelő állapotban van (világít a sárga LED)
	{
		uint32_t check_flash = *(__IO uint32_t *) FLASH_ADDRESS_NODEWORTH; //tényleg ottvanak  flashbena megfelelő helyen a worth értékek?

		if(check_flash==0xffffffff)//nincs semmi a flashben
		{
			char str[]="Default worths because of FLASH ERROR!\n\r";
			HAL_UART_Transmit(huart_debugg,(uint8_t*) str, strlen(str), 10);
			HAL_FLASH_Unlock();
			HAL_Delay(50);
			FLASH_Erase_Sector(6, FLASH_VOLTAGE_RANGE_3);
			HAL_Delay(50);
			HAL_FLASH_Lock();
			HAL_Delay(50);
			HAL_FLASH_Unlock();
			HAL_Delay(50);
			for(i=0;i<25;i++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_NODEWORTH+i, Nodes[i].worth);
			}
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_NODEWORTH+25, collectedPoints);
			HAL_Delay(50);
			HAL_FLASH_Lock();
			return; //ha nem akkor használjuk a default értékeket
		}
		for(i=0;i<25;i++)
		{
			Nodes[i].worth=*(__IO uint8_t *) (FLASH_ADDRESS_NODEWORTH+i); //ha igen akkor töltsük be a backup mentést
		}
		collectedPoints=*(__IO uint8_t *) (FLASH_ADDRESS_NODEWORTH+25);
		char str[]="Worths from FLASH backup!\n\r";
		HAL_UART_Transmit(huart_debugg,(uint8_t*) str, strlen(str), 10);
	}
	else
	{
		char str[]="Default worths!\n\r";
		HAL_UART_Transmit(huart_debugg,(uint8_t*) str, strlen(str), 10);
	}

}


void Control_Task(UART_HandleTypeDef *huart_debugg,TIM_HandleTypeDef *htim_rand,uint32_t tick, uint32_t period)
{
	static uint8_t pos[2]	=	{'Y','W'}; 				//my, next
	static uint8_t dir[2]	=	{1,1}; 					//my, next
	static uint8_t bestNb[2]=	{NEIGHBOUR1,NEIGHBOUR1};//tmp, next
	static uint8_t nextOri	=	FORWARD;
	static uint8_t nextPath	=	LEFT;

	//static uint32_t last_tb_msg=0;
	static uint32_t tick_prev=0;
	static float s=0;
	static uint32_t sMAX=351;
	static float fitness[4]={0,0,0,0};
	static float bestFitness=-150;
	static uint8_t piratePos_prev[]={'P','M','K',0};

	static uint32_t control_task_tick = 0;
	static uint8_t control_task_state=NEIGHBOUR1;//5 db állapot ->5.után megint 1.jön
	//szomszéd1,szomszéd2,szomszéd3,szomszéd4,kiértékelés

	uint8_t nID=0;

	if(control_task_tick>tick)return;
	control_task_tick=tick+period;
	if(mode!=SKILL)return;
	//if(!readytorace)return;

	//ha kapu nélküli nodeba tartunk éppen, akkor időzítéssel "detektáljuk" a nodot
	if(N(pos[MY]).type>2)
	{
		s += (float)(tick-tick_prev)*abs((int)v)/10000;
		if(s>sMAX)nodeDetected=1;

	}
	tick_prev=tick;//mostantól mérjük az időt

	//ha odaértünk a myPositionbe, akkor indulhat a mozgás a nextPosition felé
	if(nodeDetected)
	{
		LED_B_TOGGLE;
		if(N(pos[NEXT]).type>2)//ha a kövi node-on nincs kapu
		{
			s=0;
			sMAX=N(pos[MY]).distance[bestNb[NEXT]]+25;
		}

		//pontok nyugtázása
		if(!laneChange)//ha nem sávváltó üzemmódban vagyunk pontotszámolunk és felszedett kapukat nullázzuk
		{
			collectedPoints +=N(pos[MY]).worth;//sávváltás módik vizsgáljuk az össezgyűjtött kapuk számát
			N(pos[MY]).worth=0;//ez a kapu már nem ér pontot
		}

		if(collectedPoints >= 22 && !laneChange) //átváltás lane change módba
		{
			laneChange=1; //flag állítás
			Lane_Change_Init(); //a sávváltóhely felé nőnek a rewardok
			LED_Y(1); //sárga led világít
		}

		if(laneChange==1 && pos[MY]=='N' && pos[NEXT]=='Q')//ha a tett színhelyén vagyunk
		{
			laneChange=2;
		}

		static char str[50]; //kiiratás
		sprintf(str,"d,d,%2d,%3.2f,%3.2f,%3.2f,%3.2f\n\r",(int)collectedPoints,fitness[0],fitness[1],fitness[2],fitness[3]);
		str[0]=pos[MY];
		str[2]=pos[NEXT];
		HAL_UART_Transmit(huart_debugg, (uint8_t*)str, strlen(str), 4);

		pos[MY]=pos[NEXT];
		path=nextPath;
		dir[MY]=dir[NEXT];
		orientation=nextOri;//FORWARD

		control_task_state=NEIGHBOUR1;
		nodeDetected=0;
	}

	if(thunderboardFlag)//ha új kalózpozíció jött a TB-től ujrakezdjük a számolást (első szomszéd vizsgálata jön)
	{
		if(piratePos_prev[1]!=piratePos[1] && !laneChange)//a kalóz átment egy Node-on
		{
			if(N(piratePos[0]).worth==2)N(piratePos[0]).worth=1; //az a node már kevesebbet ér
			else N(piratePos[0]).worth=0;
		}
		control_task_state=NEIGHBOUR1;//kezdjük előrröl a fitneszérték számítást az 1. szomszédtól

		piratePos_prev[0]=piratePos[0];//előző kalozpozíció frissítése
		piratePos_prev[1]=piratePos[1];
		piratePos_prev[2]=piratePos[2];
		piratePos_prev[3]=piratePos[3];

		thunderboardFlag=0; //várjuk az újabb kalózrobot pozíciókat a thunderboardtól
	}
	//if(control_task_state>EVALUATE)return;//ha már kiértékelés is megvolt akkor nincs mit számolni
#ifdef CONTROL_DEBUGG
	char str[]="Control state: d\n\r";
	str[15]=control_task_state+0x30;
	HAL_UART_Transmit(huart_debugg, (uint8_t*)str, strlen(str), 2);
#endif

	/******************LEGJOBB SZOMSZÉD KIVÁLASZTÁSA (első 4 állapot)******************/
	if(control_task_state <EVALUATE)//1.szomszéd/2.szomszéd/3.szomszéd/4.szomszéd
	{
		if(control_task_state==NEIGHBOUR1)
		{
			bestFitness=-100;//az előző számolás legjob fitneszértéke volt még benne
		}
		nID=N(pos[MY]).neighbours[control_task_state]; //a vizsgált 1.rendű szomszéd azonosítója
		if(nID) //ha létezik a szomszéd
		{
			fitness[control_task_state]=(float)N(nID).worth; //fitneszérték 1.rendű szomszéd alapján
			//kalozrobot hatása az 1.rendű szomszéd esetén
			if(piratePos[1]==nID) fitness[control_task_state] -= 80/*P*/;//ha a kalóz is ebbe az 1.rendű tart éppen akkor kerüljük el az ütközést
			else if(piratePos[2]==nID) fitness[control_task_state] -= 40/*P*/;//ha még csak tervezi, hogy odamegy, akkor is kerüljük a pontot
			int i;
			uint8_t nnID;
			float nnFit;
			for(i=0;i<4;i++)//2.rednű szomszédok
			{
				nnFit=0.0;
				nnID=N(nID).neighbours[i]; //2.rednű szomszéd ID-ja
				if(nnID && nnID!=pos[MY])//ha létezik a 2.rendű szomszéd (és nem a myposition az)
				{
					nnFit += (float)N(nnID).worth;
					if(piratePos[1]==nnID) nnFit -= 1/*P*/;//ha a kalóz is ebbe a pontba tart éppen akkor kerüljük el az ütközést
					else if(piratePos[2]==nnID) fitness[control_task_state] -= 0.5/*P*/;//ha még csak tervezi, hogy odamegy, akkor se fogjuk tudni megelőnzi, mert mi 3 nodnyira vagyunk ő pedig csak 2
					//if(!lane_change)nnFit = nnFit * (float)DIST_AVG/N(nID).distance[i];//a 2.rendű szomszédhoz tartozó fitneszérték jobb ha az közelebb van az 1.rendű szomszédjához
					//ha a sávváltó szakaszt keressük akkor viszont nem díjazzuk a közelséget
					fitness[control_task_state] += nnFit/4/*P*/;
				}

			}
			//if(!lane_change) fitness[control_task_state] =fitness[control_task_state] * (float)DIST_AVG/N(pos[MY]).distance[control_task_state]; //minél közelebb van a szomszéd annál jobb
			//ha a sávváltó szakaszt keressük akkor viszont nem díjazzuk a közelséget

		}
		else fitness[control_task_state]=-150.0;//ha nem létezik a szomszéd erre tuti ne menjünk
		uint16_t tmp= __HAL_TIM_GET_COUNTER(htim_rand)%2;
		if((fitness[control_task_state]==bestFitness && tmp==1) || fitness[control_task_state]>bestFitness) //ha ez a fitness jobb mint az eddigi legjobb, akkor mostantól ez a legjobb
		{
			bestFitness=fitness[control_task_state];
			bestNb[TMP] = control_task_state;//ez az egy érték amivel a task első 4 (fitnesszámoló) álapota kommunikál a kiértékelő álapottal
		}
		control_task_state++;
		return; //ha csak valamelyik szomszédot vizsgáltuk még akkor eddig tartott ez a task fuitás, itt kilépünk
	}
	/**************************************************************************************/
	//ide csak akkor jutunk el ha control_task_state>NEIGHBOUR4

	/**********************KIÉRTÉKELÉS (control_task_state=EVALUATE ->5.állapot)**********************/
	bestNb[NEXT]=bestNb[TMP];
	pos[NEXT]=N(pos[MY]).neighbours[bestNb[NEXT]];//a következő poziciónk a legjobb szomszéd lesz
	dir[NEXT]=N(pos[MY]).directions[bestNb[NEXT]];//már most tudjuk, mi lesz az irányunk, ha odaértünk

	//a kocsi az egyik node-ból átmegy egy másikba-> az irányok segítségével meghatározzu az új orientationt
	if(bestNb[NEXT] <= NEIGHBOUR2) //ha balra/le kell majd mennünk a nextPosition -höz
	{
		if(dir[MY]==2)//és eddig jobbra/fel mentünk,
			nextOri = !orientation;//akkor most orientációt kell váltanunk
		else nextOri = orientation; //különben nem kell
	}
	else //ha jobbra kell majd mennünk
	{
		if(dir[MY]==1)//és eddig jobbra/fel mentünk,
			nextOri =! orientation;//akkor most irányt kell váltanunk
		else nextOri = orientation; //különben nem kell
	}

	//path kiválasztás -> az orientációt mostmár tudjuk (tolatás/előre), már csak az ösvény kell kivákasztani, hogy a megfelelő szomszédhoz jussunk
	if(nextOri==FORWARD)
	{
		if(bestNb[NEXT]==NEIGHBOUR1 || bestNb[NEXT]==NEIGHBOUR3)nextPath=LEFT;
		else if(bestNb[NEXT]==NEIGHBOUR2 || bestNb[NEXT]==NEIGHBOUR4)nextPath=RIGHT;
	}
	else if(nextOri==REVERSE) //tolatásnál pont forditva vannak a pathirányok
	{
		if(bestNb[NEXT]==NEIGHBOUR1 || bestNb[NEXT]==NEIGHBOUR3)nextPath=RIGHT;
		else if(bestNb[NEXT]==NEIGHBOUR2 || bestNb[NEXT]==NEIGHBOUR4)nextPath=LEFT;
	}
	control_task_state=NEIGHBOUR1;
	//HAL_UART_Transmit(huart_debugg, (uint8_t*)"##\r\n", 4, 2);
	/**************************************************************************************/
	return;
}


void Mode_Selector(UART_HandleTypeDef *huart_debugg, UART_HandleTypeDef *huart_stm)
{
	//Milyen módban kell működni?
	uint8_t buffer[40];
	uint8_t tmp=*(__IO uint8_t *) FLASH_ADDRESS_MODESELECTOR; //FLASH-ből kiolvassuk, hogy milyen módban vagyunk

	if(tmp==SKILL || tmp==FAST) mode = tmp;
	else
	{
		HAL_FLASH_Unlock();
		HAL_Delay(50);
		FLASH_Erase_Sector(7, FLASH_VOLTAGE_RANGE_3);
		HAL_Delay(50);
		HAL_FLASH_Lock();
		HAL_FLASH_Unlock();
		HAL_Delay(50);
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_MODESELECTOR, SKILL); //ha eddig skill mód volt akor msot gyors lesz
		HAL_Delay(50);
		HAL_FLASH_Lock();

		mode=SKILL;
	}

	if(mode==SKILL)
	{
		buffer[0] = CMD_MODE_SKILL; //szólunk a g0-nak, hogy ügyességi módban vagyunk
		HAL_UART_Transmit(huart_stm, buffer,1, 10);
		HAL_Delay(10);
		HAL_UART_Transmit(huart_stm, buffer,1, 10);//3 szór is szólunk neki, hogy tuti megkapja a módváltásról az üzenetet
		HAL_Delay(10);
		HAL_UART_Transmit(huart_stm, buffer,1, 10);

		sprintf((char*)buffer,"Skill mode!\n\r"); //Debugg uart-ra is kiküldjük, hogy milyen módban vagyunk
		HAL_UART_Transmit(huart_debugg, buffer, strlen((char*)buffer), 100);
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

		sprintf((char*)buffer,"Fast mode!\n\r");
		HAL_UART_Transmit(huart_debugg, buffer, strlen((char*)buffer), 100);
		LED_NUCLEO(0);
	}
	else
	{
		sprintf((char*)buffer,"Flash error! Press blue button!\n\r");
		HAL_UART_Transmit(huart_debugg, buffer, strlen((char*)buffer), 100);
	}
}

//bemenet detect, kalozrobpoz; kimenet direction


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


void Wait_For_Start_Sigal(UART_HandleTypeDef *huart_TB, UART_HandleTypeDef *huart_debugg)
{
	uint8_t rcv[]={0};
	static uint8_t cnt=5;
	if(mode==FAST)return;
	while(1)
	{
		if(SW2) //Gombal töerténő indítás
		{
			if(B2)
			{
				LED_R(0);//kilaszik a sárga fény pár másodpercre amiíg el nem indul a robot
				break;//ha megnyomtuka 2-es gombot kiugrunk a while ciklusból
			}
		}
		else //Bluetooth-on érkezika  start jel
		{
			HAL_UART_Receive(huart_TB, rcv, 1, HAL_MAX_DELAY);
			if(rcv[0]==cnt+0x30)
			{
				if(cnt<4)
				{
					HAL_UART_Transmit(huart_debugg, rcv, 1, 2);
					HAL_UART_Transmit(huart_debugg, (uint8_t*)"\n\r", 2, 2);
				}
				if(rcv[0]=='0')break;
				cnt--;
			}
			else cnt=5;
		}
	}
	HAL_UART_Receive_IT(huart_TB, tb_msg, 6);
	if(SW2)	HAL_Delay(2000);
	HAL_UART_Transmit(huart_debugg, (uint8_t*)"START!\n\r",8, 3);
}
void Uart_Receive_Thunderboard_ISR(UART_HandleTypeDef *huart_TB, UART_HandleTypeDef *huart_debugg)
{
	static uint8_t sp[]={0};//slip protection
	static uint8_t cnt=0;
	if(tb_msg[0]>='A' && tb_msg[0]<='Z' && tb_msg[5]>='0' && tb_msg[5]<='9')
	{
		piratePos[0]=tb_msg[0];	piratePos[1]=tb_msg[1];	piratePos[2]=tb_msg[2];
		piratePos[3]=100*(tb_msg[3]-0x30) + 10*(tb_msg[4]-0x30) + (tb_msg[5]-0x30);
		thunderboardFlag=1;
		HAL_UART_Receive_IT(huart_TB, tb_msg, 6);
#ifdef TB_DEBUGG
		HAL_UART_Transmit(huart_debugg, tb_msg, 6, 2);
		HAL_UART_Transmit(huart_debugg, (uint8_t*)"\n\r", 2, 2);
#endif
		return;
	}
	//SLIP PROTECTION
	if(sp[0]>='0' && sp[0]<='9')cnt++;
	else cnt=0;

	if(cnt<3) //3 darab ASCI számnak össze kell gyűlnie egymás után
	{
		HAL_UART_Receive_IT(huart_TB, sp, 1);//amig ez nincs meg addig cask egyesével olvasunk
	}
	else//ha megvan megint 6-ossával olvasunk
	{
		sp[0]=0;
		cnt=0;
		HAL_UART_Receive_IT(huart_TB, tb_msg, 6);
		thunderboardFlag=1;
	}

}


void Lane_Change_Init(void)
{
	N('A').worth = N('B').worth = N('C').worth = N('Y').worth = 0;
	N('D').worth = N('E').worth=1;
	N('F').worth = N('G').worth = N('W').worth = N('X').worth = 2;
	N('H').worth = N('I').worth = N('J').worth = N('M').worth = N('P').worth = N('T').worth = N('V').worth = N('U').worth = 4;
	N('L').worth = N('K').worth = N('O').worth = N('R').worth = N('S').worth = 8;
	N('Q').worth = 16;
	N('N').worth = 32;

}
