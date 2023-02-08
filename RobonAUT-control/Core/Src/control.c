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
uint8_t v_control;

volatile uint8_t thunderboardFlag=0;
uint8_t tb_msg[6];
uint8_t piratePos[4];

node Nodes[22];

void Create_Nodes(UART_HandleTypeDef *huart_debugg)
{
	int i;
	orientation=FORWARD;
	collectedPoints=0;
	laneChange=0;
	path=LEFT;

	//Ct2			//Ct
	//nodeDetected=0;
	nodeDetected=1;
	//v_control=STOP;
	v_control=NORMAL_VEL;

	if(mode!=SKILL)return;

	for(i=0;i<22;i++)
	{
		Nodes[i].id=65+i;
		Nodes[i].worth=0;
		Nodes[i].type=0;
		VALUE_2(Nodes[i].neighbours,0,0,0,0,0,0);
		VALUE_2(Nodes[i].directions,0,0,0,0,0,0);
		VALUE_2(Nodes[i].distance,0,0,0,0,0,0);
	}
	//A node
	N('A').worth=2;
	N('A').type=1;
	VALUE_2(N('A').neighbours,'C',0,0,'B',0,'D');
	VALUE_2(N('A').directions,1,0,0,2,0,1);
	VALUE_2(N('A').distance,199,0,0,241,0,198);
	N('A').middle=NEIGHBOUR6;

	//B node
	N('B').worth=2;
	N('B').type=1;
	VALUE_2(N('B').neighbours,'D',0,'A',0,0,'E');
	VALUE_2(N('B').directions,1,0,1,0,0,1);
	VALUE_2(N('B').distance,199,0,241,0,0,198);
	N('B').middle=NEIGHBOUR1;

	//C node
	N('C').worth=2;
	N('C').type=2;
	VALUE_2(N('C').neighbours,'F',0,0,0,0,'A');
	VALUE_2(N('C').directions,2,0,0,0,0,2);
	VALUE_2(N('C').distance,198,0,0,0,0,199);
	N('C').middle=NEIGHBOUR1;

	//D node
	N('D').worth=2;
	N('D').type=2;
	VALUE_2(N('D').neighbours,'G','I','F','A',0,'B');
	VALUE_2(N('D').directions,2,1,1,1,0,2);
	VALUE_2(N('D').distance,198,241,197,198,0,199);
	N('D').middle=NEIGHBOUR2;

	//E node
	N('E').worth=2;
	N('E').type=2;
	VALUE_2(N('E').neighbours,'J',0,'G','B',0,0);
	VALUE_2(N('E').directions,1,0,1,1,0,0);
	VALUE_2(N('E').distance,241,0,197,198,0,0);
	N('E').middle=NEIGHBOUR3;

	//F node
	N('F').worth=2;
	N('F').type=1;
	VALUE_2(N('F').neighbours,'H',0,'C','D','G','I');
	VALUE_2(N('F').directions,1,0,2,2,2,1);
	VALUE_2(N('F').distance,200,0,198,197,241,198);
	N('F').middle=NEIGHBOUR6;

	//G node
	N('G').worth=2;
	N('G').type=1;
	VALUE_2(N('G').neighbours,'I','F','D','E',0,'J');
	VALUE_2(N('G').directions,1,1,2,2,0,1);
	VALUE_2(N('G').distance,199,241,197,197,0,198);
	N('G').middle=NEIGHBOUR1;

	//H node
	N('H').worth=2;
	N('H').type=2;
	VALUE_2(N('H').neighbours,'K',0,'M',0,0,'F');
	VALUE_2(N('H').directions,2,0,1,0,0,2);
	VALUE_2(N('H').distance,198,0,241,0,0,200);
	N('H').middle=NEIGHBOUR1;

	//I node
	N('I').worth=2;
	N('I').type=2;
	VALUE_2(N('I').neighbours,'L','N','K','F','D','G');
	VALUE_2(N('I').directions,2,1,1,1,2,2);
	VALUE_2(N('I').distance,198,241,197,198,241,199);
	N('I').middle=NEIGHBOUR6;

	//J node
	N('J').worth=2;
	N('J').type=2;
	VALUE_2(N('J').neighbours,0,0,'L','G',0,'E');
	VALUE_2(N('J').directions,0,0,1,1,0,2);
	VALUE_2(N('J').distance,0,0,197,198,0,241);
	N('J').middle=NEIGHBOUR4;

	//K node
	N('K').worth=2;
	N('K').type=1;
	VALUE_2(N('K').neighbours,'M',0,'H','I','L','N');
	VALUE_2(N('K').directions,1,0,2,2,2,1);
	VALUE_2(N('K').distance,200,0,198,197,241,198);
	N('K').middle=NEIGHBOUR4;

	//L node
	N('L').worth=2;
	N('L').type=2;
	VALUE_2(N('L').neighbours,'N','K','I','J',0,'O');
	VALUE_2(N('L').directions,1,1,2,2,0,1);
	VALUE_2(N('L').distance,199,241,198,197,0,198);
	N('L').middle=NEIGHBOUR3;

	//M node
	N('M').worth=2;
	N('M').type=2;
	VALUE_2(N('M').neighbours,'P','U',0,'H',0,'K');
	VALUE_2(N('M').directions,2,2,0,2,0,2);
	VALUE_2(N('M').distance,198,319-25,0,241,0,200);
	N('M').middle=NEIGHBOUR6;

	//N node
	N('N').worth=2;
	N('N').type=2;
	VALUE_2(N('N').neighbours,'Q','T','P','K','I','L');
	VALUE_2(N('N').directions,2,1,1,1,2,2);
	VALUE_2(N('N').distance,198,120-25,197,198,241,199);
	N('N').middle=NEIGHBOUR4;

	//O node
	N('O').worth=2;
	N('O').type=2;
	VALUE_2(N('O').neighbours,0,'V','Q','L',0,0);
	VALUE_2(N('O').directions,0,1,1,1,0,0);
	VALUE_2(N('O').distance,0,318-25,197,198,0,0);
	N('O').middle=NEIGHBOUR3;

	//P node
	N('P').worth=2;
	N('P').type=1;
	VALUE_2(N('P').neighbours,0,0,'M','N',0,'Q');
	VALUE_2(N('P').directions,0,0,2,2,0,2);
	VALUE_2(N('P').distance,0,0,198,197,0,241);
	N('P').middle=NEIGHBOUR4;

	//Q node
	N('Q').worth=2;
	N('Q').type=1;
	VALUE_2(N('Q').neighbours,'P',0,'N','O',0,0);
	VALUE_2(N('Q').directions,1,0,2,2,0,0);
	VALUE_2(N('Q').distance,241,0,198,197,0,0);
	N('Q').middle=NEIGHBOUR3;

	//R node
	N('R').worth=0;
	N('R').type=3;
	VALUE_2(N('R').neighbours,0,0,0,'M',0,'P');
	VALUE_2(N('R').directions,0,0,0,2,0,2);
	VALUE_2(N('R').distance,0,0,0,197,0,241);
	N('R').middle=NEIGHBOUR6;

	//S node
	N('S').worth=0;
	N('S').type=3;
	VALUE_2(N('S').neighbours,'Q',0,'O',0,0,0);
	VALUE_2(N('S').directions,1,0,2,0,0,0);
	VALUE_2(N('S').distance,241,0,198,0,0,0);
	N('S').middle=NEIGHBOUR1;

	//T node
	N('T').worth=0;
	N('T').type=4;
	VALUE_2(N('T').neighbours,'V',0,'U','N',0,0);
	VALUE_2(N('T').directions,2,0,1,2,0,0);
	VALUE_2(N('T').distance,199,0,198,120-25,0,0);
	N('T').middle=NEIGHBOUR4;

	//U node
	N('U').worth=0;
	N('U').type=3;
	VALUE_2(N('U').neighbours,0,0,'M','T',0,'V');
	VALUE_2(N('U').directions,0,0,2,2,0,2);
	VALUE_2(N('U').distance,0,0,319-25,198,0,241);
	N('U').middle=NEIGHBOUR3;

	//V node
	N('V').worth=0;
	N('V').type=3;
	VALUE_2(N('V').neighbours,'U',0,'T','O',0,0);
	VALUE_2(N('V').directions,1,0,2,2,0,0);
	VALUE_2(N('V').distance,241,0,199,318-25,0,0);
	N('V').middle=NEIGHBOUR3;

	//Nodeértékek backup mentésből való visszatöltése
	if(SW2)//ha a kacsapoló2 a megfelelő állapotban van (világít a piros LED)
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
			for(i=0;i<22;i++)
			{
				HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_NODEWORTH+i, Nodes[i].worth);
			}
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_BYTE, FLASH_ADDRESS_NODEWORTH+22, collectedPoints);
			HAL_Delay(50);
			HAL_FLASH_Lock();
			return; //ha nem akkor használjuk a default értékeket
		}
		for(i=0;i<22;i++)
		{
			Nodes[i].worth=*(__IO uint8_t *) (FLASH_ADDRESS_NODEWORTH+i); //ha igen akkor töltsük be a backup mentést
		}
		collectedPoints=*(__IO uint8_t *) (FLASH_ADDRESS_NODEWORTH+22);
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
	static uint8_t pos[2]	=	{'S','Q'}; 				//my, next
	static uint8_t dir[2]	=	{1,1}; 					//my, next
	static uint8_t bestNb[2]=	{NEIGHBOUR1,NEIGHBOUR1};//tmp, next
	static uint8_t nextOri	=	FORWARD;
	static uint8_t nextPath	=	LEFT;

	//static uint32_t last_tb_msg=0;
	static uint32_t t_stamp=0;
	static uint32_t tick_prev=0;
	static float s=0;
	static uint32_t sMAX=351;
	static float fitness[4]={0,0,0,0};
	static float bestFitness=-150;
	static uint8_t piratePos_prev[]={0,0,0,0};

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
		s += (float)(tick-tick_prev)*fabs(v)/10000;
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

		if(collectedPoints >= 34 && !laneChange) //átváltás lane change módba
		{
			laneChange=1; //flag állítás
			Lane_Change_Init(); //a sávváltóhely felé nőnek a rewardok
			LED_Y(1); //sárga led világít
		}

		if(laneChange==1 && pos[MY]=='V' && pos[NEXT]=='U')//ha a tett színhelyén vagyunk
		{
			laneChange=2;
		}

		static char str[15]; //kiiratás
		sprintf(str,"d,d,%2d\n\r",(int)collectedPoints);
		str[0]=pos[MY];
		str[2]=pos[NEXT];
		HAL_UART_Transmit(huart_debugg, (uint8_t*)str, strlen(str), 2);

		if(Cross_Collision(pos[MY], pos[NEXT]))
		{
			control_task_state = WAIT;
			t_stamp=tick;
			LED_G(1);
		}
		else control_task_state = NEIGHBOUR1;

		pos[MY]=pos[NEXT];
		path=nextPath;
		dir[MY]=dir[NEXT];
		orientation=nextOri;//FORWARD

		nodeDetected=0;
		return;
	}

	if(thunderboardFlag)//ha új kalózpozíció jött a TB-től ujrakezdjük a számolást (első szomszéd vizsgálata jön)
	{
		if(piratePos_prev[1]!=piratePos[1] && !laneChange && piratePos[0] !='R')//a kalóz átment egy Node-on
		{
			if(N(piratePos[0]).worth > 0)
			{
				N(piratePos[0]).worth--; //az a node már kevesebbet ér
				collectedPoints ++;
			}
			else N(piratePos[0]).worth=0;
		}
		if(control_task_state!=WAIT)//wait állapotból nem tud mindekt kibillenteni az új kalózrobot pozíció
			control_task_state=NEIGHBOUR1;//kezdjük előrröl a fitneszérték számítást az 1. szomszédtól

		piratePos_prev[0]=piratePos[0];//előző kalozpozíció frissítése
		piratePos_prev[1]=piratePos[1];
		piratePos_prev[2]=piratePos[2];
		piratePos_prev[3]=piratePos[3];

		thunderboardFlag=0; //várjuk az újabb kalózrobot pozíciókat a thunderboardtól
	}
	if(!piratePos_prev[0])return;


	/******************LEGJOBB SZOMSZÉD KIVÁLASZTÁSA (első 4 állapot)******************/
	if(control_task_state < EVALUATE)//1.szomszéd/2.szomszéd/3.szomszéd/4.szomszéd
	{
		if(control_task_state==NEIGHBOUR1)
		{
			bestFitness=-200.0;//az előző számolás legjob fitneszértéke volt még benne
		}
		nID=N(pos[MY]).neighbours[control_task_state]; //a vizsgált 1.rendű szomszéd azonosítója
		if(nID) //ha létezik a szomszéd
		{
			fitness[control_task_state]=(float)N(nID).worth; //fitneszérték 1.rendű szomszéd alapján
			//kalozrobot hatása az 1.rendű szomszéd esetén
			if(piratePos[1]==nID) fitness[control_task_state] -= 80/*P*/;//ha a kalóz is ebbe az 1.rendű tart éppen akkor kerüljük el az ütközést
			else if(piratePos[2]==nID) fitness[control_task_state] -= 60/*P*/;//ha még csak tervezi, hogy odamegy, akkor is kerüljük a pontot
			else if(piratePos[0]==nID)//ha elhaygta azt  apontot akkor 3 szomszédot is büntetünk
			{
				fitness[control_task_state] -= 20;
				if(control_task_state<=NEIGHBOUR3)
				{
					fitness[NEIGHBOUR1]-=60;
					fitness[NEIGHBOUR2]-=60;
					fitness[NEIGHBOUR3]-=60;
				}
				else
				{
					fitness[NEIGHBOUR4]-=60;
					fitness[NEIGHBOUR5]-=60;
					fitness[NEIGHBOUR6]-=60;
				}
			}
			int i;
			uint8_t nnID;
			float nnFit;
			for(i=0;i<6;i++)//2.rednű szomszédok
			{
				nnFit=0.0;
				nnID=N(nID).neighbours[i]; //2.rednű szomszéd ID-ja
				if(nnID && nnID!=pos[MY])//ha létezik a 2.rendű szomszéd (és nem a myposition az)
				{
					nnFit += (float)N(nnID).worth;
					//if(piratePos[1]==nnID) nnFit -= 0.5/*P*/;//ha a kalóz is ebbe a pontba tart éppen akkor kerüljük el az ütközést
					//else if(piratePos[2]==nnID) fitness[control_task_state] -= 0.25/*P*/;//ha még csak tervezi, hogy odamegy, akkor se fogjuk tudni megelőnzi, mert mi 3 nodnyira vagyunk ő pedig csak 2
					//if(!lane_change)nnFit = nnFit * (float)DIST_AVG/N(nID).distance[i];//a 2.rendű szomszédhoz tartozó fitneszérték jobb ha az közelebb van az 1.rendű szomszédjához
					//ha a sávváltó szakaszt keressük akkor viszont nem díjazzuk a közelséget
					fitness[control_task_state] += nnFit/6/*P*/;
				}
			}
			//if(!lane_change) fitness[control_task_state] =fitness[control_task_state] * (float)DIST_AVG/N(pos[MY]).distance[control_task_state]; //minél közelebb van a szomszéd annál jobb
			//ha a sávváltó szakaszt keressük akkor viszont nem díjazzuk a közelséget
		}
		else fitness[control_task_state]=-150.0;//ha nem létezik a szomszéd erre tuti ne menjünk
		//uint16_t tmp= __HAL_TIM_GET_COUNTER(htim_rand)%2;
		if(fitness[control_task_state]>=bestFitness) //ha ez a fitness jobb mint az eddigi legjobb, akkor mostantól ez a legjobb
		{
			bestFitness=fitness[control_task_state];
			bestNb[TMP] = control_task_state;//ez az egy érték amivel a task első 4 (fitnesszámoló) álapota kommunikál a kiértékelő álapottal
		}
		control_task_state++;
		return; //ha csak valamelyik szomszédot vizsgáltuk még akkor eddig tartott ez a task futás, itt kilépünk
	}
	/**************************************************************************************/
	//ide csak akkor jutunk el ha control_task_state>NEIGHBOUR4

	/**********************KIÉRTÉKELÉS (control_task_state=EVALUATE ->5.állapot)**********************/
	else if(control_task_state==EVALUATE)
	{
		if(bestFitness==0 && fitness[N(pos[MY]).middle]==0) bestNb[TMP]=N(pos[MY]).middle;
		bestNb[NEXT]=bestNb[TMP];
		pos[NEXT]=N(pos[MY]).neighbours[bestNb[NEXT]];//a következő poziciónk a legjobb szomszéd lesz
		dir[NEXT]=N(pos[MY]).directions[bestNb[NEXT]];//már most tudjuk, mi lesz az irányunk, ha odaértünk

		//a kocsi az egyik node-ból átmegy egy másikba-> az irányok segítségével meghatározzu az új orientationt
		if(bestNb[NEXT] < NEIGHBOUR4) //ha balra/fel kell majd mennünk a nextPosition -höz
		{
			if(dir[MY]==2)//és eddig jobbra/fel mentünk,
				nextOri = !orientation;//akkor most orientációt kell váltanunk
			else nextOri = orientation; //különben nem kell
		}
		else //ha jobbra/le kell majd mennünk
		{
			if(dir[MY]==1)//és eddig balra/fel mentünk,
				nextOri =! orientation;//akkor most irányt kell váltanunk
			else nextOri = orientation; //különben nem kell
		}

		//path kiválasztás -> az orientációt mostmár tudjuk (tolatás/előre), már csak az ösvény kell kivákasztani, hogy a megfelelő szomszédhoz jussunk

		if(bestNb[NEXT]==NEIGHBOUR1 || bestNb[NEXT]==NEIGHBOUR4)nextPath=LEFT;
		else if(bestNb[NEXT]==NEIGHBOUR2 || bestNb[NEXT]==NEIGHBOUR5)nextPath=MIDDLE;
		else if(bestNb[NEXT]==NEIGHBOUR3 || bestNb[NEXT]==NEIGHBOUR6)nextPath=RIGHT;

		control_task_state=NEIGHBOUR1;
		return;
	}
	/**************************************************************************************/

	/**********ÜTKÖZÉSELKERÜLÉS VÁRAKOZÁSSAL (control_task_state=EVALUATE ->6.állapot)***********/
	else if(control_task_state==WAIT)
	{
		if(tick-t_stamp<4000)
		{
			v_control=STOP;
			control_task_state=WAIT;
		}
		else
		{
			LED_G(0);
			v_control=NORMAL_VEL;
			control_task_state=NEIGHBOUR1;
		}
	}
	/**************************************************************************************/
}

void Control_Task_2(UART_HandleTypeDef *huart_debugg,uint32_t tick, uint32_t period)
{
	static uint8_t pos[2]	=	{'S','Q'}; 				//my, next,
	static uint8_t bestNb[2]=	{NEIGHBOUR1,NEIGHBOUR1};//tmp, next

	static uint32_t tick_prev=0;
	static float s=0;									// Intergralashoz, megtett út
	static uint32_t sMAX=351;
	static uint8_t piratePos_prev[]={0,0,0,0};

	static uint8_t stage=0;
	static uint8_t route_index=0;
	static uint8_t control_task_2_state=0;

	static uint8_t route_1 [6]={'S','O','L', 0 , 0 , 0};
	static uint8_t route_2 [6]={'S','Q','N','I', 0 , 0};
	static uint8_t route_3 [6]={'S','Q','N','I','D', 0};
	static uint8_t route_4 [6]={'S','O','L','I', 0 , 0};
	static uint8_t route [6];
	static uint8_t selected_route=0;
	static uint8_t i=0;
	static char str[30];

	static uint32_t control_task_2_tick = 0;

	if(mode!=SKILL || laneChange)return;

	else if(control_task_2_tick>tick)return;
	control_task_2_tick=tick+period;

	//ha kapu nélküli nodeba tartunk éppen, akkor időzítéssel "detektáljuk" a nodot
/**/
	if(N(pos[MY]).type>2 && control_task_2_state==2)
	{
		s += (float)(tick-tick_prev)*fabs(v)/10000;
		if(s>sMAX)nodeDetected=1;
	}
	tick=tick_prev;

	if(thunderboardFlag==1)		//uj adat erkezik (minden 200ms)
	{

		piratePos_prev[0]=piratePos[0];//előző kalozpozíció frissítése P
		piratePos_prev[1]=piratePos[1];			//M
		piratePos_prev[2]=piratePos[2];			//K
		piratePos_prev[3]=piratePos[3];			//65

		if(piratePos_prev[1]!=piratePos[1] && !laneChange && piratePos[0] !='R')//a kalóz átment egy Node-on
		{
			if(N(piratePos[0]).worth > 0)
			{
				N(piratePos[0]).worth--; //az a node már kevesebbet ér
				collectedPoints ++;
			}
			else N(piratePos[0]).worth=0;
		}
		thunderboardFlag=0;
	}
	if(piratePos_prev[0]==0)return; //ha nem kaptunk még adatot a TB-tol return


	if(control_task_2_state==0)
	{

		if(nodeDetected)
		{
			LED_B_TOGGLE;
			//pontok nyugtázása
			collectedPoints +=N(pos[MY]).worth;//sávváltás módik vizsgáljuk az össezgyűjtött kapuk számát
			N(pos[MY]).worth=0;//ez a kapu már nem ér pontot
			if (route[route_index+2]==0)//még nincs kész az eléállás
			{
				//WAITING
				v_control=STOP;
				control_task_2_state=1;
				LED_Y(0);
			}

			pos[MY]=pos[NEXT]; //route 2 eetén a végén még hulyeség kerül ide
			pos[NEXT]=route[route_index+1];
			nodeDetected=0;
			route_index++;

			for(i=0; i<6;i++)
			{
				if (N(pos[MY]).neighbours[i]==pos[NEXT])//O=O-2
				{
					bestNb[NEXT]=i;
					break;// ha megvan akkor breakkel kilépünk a for ciklusból azonnal
				}
			}
			if(bestNb[NEXT]==NEIGHBOUR1 || bestNb[NEXT]==NEIGHBOUR4)path=LEFT;
			else if(bestNb[NEXT]==NEIGHBOUR2 || bestNb[NEXT]==NEIGHBOUR5)path=MIDDLE;
			else if(bestNb[NEXT]==NEIGHBOUR3 || bestNb[NEXT]==NEIGHBOUR6)path=RIGHT;

#ifdef ADIBUGG
			sprintf(str,"dd\n\r");
			str[0]='9';
			str[1]=pos[MY];//honnan
			HAL_UART_Transmit(huart_debugg, (uint8_t*)str, 4, 3);
#endif
		}
		else if(!stage)
		{
			LED_Y(1);
			switch(piratePos_prev[2])
			{
			case 'Q':
				memcpy(route, route_1,6);
				selected_route=1;
				break;
			case 'K':
				memcpy(route, route_2,6);
				selected_route=2;
				break;
			case 'H':
				memcpy(route, route_3,6);
				selected_route=3;
				break;
			case 'N':
				memcpy(route, route_4,6);
				selected_route=4;
				break;
			}

			pos[MY]=route[route_index];
			pos[NEXT]=route[route_index+1];
			route_index++;
			////////////////////////////////////////////////////////////
			for(i=0; i<6;i++)
			{
				if (N(pos[MY]).neighbours[i]==pos[NEXT])//O=O-2
				{
					bestNb[NEXT]=i;
					break;// ha megvan akkor breakkel kilépünk a for ciklusból azonnal
				}
			}
			if(bestNb[NEXT]==NEIGHBOUR1 || bestNb[NEXT]==NEIGHBOUR4)path=LEFT;
			else if(bestNb[NEXT]==NEIGHBOUR2 || bestNb[NEXT]==NEIGHBOUR5)path=MIDDLE;
			else if(bestNb[NEXT]==NEIGHBOUR3 || bestNb[NEXT]==NEIGHBOUR6)path=RIGHT;
			stage=1;
			v_control=NORMAL_VEL;
			//////////////////////////////////////////////////////////////
#ifdef ADIBUGG
			sprintf(str,"dd\n\r");
			str[0]=pos[MY];
			str[1]=pos[NEXT];//honnan
			HAL_UART_Transmit(huart_debugg, (uint8_t*)str, 4, 3);
#endif
		}
	}

	else if(control_task_2_state==1)
	{
		//O-ban várakoz
		if (selected_route==1 && piratePos_prev[1]=='O')
		{
			control_task_2_state=2;//pos[MY]=piratePos[2];//amíg a kalózrobot azt célba nem veszi
			pos[MY]=piratePos_prev[2];
		}
		else if (selected_route==2) //N-ben várakozunk
		{
			if(piratePos_prev[1]=='K' && piratePos_prev[2] != 'N')
			{
				pos[MY]=piratePos_prev[2];//menjünk oda ahova ő akar, ez akkor jo ha nem felénk jön, felülbíráljuk a pos[my]='I'-t
				control_task_2_state=2;
			}
			else if(piratePos_prev[1]=='N') //waiting
			{
				pos[MY]=piratePos_prev[2];
				orientation=REVERSE;
				control_task_2_state=2;
#ifdef CONTROL_TASK_2_DEBUGG
				sprintf(str,"Wait!\n\r");
				HAL_UART_Transmit(huart_debugg, (uint8_t*)str, strlen(str), 3);
#endif
			}
		}
		else if(selected_route==3)//I-ben várakozunk
		{
			if(piratePos_prev[1]=='F' && piratePos_prev[2] != 'I')
			{
				pos[MY]=piratePos_prev[2];//menjünk oda ahova ő akar, ez akkor jo ha nem felénk jön, felülbíráljuk a pos[my]='D'-t
				control_task_2_state=2;
			}
			else if(piratePos_prev[1]=='I')
			{
				//waiting
				pos[MY]=piratePos_prev[2];
				orientation=REVERSE;
				control_task_2_state=2;

#ifdef CONTROL_TASK_2_DEBUGG
				sprintf(str,"Wait!\n\r");
				HAL_UART_Transmit(huart_debugg, (uint8_t*)str, strlen(str), 3);
#endif
			}
		}

		else if (selected_route==4){//I-ben várakozunk

			if(piratePos_prev[1]=='N' && piratePos_prev[2]!='L')
			{
				pos[MY]=piratePos_prev[2];//menjünk oda ahova ő akar, ez akkor jo ha nem felénk jön, felülbíráljuk a pos[my]='I'-t
				control_task_2_state=2;
			}
			else if(piratePos_prev[1]=='L')//waiting
			{
				pos[MY]=piratePos_prev[2];
				orientation=REVERSE;
				control_task_2_state=2;
#ifdef CONTROL_TASK_2_DEBUGG
				sprintf(str,"Wait!\n\r");
				HAL_UART_Transmit(huart_debugg, (uint8_t*)str, strlen(str), 3);
#endif
			}
		}
		if (control_task_2_state==2)
		{
			for(i=0;i<6;i++)
			{
				if (N(piratePos_prev[1]).neighbours[i]==piratePos_prev[2])//O-hanyadik szomszedja L?
				{
					bestNb[NEXT]=i;
					break;
				}
			}
			if(bestNb[NEXT]==NEIGHBOUR1 || bestNb[NEXT]==NEIGHBOUR4)path=LEFT;
			else if(bestNb[NEXT]==NEIGHBOUR2 || bestNb[NEXT]==NEIGHBOUR5)path=MIDDLE;
			else if(bestNb[NEXT]==NEIGHBOUR3 || bestNb[NEXT]==NEIGHBOUR6)path=RIGHT;
			v_control=NORMAL_VEL;
#ifdef CONTROL_TASK_2_DEBUGG
			sprintf(str,"d\n\r");
			str[0]=pos[MY];//honnan
			HAL_UART_Transmit(huart_debugg, (uint8_t*)str, 3, 3);
#endif
#ifdef ADIBUGG
			sprintf(str,"dd\n\r");
			str[1]=pos[MY];//honnan
			str[0]=9;//honnan
			HAL_UART_Transmit(huart_debugg, (uint8_t*)str, 4, 3);
#endif
		}
	}

	if (control_task_2_state==2)//elotte mozgas
	{
		static uint8_t stopAfterNode=0;

		if(nodeDetected || stopAfterNode)
		{
			LED_B_TOGGLE;
			if(nodeDetected)//ha nem sávváltó üzemmódban vagyunk pontotszámolunk és felszedett kapukat nullázzuk
			{
				collectedPoints +=N(pos[MY]).worth;//sávváltás módik vizsgáljuk az össezgyűjtött kapuk számát
				N(pos[MY]).worth=0;//ez a kapu már nem ér pontot
			}

			if(collectedPoints >= 34 && !laneChange)
			{
				laneChange=1; //flag állítás
				Lane_Change_Init(); //a sávváltóhely felé nőnek a rewardok
				LED_Y(1); //sárga led világít
				return;
			}

			if(piratePos_prev[1]==pos[MY])//ha celja a mi pozink, tudjuk a kovi celt
			{
				pos[MY]=piratePos_prev[2];
				for(i=0; i<6;i++)
				{
					if (N(piratePos_prev[1]).neighbours[i]==piratePos_prev[2])//O-hanyadik szomszedja L?
					{
						bestNb[NEXT]=i;
						break;
					}
				}
				if(N(pos[MY]).type>2)//ha a kövi node-on nincs kapu
				{
					s=0;
					sMAX=N(pos[MY]).distance[bestNb[NEXT]]+25;
				}
				if(bestNb[NEXT]==NEIGHBOUR1 || bestNb[NEXT]==NEIGHBOUR4)path=LEFT;
				else if(bestNb[NEXT]==NEIGHBOUR2 || bestNb[NEXT]==NEIGHBOUR5)path=MIDDLE;
				else if(bestNb[NEXT]==NEIGHBOUR3 || bestNb[NEXT]==NEIGHBOUR6)path=RIGHT;
				v_control=NORMAL_VEL;
				stopAfterNode=0;
#ifdef ADIBUGG
			sprintf(str,"dd\n\r");
			str[1]=pos[MY];//honnan
			str[0]=9;//honnan
			HAL_UART_Transmit(huart_debugg, (uint8_t*)str, 4, 3);
#endif

#ifdef CONTROL_TASK_2_DEBUGG
			sprintf(str,"d\n\r");
			str[0]=pos[MY];//honnan
			HAL_UART_Transmit(huart_debugg, (uint8_t*)str, 3, 3);
#endif

			}
			else
			{
				v_control=STOP;//wait
				stopAfterNode=1;
			}
			nodeDetected=0;
		}
	}
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
	if(mode!=SKILL)return;
	while(1)
	{
		//Bluetooth-on érkezika  start jel
		HAL_UART_Receive(huart_TB, rcv, 1, 7000);
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

		//Kézi próbaindítás
		if(B2)
		{
			int i;
			for(i=0;i<10;i++)
			{
				LED_G_TOGGLE;
				Delay(200);
			}
			LED_G(0);
			break;//ha megnyomtuka 2-es gombot kiugrunk a while ciklusból
		}

	}
	HAL_UART_Receive_IT(huart_TB, tb_msg, 6);
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
	N('A').worth = N('B').worth = N('C').worth = N('E').worth = 0;
	N('F').worth = N('D').worth = N('G').worth = 1;
	N('I').worth = 2;
	N('H').worth = N('J').worth = N('K').worth = N('L').worth = N('Q').worth = N('P').worth =  4;
	N('N').worth = N('T').worth =8;
	N('O').worth = N('M').worth = 9;
	N('U').worth = 16;
	N('V').worth = 32;
}

uint8_t Cross_Collision(uint8_t myPos, uint8_t nextPos)
{
	/********************************NI, KL kereszteződés**********************/
	if((myPos=='N' && nextPos=='I') || (myPos=='I' && nextPos=='N'))
	{
		if((piratePos[0]=='K' && piratePos[1]=='L') || (piratePos[0]=='L' && piratePos[1]=='K')){ if(piratePos[3]<60) return 1;}
		else if((piratePos[1]=='K' && piratePos[2]=='L') || (piratePos[1]=='L' && piratePos[2]=='K')){ if(piratePos[3]>50) return 1;}
	}
	else if((myPos=='K' && nextPos=='L') || (myPos=='L' && nextPos=='K'))
	{
		if((piratePos[0]=='N' && piratePos[1]=='I') || (piratePos[0]=='I' && piratePos[1]=='N')){ if(piratePos[3]<60) return 1;}
		else if((piratePos[1]=='N' && piratePos[2]=='I') || (piratePos[1]=='I' && piratePos[2]=='N')){ if(piratePos[3]>50) return 1;}
	}
	/**************************************************************************/


	/********************************ID, FG kereszteződés**********************/
	else if((myPos=='I' && nextPos=='D') || (myPos=='D' && nextPos=='I'))
	{
		if((piratePos[0]=='F' && piratePos[1]=='G') || (piratePos[0]=='G' && piratePos[1]=='F')){ if(piratePos[3]<60) return 1;}
		else if((piratePos[1]=='F' && piratePos[2]=='G') || (piratePos[1]=='G' && piratePos[2]=='F')){ if(piratePos[3]>50) return 1;}
	}
	else if((myPos=='F' && nextPos=='G') || (myPos=='G' && nextPos=='F'))
	{
		if((piratePos[0]=='I' && piratePos[1]=='D') || (piratePos[0]=='D' && piratePos[1]=='I')){ if(piratePos[3]<60) return 1;}
		else if((piratePos[1]=='I' && piratePos[2]=='D') || (piratePos[1]=='D' && piratePos[2]=='I')){ if(piratePos[3]>50) return 1;}
	}
	/**************************************************************************/


	/********************************TN, PQ kereszteződés**********************/
	else if((myPos=='P' && nextPos=='Q') || (myPos=='Q' && nextPos=='P'))
	{
		if(piratePos[0]=='T'){ if(piratePos[3]<50) return 1; }
		else if(piratePos[1]=='T') return 1;
		else if(piratePos[2]=='T'){ if(piratePos[3]>60) return 1; };
	}
	else if(nextPos=='T')
	{
		if((piratePos[0]=='P' && piratePos[1]=='Q') || (piratePos[0]=='Q' && piratePos[1]=='P')){if(piratePos[3]<70)return 1;}
		else if((piratePos[1]=='P' && piratePos[2]=='Q') || (piratePos[1]=='Q' && piratePos[2]=='P')){if(piratePos[3]>50)return 1;}
	}
	/**************************************************************************/
	return 0;
}
