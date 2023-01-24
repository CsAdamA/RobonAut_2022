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
uint8_t ignore=0; //ha a node jelölés miatt látunk több vonalat, akkor azt ne kezeljük útelágazásnak (ignoráljuk)

////LEVI globals
uint8_t readytorace;
uint8_t pirate_pos[6];
volatile uint8_t uartThunder[6];
volatile uint8_t thunderboardFlag=0;
node N[24];

void Create_Nodes(void)
{
	int i;
	orientation=FORWARD;
	nodeDetected=0;

	for(i=0;i<24;i++)
	{
		N[i].id=65+i;
		N[i].worth=0;
		N[i].type=0;
		VALUE(N[i].neighbours,0,0,0,0);
		VALUE(N[i].directions,0,0,0,0);
		VALUE(N[i].distance,0,0,0,0);
	}
	//A node
	N[ID('A')].worth=0;
	N[ID('A')].type=1;
	VALUE(N[ID('A')].neighbours,0,0,0,'C');
	VALUE(N[ID('A')].directions,0,0,0,2);
	VALUE(N[ID('A')].distance,0,0,0,365);

	//B node
	N[ID('B')].worth=2;
	N[ID('B')].type=2;
	VALUE(N[ID('B')].neighbours,'D',0,'C',0);
	VALUE(N[ID('B')].directions,2,0,2,0);
	VALUE(N[ID('B')].distance,452,0,218,0);

	//C node
	N[ID('C')].worth=0;
	N[ID('C')].type=3;
	VALUE(N[ID('C')].neighbours,0,'B','E',0);
	VALUE(N[ID('C')].directions,0,1,2,0);
	VALUE(N[ID('C')].distance,0,218,160,0);

	//D node
	N[ID('D')].worth=2;
	N[ID('D')].type=1;
	VALUE(N[ID('D')].neighbours,'B',0,'F',0);
	VALUE(N[ID('D')].directions,2,0,2,0);
	VALUE(N[ID('D')].distance,452,0,316,0);

	//E node
	N[ID('E')].worth=0;
	N[ID('E')].type=3;
	VALUE(N[ID('E')].neighbours,'C',0,'F','G');
	VALUE(N[ID('E')].directions,1,0,2,2);
	VALUE(N[ID('E')].distance,160,0,428,385);

	//F node
	N[ID('F')].worth=2;
	N[ID('F')].type=1;
	VALUE(N[ID('F')].neighbours,'E','D','H','I');
	VALUE(N[ID('F')].directions,1,1,2,2);
	VALUE(N[ID('F')].distance,428,316,284,335);

	//G node
	N[ID('G')].worth=2;
	N[ID('G')].type=1;
	VALUE(N[ID('G')].neighbours,'E',0,'H','I');
	VALUE(N[ID('G')].directions,1,0,2,2);
	VALUE(N[ID('G')].distance,385,0,336,284);

	//H node
	N[ID('H')].worth=2;
	N[ID('H')].type=1;
	VALUE(N[ID('H')].neighbours,'G','F','K','J');
	VALUE(N[ID('H')].directions,1,1,2,2);
	VALUE(N[ID('H')].distance,336,284,407,230);

	//I node
	N[ID('I')].worth=0;
	N[ID('I')].type=3;
	VALUE(N[ID('I')].neighbours,'G','F',0,'L');
	VALUE(N[ID('I')].directions,1,1,0,2);
	VALUE(N[ID('I')].distance,284,335,0,418);

	//J node
	N[ID('J')].worth=0;
	N[ID('J')].type=3;
	VALUE(N[ID('J')].neighbours,'H',0,'K','L');
	VALUE(N[ID('J')].directions,1,0,2,2);
	VALUE(N[ID('J')].distance,230,0,204,229);

	//K node
	N[ID('K')].worth=2;
	N[ID('K')].type=1;
	VALUE(N[ID('K')].neighbours,'J','H','M','N');
	VALUE(N[ID('K')].directions,1,1,2,2);
	VALUE(N[ID('K')].distance,204,407,288,319);

	//L node
	N[ID('L')].worth=2;
	N[ID('L')].type=1;
	VALUE(N[ID('L')].neighbours,'I','J','M','N');
	VALUE(N[ID('L')].directions,1,1,2,2);
	VALUE(N[ID('L')].distance,418,229,329,258);

	//M node
	N[ID('M')].worth=2;
	N[ID('M')].type=1;
	VALUE(N[ID('M')].neighbours,'L','K','P','O');
	VALUE(N[ID('M')].directions,1,1,2,2);
	VALUE(N[ID('M')].distance,329,288,416,198);

	//N node
	N[ID('N')].worth=2;
	N[ID('N')].type=1;
	VALUE(N[ID('N')].neighbours,'L','K','O','Q');
	VALUE(N[ID('N')].directions,1,1,2,2);
	VALUE(N[ID('N')].distance,258,318,228,447);

	//O node
	N[ID('O')].worth=2;
	N[ID('O')].type=1;
	VALUE(N[ID('O')].neighbours,'N','M','P',0);
	VALUE(N[ID('O')].directions,1,1,2,0);
	VALUE(N[ID('O')].distance,228,198,248,0);

	//P node
	N[ID('P')].worth=2;
	N[ID('P')].type=1;
	VALUE(N[ID('P')].neighbours,'O','M','R','S');
	VALUE(N[ID('P')].directions,1,1,2,2);
	VALUE(N[ID('P')].distance,248,416,305,346);

	//Q node
	N[ID('Q')].worth=2;
	N[ID('Q')].type=1;
	VALUE(N[ID('Q')].neighbours,'N',0,'R','S');
	VALUE(N[ID('Q')].directions,1,0,2,2);
	VALUE(N[ID('Q')].distance,447,0,346,284);

	//R node
	N[ID('R')].worth=2;
	N[ID('R')].type=1;
	VALUE(N[ID('R')].neighbours,'Q','P','U','T');
	VALUE(N[ID('R')].directions,1,1,2,2);
	VALUE(N[ID('R')].distance,346,305,366,204);

	//S node
	N[ID('S')].worth=2;
	N[ID('S')].type=1;
	VALUE(N[ID('S')].neighbours,'Q','P','T','V');
	VALUE(N[ID('S')].directions,1,1,2,2);
	VALUE(N[ID('S')].distance,284,346,223,406);

	//T node
	N[ID('T')].worth=2;
	N[ID('T')].type=1;
	VALUE(N[ID('T')].neighbours,'S','R','U','V');
	VALUE(N[ID('T')].directions,1,1,2,2);
	VALUE(N[ID('T')].distance,223,204,192,233);

	//U node
	N[ID('U')].worth=2;
	N[ID('U')].type=1;
	VALUE(N[ID('U')].neighbours,'T','R','X',0);
	VALUE(N[ID('U')].directions,1,1,2,0);
	VALUE(N[ID('U')].distance,192,366,371,0);

	//V node
	N[ID('V')].worth=2;
	N[ID('V')].type=1;
	VALUE(N[ID('V')].neighbours,'S','T','W',0);
	VALUE(N[ID('V')].directions,1,1,2,0);
	VALUE(N[ID('V')].distance,406,233,149,0);

	//W node
	N[ID('W')].worth=0;
	N[ID('W')].type=3;
	VALUE(N[ID('W')].neighbours,'V',0,'X',0);
	VALUE(N[ID('W')].directions,1,0,1,0);
	VALUE(N[ID('W')].distance,149,0,189,0);

	//X node
	N[ID('X')].worth=2;
	N[ID('X')].type=2;
	VALUE(N[ID('X')].neighbours,'U',0,0,'W');
	VALUE(N[ID('X')].directions,1,0,0,1);
	VALUE(N[ID('X')].distance,371,0,0,189);
}


void Control_Task(uint32_t tick, uint32_t period)
{
	static uint8_t myPosition='A';
	static uint8_t nextPosition='C';
	static uint8_t myDirection=2; //előre megy a kocsi
	static uint8_t nextOrientation=FORWARD;
	static uint8_t nextDirection=2;
	static uint8_t nextPath=RIGHT;
	static uint32_t t_prev=0;
	static uint32_t node_detection_time=0;
	static float fitness[4]={0,0,0,0};
	uint8_t i=0;

	float bestFitness=0;
	static uint8_t bestPath=0;
	uint8_t nID=0;

	static uint32_t control_task_tick = 0;

	if(mode!=SKILL)return;
	if(control_task_tick>tick)return;
	control_task_tick=tick+period;

	//a koccsi szempontjából 2 irány van: előre(0)/hátra(1) (orientation->globális változó), ezt mostantól hívjuk orientációnak
	//a node szempontjából 2 irány van:jobbra(2)/balra(1) (myDirection) ->ezek a pálya k.r.-ben értelmezettek tehát a pályatérképen->mostantól irány
	//az adott irány további két ösvényre bontható (path)-> ez mostantól ösvény/pathirány
	//a kocsi az egyik node-ból átmegy egy másikba. Ez a művelet meghatároz két irányt
	//Az első érték, hogy merről hagytuk el az előző node-ot. A második, hogy milyen haladási iránnyal érkezünk a következő node-ba
	//a myPosition ahova éppen tartok, a nextPosition, ahova fogok menni, ha odaértem a myPositionbe
	//ha a myPos-ba odaérek és mennék tovább azonos orientációval, akkor ez az irány a myDirection lenne.
	//tehát a myPos azt tartalmazza, hogy ha orientáciováltoztazás nélkül mennék át a node-on akkor jobbra vagy balra hagynám-e el azt.
	//a next position ugyanez csak a következő (legoptimálisabb node-ra)
	//ha a nextPosition, be csak úgy tudok eljutni, hogy a myDirectionnel ellentétes irányba kéne indulnom,
	//akkor orientációt kell változtatni
	//a pathirány megahtárzása az orientation ismeretében már egyszerű

	//ha odaértünk a myPositionbe, akkor indulhat a mozgás a nextPosition felé
	if(nodeDetected)
	{
		LED_B_TOGGLE;
		nodeDetected=0;
		N[ID(myPosition)].worth=0;//ez a kapu már nem ér pontot
		if(N[ID(nextPosition)].type>2)//ha a kövi node-on nincs kapu
		{
			t_prev=tick;//mostantól mérjük az időt
			if(v>100|| v<-100)node_detection_time=1000*N[ID(myPosition)].distance[bestPath]/1100;//ennyi ms-nek kell eltelnie, amíg odaérünk
			else nodeDetected=1;
		}
		myPosition=nextPosition;
		path=nextPath;
		myDirection=nextDirection;
		orientation=nextOrientation;

	}

	//legjobb szomszéd kiválasztása
	bestFitness=0;
	for(i=0;i<4;i++)
	{
		if(N[ID(myPosition)].neighbours[i]>0) //ha létezik a szomszéd
		{
			nID=N[ID(myPosition)].neighbours[i]; //a vizsgált szomszéd azonosítója
			fitness[i]=(float)N[ID(nID)].worth/(N[ID(myPosition)].distance[i]); //a fitneszértéke
		}
		else fitness[i]=-100.0;//ha nem létezik a szomszéd erre tuti ne menjünk
		if(fitness[i]>bestFitness)
		{
			bestFitness=fitness[i];
			bestPath = i;
		}
	}
	//a következő poziciónk a legjobb szomszéd lesz
	nextPosition=N[ID(myPosition)].neighbours[bestPath];
	nextDirection=N[ID(myPosition)].directions[bestPath];//már most tudjuk, mi lesz az irányunk, ha odaértünk

	//a kocsi az egyik node-ból átmegy egy másikba-> az irányok segítségével meghatározzu az új orientationt
	if(bestPath<3) //ha balra/le kell majd mennünk a nextPosition -höz
	{
		if(nextDirection==2)//és eddig jobbra/fel mentünk,
			nextOrientation = !orientation;//akkor most orientációt kell váltanunk
		else nextOrientation=orientation; //különben nem kell
	}
	else //ha jobbra kell majd mennünk
	{
		if(myDirection==1)//és eddig jobbra/fel mentünk,
			nextOrientation = !orientation;//akkor most irányt kell váltanunk
		else nextOrientation=orientation; //különben nem kell
	}

	//path kiválasztás -> az orientációt mostmár tudjuk (tolatás/előre), már csak az ösvény kell kivákasztani, hogy a megfelelő szomszédhoz jussunk
	if(nextOrientation==FORWARD)
	{
		if(bestPath==1 || bestPath==3)nextPath=LEFT;
		else if(bestPath==2 || bestPath==4)nextPath=RIGHT;
	}
	else if(nextOrientation==REVERSE) //tolatásnál pont forditva vannak a pathirányok
	{
		if(bestPath==1 || bestPath==3)nextPath=RIGHT;
		else if(bestPath==2 || bestPath==4)nextPath=LEFT;
	}

	//ha kapu nélküli nodeba tartunk éppen, akkor időzítéssel "detektáljuk" a nodot
	if(N[ID(myPosition)].type>2 && (tick-t_prev)>node_detection_time)
	{
		nodeDetected=1;
	}

}


void Mode_Selector(UART_HandleTypeDef *huart_debugg, UART_HandleTypeDef *huart_stm)
{
	//Milyen módban kell működni?
	uint8_t buffer[40];
	uint32_t tmp=0;

	//HAL_FLASH_Unlock();
	tmp= *(__IO uint32_t *) FLASH_ADDRESS_SECTOR7; //FLASH-ből kiolvassuk, hogy milyen módban vagyunk
	//HAL_FLASH_Lock();
	mode = (uint8_t)tmp;

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
float Skill_Mode(UART_HandleTypeDef *huart_debugg, float kP, float kD, uint32_t t)
{
	static uint32_t t_prev=0;
	int byte=0;
	static int byte_prev=0;
	uint8_t delta_byte;
	float p=0;
	static float p_prev=0;
	static uint8_t estuary=ESTUARY_MODE_INIT;
	static float gamma;
	int i;
	static int tmp1,tmp2;
/*	uint8_t str[40];
	sprintf(str,"%d,  %d,  %d,  %d,  %d\n\r",rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5]);
	HAL_UART_Transmit(huart_debugg, str, strlen(str), 10);
*/

	if(LINE_CNT>3 || ignore)//ha éppen node-on vagyunk, akkor az átlagot követjük
	{
		byte=0;
		for(i=0;i<LINE_CNT;i++)
		{
			byte += rxBuf[i+2];
		}

		if(LINE_CNT) byte /= LINE_CNT;
	}
	else if(path==LEFT)
	{
		byte = LINE1; //az első vonalt kell követni
		delta_byte=abs((int)byte-byte_prev);
		/**/
		if((delta_byte>ESTUARY_TH && estuary!=ESTUARY_MODE_INIT)|| estuary==ESTUARY_MODE_ON) //torkolatkompenzálás
		{
			if(LINE_CNT>1)//torkolatkompenzálás csak akkor van ha legalább 2 vonalat látunk
			{
				if(estuary==ESTUARY_MODE_OFF)t_prev=t;//ha most kapcsoltuk be a torkolatkompenzálást, akkor mostantól mérjük az eltelt időt
				if((t-t_prev)>ESTURAY_TIMEOUT)//400ms után mindenképpen kilépünk a kompenzálásból
				{
					estuary=ESTUARY_MODE_OFF; //ha letelt a timeout kilépünk a kompenzálásból
					LED_G(0);
				}
				else //ha még nem telt le az timout idő
				{
					byte = rxBuf[1+LINE_CNT]; //ilyenkor az utolsó vonalat nézzük az első helyett
					estuary=ESTUARY_MODE_ON; //öntartás
					LED_G(1);
				}

			}
			else
			{
				estuary=ESTUARY_MODE_OFF; //ha nincs elég vonal kikapcsoljuk az öntartást (legalább2 vonal esetén beszélhetünk torkolatról)
				LED_G(0);
			}
		}
		else if(delta_byte<ESTUARY_EXIT && estuary==ESTUARY_MODE_ON) //ha már eléggé összeszűkült a torkolat, akkor nem kell kompenzálni
		{
			estuary=ESTUARY_MODE_OFF;
			LED_G(0);
		}

	}
	else if(path==RIGHT)
	{
		byte = rxBuf[1+LINE_CNT];//az utolsó vonalat kell követni
		delta_byte=abs((int)byte-byte_prev);
		/**/
		if((delta_byte>ESTUARY_TH && estuary!=ESTUARY_MODE_INIT)|| estuary==ESTUARY_MODE_ON) //torkolatkompenzálás
		{
			if(LINE_CNT>1)//torkolatkompenzálás csak akkor van ha legalább 2 vonalat látunk
			{
				if(estuary==ESTUARY_MODE_OFF)t_prev=t;//ha most kapcsoltuk be a torkolatkompenzálást, akkor mostantól mérjük az eltelt időt
				if((t-t_prev)>ESTURAY_TIMEOUT)//400ms után mindenképpen kilépünk a kompenzálásból
				{
					estuary=ESTUARY_MODE_OFF; //ha letelt a timeout kilépünk a kompenzálásból
					LED_G(0);
				}
				else //ha még nem telt le az idő
				{
					byte = rxBuf[2]; //ilyenkor az első vonalat nézzük az utolsó helyett
					estuary=ESTUARY_MODE_ON; //öntartás
					LED_G(1);
				}

			}
			else
			{
				estuary=ESTUARY_MODE_OFF; //ha nincs elég vonal kikapcsoljuk az öntartást
				LED_G(0);
			}
		}
		else if(delta_byte<ESTUARY_EXIT && estuary==ESTUARY_MODE_ON) //ha már eléggé összeszűkült a torkolat, akkor nem kell kompenzálni
		{
			estuary=ESTUARY_MODE_OFF;
			LED_G(0);
		}

	}

	else if(path==MIDDLE)
	{
		if(LINE_CNT==1)byte = LINE1;//ha csak 1 vonal van, akkor azt követjük
		else if(LINE_CNT==3)//ha 3 vonal van
		{
			byte = rxBuf[3];//a középsőt követjük
			//folyamatosan nézzük, hogy az 1. és 3.vonal milyen messze van a vonalszenor középontjától
			tmp1=abs((int)LINE1-123);
			tmp2=abs((int)LINE3-123);
		}
		else if(LINE_CNT==2)//ha 2 vonal van, az azt jelenti, hogy az elágazás már annyira szétgáazott, hogy csak 2-t látunk a 3 vonalból
		{
			if(tmp1<tmp2) byte = LINE1; //ha a jobboldali vonalat veszítettük el
			else byte = LINE2; //ha a baloldali vonalat veszítettük el
		}
	}
	if(estuary==ESTUARY_MODE_INIT)estuary=ESTUARY_MODE_OFF;
	//p = (float)byte * 204/248.0-102;
	p = (float)byte * 204/255.0-102;
	gamma = -kP * p  - kD*(p-p_prev);
	p_prev = p;
	byte_prev=byte;

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
		if(dt > TH_MIN(70) && dt < TH_MAX(70) && rxBuf[1]<4) detect_node_state = MAYBE_HORIZONTAL_NODE_1;
		else if(dt > TH_MIN(200) && dt < TH_MAX(200) && rxBuf[1]<4) detect_node_state=VERTICAL_NODE_DETECTED;
		else detect_node_state=STEADY;
		break;

	case MAYBE_HORIZONTAL_NODE_1: //többször futó állapot
		if(rxBuf[1]<4) dt = t - t_prev;//mennyi ideje van alattunk 4 vonal
		else
		{
			t_prev=t;
			if(dt > TH_MIN(60))detect_node_state=MAYBE_HORIZONTAL_NODE_2;
			else detect_node_state=STEADY;
		}
		break;

	case MAYBE_HORIZONTAL_NODE_2: //egyszer futó állapot
		if(dt > TH_MIN(60) && dt < TH_MAX(60) && rxBuf[1]>2) detect_node_state=MAYBE_HORIZONTAL_NODE_3;

		else detect_node_state=STEADY;
		break;

	case MAYBE_HORIZONTAL_NODE_3: //többször futó állapot
		if(rxBuf[1]>2) dt = t - t_prev;//mennyi ideje van alattunk 4 vonal
		else
		{
			t_prev=t;
			if(dt > TH_MIN(70))	detect_node_state=HORIZONTAL_NODE_DETECTED;
			else detect_node_state=STEADY;
		}
		break;

	case HORIZONTAL_NODE_DETECTED: //egyszer futó állapot
		if(dt > TH_MIN(70) && dt < TH_MAX(70) && rxBuf[1]<3)LED_B(1);//vízintes csomópont
		detect_node_state=STEADY;
		t_prev=t;
		break;

	case VERTICAL_NODE_DETECTED: //egyszer futó állapot
		if(rxBuf[1]<3)LED_G(1); //függőleges csomópont
		detect_node_state=STEADY;
		t_prev=t;
		break;

	}
}

void Detect_Node2(UART_HandleTypeDef *huart_debugg, uint32_t t)
{
	static uint8_t detect_node_state=0;
	static uint8_t val=0;
	static uint32_t dt=0;
	static uint32_t t_prev=0;
/*	static uint8_t str[30];


	sprintf(str,"%2d\r\n",rxBuf[1]);
	HAL_UART_Transmit(huart_debugg, str, 4, 10);
*/
	if (LINE_CNT<1 || LINE_CNT > 4)
	{
		t_prev=t;
		detect_node_state=STEADY;
		val=4;
	}
	switch(detect_node_state)
	{
	case STEADY: //többször futó állapot
		if(rxBuf[1]==4)
		{
			dt = t-t_prev;//mennyi ideje van alattunk 4 vonal
			if(dt > TH_MIN(70))
			{
				detect_node_state=QUAD_LINE_DETECTED;
				ignore=1;
			}
			val=0;
		}
		else
		{
			t_prev=t;
			ignore=0;
		}
		break;

	case QUAD_LINE_DETECTED:
		if(rxBuf[1]==2 && !val) val=1; //horizontal node lehetséges
		else if(rxBuf[1]==4 && val==1) val=2; //horizontal node tuti

		dt=t-t_prev;
		if(dt> TH(170) && rxBuf[1]==4 && !val)val=3;

		if(dt> TH_MAX(200))
		{
			if(val==3 && rxBuf[1]<4)LED_G(1);//vert node
			else if(val==2 && rxBuf[1]<4)
			{
				LED_B_TOGGLE; //horizont node
				if(path==0)path=2;
				else if(path==2)path=0;

			}
			detect_node_state=STEADY;
			val=0;
		}
		break;
	}

}

void Detect_Node3(UART_HandleTypeDef *huart_debugg, uint32_t t)
{
	static uint32_t dt=0;
	static uint32_t t_prev=0;

	dt=t-t_prev;
	if(LINE_CNT==4 && dt> 1500)
	{
		nodeDetected=1;
		/*
		if(path==0)path=2;
		else if(path==2)path=0;*/
		ignore=1;

		t_prev=t;
	}

	if(ignore && dt>200)
	{
		ignore=0;
	}
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
				HAL_UART_Receive_IT(huart_TB, (uint8_t*)uartThunder, 6);
				HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
			}else HAL_UART_Receive_IT(huart_TB, (uint8_t*)uartThunder, 1);
		}																					//extern volatile uint8_t flag,buffer

		else if (whichState==1){							// pozicio lekérés

			pirate_pos[0]=uartThunder[0];//honnan
			pirate_pos[1]=uartThunder[1];//hová
			pirate_pos[2]=uartThunder[2];//kovi cel
			pirate_pos[3]=uartThunder[3];//elso szj
			pirate_pos[4]=uartThunder[4];//masodik szj
			pirate_pos[5]=uartThunder[5];//harmadik szj

			HAL_UART_Receive_IT(huart_TB, (uint8_t*) uartThunder, 6);

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
