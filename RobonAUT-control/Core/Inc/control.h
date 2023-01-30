/*
 * control.h
 *
 *  Created on: Jan 1, 2023
 *      Author: Zsombesz
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>

//Detect Node állapotai
#define STEADY 0
#define QUAD_LINE_DETECTED 1
#define MAYBE_HORIZONTAL_NODE_1 2
#define MAYBE_HORIZONTAL_NODE_2 3
#define MAYBE_HORIZONTAL_NODE_3 4
#define HORIZONTAL_NODE_DETECTED 5
#define VERTICAL_NODE_DETECTED 6

//path állapotai
#define LEFT 0
#define MIDDLE 1
#define RIGHT 2

//orientation állapotai
#define FORWARD 0
#define REVERSE 1

//control task state állapotok
#define NEIGHBOUR1	0
#define NEIGHBOUR2	1
#define NEIGHBOUR3	2
#define NEIGHBOUR4	3
#define EVALUATE 	4

//control task statikus tömbök indexelése
#define MY 0
#define NEXT 1
#define TMP 0

#define DIST_AVG 300

//torkolatkompenzáció állapotai és konstansai
#define ESTUARY_TH 75 //milyen thrashold érték felett lépünk be a torkolatkompenzálásba
#define ESTUARY_EXIT 45 //milyen byteértékkülönbségnél lépjünk ki a torkolatkompenzlásból
#define ESTURAY_TIMEOUT 800//hány ms után lépjünk ki a torkolatkompenzlásból (legkésőbb)
#define ESTUARY_MODE_INIT 0
#define ESTUARY_MODE_OFF 1
#define ESTUARY_MODE_ON 2

//a sebességünk(mm/s) segítségével számít az útból(mm) időt (ms)
#define TH(x) (x*1000/abs((int)v))
#define TH_MIN(x) (x*700/abs((int)v))
#define TH_MAX(x) (x*1300/abs((int)v))

typedef struct Node {
    uint8_t id; //hanyas számú node
    int32_t worth;  //ha van kapu a csomópontban, akkor hány pontot ér (ha nincs akkor 0)
    uint32_t type; //vertikális (1) vagy horizontális(2) node (további kommentek a horizontálishoz)
    uint8_t neighbours[4]; //6 szomszéd lehetséges (fölbalra, fölegyenesen, följobbra, lebalra, legegyenesen, lejobbra)
    //3 szomszéd lehet az egyik irányba (balra, egyenesen, jobbra)
    //-> tehát az előbbi tömbnek vagy csak az első 3 vagy csak az utolsó 3 elemére lesz szükség egyszerre menet közben
    //lehet hogy kevesebb szomszéd van (ilyenkor a megfelelő szomszéd(ok) 255-ök pl)
    uint8_t directions[4]; //ha a neighbours i. elemét megközelítjük akkor föl-(1) vagy lefelé(2) lesz a kocsi orientációja ott
    uint16_t distance[4];
} node;


extern volatile uint8_t uartThunder[];
extern volatile uint8_t thunderboardFlag;
extern node Nodes[];

//a pályacsomópontok egyszerű kezeléséhez és inicializáláshoz
#define N(x) (Nodes[x-65])
#define VALUE(a,b,c,d,e) a[0]=b;a[1]=c;a[2]=d;a[3]=e;

void Create_Nodes(UART_HandleTypeDef *huart_debugg);
void Mode_Selector(UART_HandleTypeDef *huart_debugg, UART_HandleTypeDef *huart_stm);
void Control_Task(UART_HandleTypeDef *huart_debugg,uint32_t tick, uint32_t period);
void Lane_Change_Init(void);

void Wait_For_Start_Sigal(UART_HandleTypeDef *huart_TB, UART_HandleTypeDef *huart_debugg);
void Monitoring_Task(UART_HandleTypeDef *huart_monitoring, int16_t sebesseg, uint8_t vonalszam, int32_t CCR, uint16_t tavolsag, uint32_t tick, uint32_t period);
void Uart_Receive_Thunderboard_ISR(UART_HandleTypeDef *huart_TB, UART_HandleTypeDef *huart_debugg);



#endif /* INC_CONTROL_H_ */
