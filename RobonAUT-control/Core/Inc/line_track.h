/*
 * line_track.h
 *
 *  Created on: 2022. nov. 29.
 *      Author: Zsombesz
 */

#ifndef INC_LINE_TRACK_H_
#define INC_LINE_TRACK_H_


#include "stm32f4xx_hal.h"

#define CMD_READ (255)
#define START_BYTE (23)
#define STOP_BYTE (18)

#define CCR_MAX (800)
#define CCR_MIN (460)
#define D (85)
#define L_SENSOR (250)
#define L (272)

#define SC_MODE (0)
#define FAST_MODE (1)

#define STOP (0)
#define GO_SLOW (1)
#define GO_FAST (2)
#define LINE_CNT (rxBuf[1])

/**************************************MÉRÉSEK ALAPJÁN KONFIGURÁLANDÓ************************************************************/
#define SERVO_CCR_MIDDLE (593) //Ilyen kitöltés mellett 0 a kormányszög
#define SERVO_M (30) //a servo RC mappelés meredeksége bal oldalt
#define BREAK_TIME_MS (300) //leglább ennyi MS - ig kell a kocsinak a tripla vonalat érzékelni, hogy lelassítson a kanyar előtt
#define DRAG_TIME_MS (600) //a gyorsítós szaggatott vonalnál ennyi időt adunk 5 váltásra
//150-as kitöltéshez tartozó szabolyzóparaméterek
#define K_DELTA_150 (-0.0007387)
#define K_P_150 (-0.6153)
//200-as kitöltéshez tartozó szabolyzóparaméterek
#define K_DELTA_200 (-0.00068)
#define K_P_200 (-0.631)
//250-as kitöltéshez tartozó szabolyzóparaméterek
#define K_DELTA_250 (-0.00068) //nem pontos szám ezt mérni kell!!!!!
#define K_P_250 (-0.6153) //nem pontos szám ezt mérni kell!!!!!
//600-as kitöltéshez tartozó szabolyzóparaméterek
#define K_DELTA_600 (-0.00068) //nem pontos szám ezt mérni kell!!!!!
#define K_P_600 (-0.6153) //nem pontos szám ezt mérni kell!!!!!

//SC követés paraméterei
#define DIST_STOP_MM (300) //ha ennyi mm nél közelebb vagyun a SC-hoz akkor fékezés
#define DIST_SLOW_MM (500) //ha gyorsan megyünk és ennyi mm-nél közelebb kerülünk a SC-hoz akkor lasítsunk
#define DIST_FAST_MM (800) //ha lassan megyünk és ennyi mm nél távolabb kerülünk a SC-tól akkor gyorsítsunk

/**************************************MÉRÉSEK ALAPJÁN KONFIGURÁLANDÓ************************************************************/


extern uint8_t txBuf[];
extern uint8_t rxBuf[];

void Line_Track_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug, uint32_t tick, uint32_t period);
uint8_t G0_Read(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug);


#endif /* INC_LINE_TRACK_H_ */
