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

#define CCR_MAX (880)
#define CCR_MIN (480)
#define D (85.0)
#define L_SENSOR (253.0)
#define L (272.0)

#define SC_MODE (0)
#define FAST_MODE (1)

#define STOP (0)
#define GO_SLOW (1)
#define GO_FAST (2)
#define LINE_CNT (rxBuf[1])

/**************************************MÉRÉSEK ALAPJÁN KONFIGURÁLANDÓ************************************************************/
#define SERVO_CCR_MIDDLE (685) //Ilyen kitöltés mellett 0 a kormányszög
#define SERVO_M (30.0) //a servo RC mappelés meredeksége bal oldalt
#define BREAK_TIME_MS (140) //leglább ennyi MS - ig kell a kocsinak a tripla vonalat érzékelni, hogy lelassítson a kanyar előtt
#define DRAG_TIME_MS (400) //a gyorsítós szaggatott vonalnál ennyi időt adunk 5 váltásra
//150-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_150 (-0.0007387)
#define K_DELTA_150 (-0.6153)
//200-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_200 (-0.0006746688)
#define K_DELTA_200 (-0.6324900864)
//250-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_250 (-0.00111166487647691)
#define K_DELTA_250 (-0.785627153598281)
//400-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_300 (-0.00110248470588235)
#define K_DELTA_300 (-0.78812416)
//400-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_400 (-0.0006219)
#define K_DELTA_400 (-0.6468)
//600-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_500 (-0.00061799)
#define K_DELTA_500 (-0.6479)
//600-as kitöltéshez tartozó szabolyzóparaméterek
//#define K_P_600 (-0.0006159)
//#define K_DELTA_600 (-0.6485)
//600-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_600 (-0.000616708824631579)
#define K_DELTA_600 (-0.648255199700211)
//650-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_650 (-0.0006153)
#define K_DELTA_650 (-0.6486)

//SC követés paraméterei
#define DIST_STOP_MM (300) //ha ennyi mm nél közelebb vagyun a SC-hoz akkor fékezés
#define DIST_SLOW_MM (500) //ha gyorsan megyünk és ennyi mm-nél közelebb kerülünk a SC-hoz akkor lasítsunk
#define DIST_FAST_MM (500) //ha lassan megyünk és ennyi mm nél távolabb kerülünk a SC-tól akkor gyorsítsunk

#define BREAK_PERIOD (80)
#define BREAK_DUTY (230)

#define M_150 (2006)
#define M_200 (1400)
#define M_250 (1400)
#define M_300 (1400)
#define M_600 (1400)

/**************************************MÉRÉSEK ALAPJÁN KONFIGURÁLANDÓ************************************************************/


extern uint8_t txBuf[];
extern uint8_t rxBuf[];

void Line_Track_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug, uint32_t tick, uint32_t period);
uint8_t G0_Read(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debug);

#endif /* INC_LINE_TRACK_H_ */
