/*
 * line_track.h
 *
 *  Created on: 2022. nov. 29.
 *      Author: Zsombesz
 */

#ifndef INC_LINE_TRACK_H_
#define INC_LINE_TRACK_H_


#include "stm32f4xx_hal.h"

#define CCR_FRONT_MAX (900)
#define CCR_FRONT_MIN (480)
#define CCR_REAR_MAX (900)
#define CCR_REAR_MIN (560)

//#define D (85.0)
#define D_FRONT (105.0)
#define D_REAR (78.0)
//#define L_SENSOR (253.0)
#define L_SENSOR (452.0)
#define L (278.0)

#define SC_MODE (0)
#define FREERUN_MODE (1)

#define LINE_CNT (rxBuf[1])
#define LINE1 (rxBuf[2])
#define LINE2 (rxBuf[3])
#define LINE3 (rxBuf[4])
#define LINE4 (rxBuf[5])
/**************************************MÉRÉSEK ALAPJÁN KONFIGURÁLANDÓ************************************************************/
#define SERVO_FRONT_CCR_MIDDLE (684) //Ilyen kitöltés mellett 0 a kormányszög
#define SERVO_REAR_CCR_MIDDLE (680) //Ilyen kitöltés mellett 0 a kormányszög
#define BREAK_TIME_MS (140) //leglább ennyi MS - ig kell a kocsinak a tripla vonalat érzékelni, hogy lelassítson a kanyar előtt
//200-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_200 (-0.0006746688)
#define K_DELTA_200 (-0.6324900864)


//Megkívánt póluspár, ha kszi=0.8, t5=1.3s
#define S1ADDS2_SLOW (-6)
#define S1MULTS2_SLOW (9.2304)
//Megkívánt póluspár, ha kszi=0.8, t5=0.79s
#define S1ADDS2_FAST (-7.6000)
#define S1MULTS2_FAST (14.6704)

#define SERVO_M (1400) //a servo RC mappelés meredeksége bal oldalt
//#define K_D (-0.05)
#define K_D (-0.025)//23.ai teszten kimért

/**************************************MÉRÉSEK ALAPJÁN KONFIGURÁLANDÓ************************************************************/

void Line_Track_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg, uint32_t tick, uint32_t period);
uint8_t G0_Read_Skill(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg, uint8_t command);
uint8_t G0_Read_Fast(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg);
float Fast_Mode(UART_HandleTypeDef *huart_debugg, uint32_t t);

#endif /* INC_LINE_TRACK_H_ */
