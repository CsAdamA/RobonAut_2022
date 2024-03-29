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
#define CCR_REAR_MAX (840)//(880)
#define CCR_REAR_MIN (520)//(460)

#define MINMAX_FRONT(x) (x>CCR_FRONT_MAX ? CCR_FRONT_MAX : (x<CCR_FRONT_MIN ? CCR_FRONT_MIN : x))
#define MINMAX_REAR(x) 	(x>CCR_REAR_MAX ? CCR_REAR_MAX : (x<CCR_REAR_MIN ? CCR_REAR_MIN : x))

#define FRONT_CCR(ccr) (TIM2->CCR1 = MINMAX_FRONT(ccr))
#define REAR_CCR(ccr)  (TIM1->CCR4 = MINMAX_REAR(ccr))

//#define D (85.0)
#define D_FRONT (105.0)
#define D_REAR (78.0)
//#define L_SENSOR (253.0)
#define L_SENSOR (452.0)
#define L (278.0)

#define SC_MODE (0)
#define FREERUN_MODE (1)
#define OVERTAKE_MODE (2)

#define LINE_CNT (rxBuf[1])
#define LINE1 (rxBuf[2])
#define LINE2 (rxBuf[3])
#define LINE3 (rxBuf[4])
#define LINE4 (rxBuf[5])
/**************************************MÉRÉSEK ALAPJÁN KONFIGURÁLANDÓ************************************************************/
#define SERVO_FRONT_CCR_MIDDLE (692) //Ilyen kitöltés mellett 0 a kormányszög
#define SERVO_REAR_CCR_MIDDLE (680) //Ilyen kitöltés mellett 0 a kormányszög
#define BREAK_TIME_MS (80) //leglább ennyi MS - ig kell a kocsinak a tripla vonalat érzékelni, hogy lelassítson a kanyar előtt
//200-as kitöltéshez tartozó szabolyzóparaméterek
#define K_P_200 (-0.0006746688)
#define K_DELTA_200 (-0.6324900864)


#define S1ADDS2_UJ (-10.91)
#define S1MULTS2_UJ (29.97)

//#define S1ADDS2_UJ (-7.5)
//#define S1MULTS2_UJ (14.2929)
//Megkívánt póluspár, ha kszi=0.8, t5=1s
#define S1ADDS2_SLOW (-5)
#define S1MULTS2_SLOW (9.2304)
//Megkívánt póluspár, ha kszi=0.8, t5=0.79s
#define S1ADDS2_FAST (-9.23)
#define S1MULTS2_FAST (21.522)

#define SERVO_M_STRAIGHT 	(900) //a servo RC mappelés meredeksége bal oldalt
#define SERVO_M_CORNER 		(1000)
#define SERVO_M 			(1400)
#define SERVO_M_SC			(1300)

//#define K_D (-0.05)
//#define K_D (-0.025)//23.ai teszten kimért
#define K_D (-0.06)

/**************************************MÉRÉSEK ALAPJÁN KONFIGURÁLANDÓ************************************************************/

void Line_Track_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg, uint32_t tick, uint32_t period);
uint8_t G0_Read_Skill(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg, uint8_t command);
uint8_t G0_Read_Fast(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg);

float Fast_Mode(UART_HandleTypeDef *huart_debugg,uint8_t* state_pointer, uint32_t t);
float Skill_Mode(UART_HandleTypeDef *huart_debugg, float kP, float kD, uint32_t t);
void Detect_Node2(UART_HandleTypeDef *huart_debugg, uint32_t t);
void Detect_Node3(UART_HandleTypeDef *huart_debugg, uint32_t t);
void Detect_Node4(UART_HandleTypeDef *huart_debugg, uint32_t t);
void Detect_Node5(UART_HandleTypeDef *huart_debugg, uint32_t t);
uint8_t Lane_Changer(UART_HandleTypeDef *huart_debugg,uint32_t t);

#endif /* INC_LINE_TRACK_H_ */
