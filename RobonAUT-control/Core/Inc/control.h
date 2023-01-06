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

#define STEADY 0
#define QUAD_LINE_DETECTED 1
#define MAYBE_HORIZONTAL_NODE_1 2
#define MAYBE_HORIZONTAL_NODE_2 3
#define MAYBE_HORIZONTAL_NODE_3 4
#define HORIZONTAL_NODE_DETECTED 5
#define VERTICAL_NODE_DETECTED 6

#define TH(x) (x*1000/abs((int)v))
#define TH_MIN(x) (x*600/abs((int)v))
#define TH_MAX(x) (x*1400/abs((int)v))


extern uint8_t readytorace;
extern uint8_t pirate_pos[];
extern volatile uint8_t uartThunder[];
extern volatile uint8_t thunderboardFlag;


void Mode_Selector(UART_HandleTypeDef *huart_debugg, UART_HandleTypeDef *huart_stm);
void Detect_Node(UART_HandleTypeDef *huart_debugg, uint32_t t);
void Detect_Node2(UART_HandleTypeDef *huart_debugg, uint32_t t);
float Skill_Mode(UART_HandleTypeDef *huart_debugg);

void Monitoring_Task(UART_HandleTypeDef *huart_monitoring, int16_t sebesseg, uint8_t vonalszam, int32_t CCR, uint16_t tavolsag, uint32_t tick, uint32_t period);
void GetBoardValue(UART_HandleTypeDef *huart_TB,UART_HandleTypeDef *huart_DEBUGG, uint32_t tick, uint32_t period);
void Uart_Receive_Thunderboard_ISR(UART_HandleTypeDef *huart);


#endif /* INC_CONTROL_H_ */
