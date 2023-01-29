/*
 * config.h
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */

#ifndef INC_CONFIGF4_H_
#define INC_CONFIGF4_H_

#include "stm32f4xx_hal.h"

#define CMD_READ_FAST (42)
#define CMD_READ_SKILL_FORWARD (57)
#define CMD_READ_SKILL_REVERSE (145)
#define CMD_MODE_FAST (63)
#define CMD_MODE_SKILL (82)
#define START_BYTE_FAST (23+CMD_READ_FAST) //65
#define START_BYTE_SKILL_FORWARD (23+CMD_READ_SKILL_FORWARD) //80
#define START_BYTE_SKILL_REVERSE (23+CMD_READ_SKILL_REVERSE) //168
#define STOP_BYTE (18)

#define FAST (63)
#define SKILL (82)

#define FLASH_ADDRESS_MODESELECTOR  (0x08060000) //Sector 7 legelső címe -> itt tároljuk, hogy milyen módban vagyunk
#define FLASH_ADDRESS_NODEWORTH  (0x08040000) //Sector 6 legelső címe -> itt tároljuk a félbehyagott ügyesség pálya kapuértékeit

void F4_Basic_Init(UART_HandleTypeDef *huart_debugg,TIM_HandleTypeDef *htim_scheduler,TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_servo1,TIM_HandleTypeDef *htim_servo2, TIM_HandleTypeDef *htim_encoder);
void HDI_Read_Task(TIM_HandleTypeDef *htim_servo,uint32_t tick, uint32_t period);
void Uart_Receive_From_PC_ISR(UART_HandleTypeDef *huart);

#endif /* INC_CONFIGF4_H_ */
