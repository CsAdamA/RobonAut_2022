/*
 * config.h
 *
 *  Created on: Nov 13, 2022
 *      Author: Levi
 */

#ifndef INC_CONFIGF4_H_
#define INC_CONFIGF4_H_

#include "stm32f4xx_hal.h"

#define START_BYTE (23)
#define STOP_BYTE (18)
#define CMD_READ_FAST (42)
#define CMD_READ_SKILL (57)
#define CMD_MODE_FAST (63)
#define CMD_MODE_SKILL (82)

#define FAST (63)
#define SKILL (82)

#define FLASH_ADDRESS_SECTOR7  (0x08060000) //Sector 7 legelső címe -> itt tároljuk, hogy milyen módban vagyunk

void F4_Basic_Init(UART_HandleTypeDef *huart_debugg,TIM_HandleTypeDef *htim_scheduler,TIM_HandleTypeDef *htim_motor,TIM_HandleTypeDef *htim_servo, TIM_HandleTypeDef *htim_encoder);
void HDI_Read_Task(TIM_HandleTypeDef *htim_servo,uint32_t tick, uint32_t period);
void Uart_Receive_From_PC_ISR(UART_HandleTypeDef *huart);

#endif /* INC_CONFIGF4_H_ */
