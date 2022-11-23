/*
 * tof.h
 *
 *  Created on: 2022. nov. 22.
 *      Author: Zsombesz
 */

#ifndef INC_TOF_H_
#define INC_TOF_H_

#include "stm32g0xx_hal.h"
#include "main.h"
#include "vl53l1_api.h"


#define XSHUT1(x) (HAL_GPIO_WritePin(XSHUT_1_GPIO_Port, XSHUT_1_Pin, x))

#define EXPANDER_1_ADDR 0x84 // 0x42 << 1
#define EXPANDER_2_ADDR 0x86 // 0x43 << 1

extern uint8_t tofData[];

void Tof_Init(I2C_HandleTypeDef *hi2c, uint16_t period);

void Tof_Task(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period);



#endif /* INC_TOF_H_ */
