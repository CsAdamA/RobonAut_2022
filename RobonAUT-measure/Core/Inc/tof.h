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

#include "../../Drivers/VL53L1X/core/inc/vl53l1_api.h"
#include "../../Drivers/VL53L1X/core/inc/vl53l1_core.h"
#include "../../Drivers/VL53L0X/core/inc/vl53l0x_api.h"
//#include "../../Drivers/VL53L0X/Inc/VL53L0X.h"


#define XSHUT1(x) (HAL_GPIO_WritePin(XSHUT_1_GPIO_Port, XSHUT_1_Pin, x))
#define XSHUT3(x) (HAL_GPIO_WritePin(XSHUT_3_GPIO_Port, XSHUT_3_Pin, x))
#define XSHUT4(x) (HAL_GPIO_WritePin(XSHUT_4_GPIO_Port, XSHUT_4_Pin, x))

#define EXPANDER_1_ADDR 0x84 // 0x42 << 1
#define EXPANDER_2_ADDR 0x86 // 0x43 << 1

#define ADDR_FRONT 	0x52
#define ADDR_LEFT 	0xBE
#define ADDR_RIGHT	0x6A


#define CATCH_UP_TH (500)


void Tof_Init(I2C_HandleTypeDef *hi2c_front,I2C_HandleTypeDef *hi2c_sides, uint16_t period);

void Tof_Task(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period);



#endif /* INC_TOF_H_ */
