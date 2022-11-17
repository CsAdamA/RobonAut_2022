/*
 * dc_driver.h
 *
 *  Created on: Nov 14, 2022
 *      Author: Levi
 */

#ifndef INC_DC_DRIVER_H_
#define INC_DC_DRIVER_H_

#include "stm32f4xx_hal.h"

void Init_PWM(TIM_HandleTypeDef *htim, uint8_t DUTY);

#endif /* INC_DC_DRIVER_H_ */
