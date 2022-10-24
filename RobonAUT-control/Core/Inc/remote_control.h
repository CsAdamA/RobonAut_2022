/*
 * remote_control.h
 *
 *  Created on: 2022. okt. 24.
 *      Author: Zsombesz
 */

#ifndef INC_REMOTE_CONTROL_H_
#define INC_REMOTE_CONTROL_H_

#include "stm32f4xx_hal.h"

//makrók
#define READ_RC1 (HAL_GPIO_ReadPin(RC_PWM1_GPIO_Port, RC_PWM1_Pin))

//globális változók és flagek
extern float freqRC; //RC pwm frekvenciája
extern volatile int32_t tHighCnt;
extern volatile int32_t tLowCnt;
extern volatile uint8_t flagRC;

void Reset_Remote_Control_Flag(void);
uint8_t Read_Control_Flag(void);


void Remote_Control_Init(TIM_HandleTypeDef *htim, uint32_t channel);
void Remote_Control_Task(TIM_HandleTypeDef *htim, uint32_t channel,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period);
void Remote_Control_ISR(TIM_HandleTypeDef *htim, uint32_t channel);


#endif /* INC_REMOTE_CONTROL_H_ */
