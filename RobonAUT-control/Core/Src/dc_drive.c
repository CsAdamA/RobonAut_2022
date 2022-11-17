/*
 * dc_drive.c
 *
 *  Created on: Nov 14, 2022
 *      Author: Levi
 */


#include "dc_driver.h"
#include "main.h"
#include <string.h>
#include <math.h>

void Init_PWM(TIM_HandleTypeDef *htim, uint8_t DUTY,uint8_t *buf,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{

//Duty megadása, 0-100 a bejovo ertek->0-1000 koze kell vinni
  DUTY=DUTY*10;

//2 Referencia megadása
//Ezeket a loopba kéne változtatni folyamatosan, pwm-elinditas mashova kell majd
  TIM3->CCR1=DUTY;
  TIM3->CCR2=1000-DUTY;


  static uint32_t PWM_tick=0;
  if(PWM_tick>tick) return;
  PWM_tick= tick + period;

  memset(buf,0,32);
  sprintf(buf,"Kitoltesi tenyezo: %d \r\n",DUTY);
  HAL_UART_Transmit(huart, buf, strlen(buf), 100);

//PWM-ek elindítása
//  HAL_TIM_PWM_Start(*htim, TIM_CHANNEL_1);
//  HAL_TIM_PWM_Start(*htim, TIM_CHANNEL_2);
}
