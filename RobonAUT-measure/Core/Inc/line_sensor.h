/*
 * line_sensor.h
 *
 *  Created on: Nov 5, 2022
 *      Author: Zsombesz
 */

#ifndef INC_LINE_SENSOR_H_
#define INC_LINE_SENSOR_H_

#include "stm32g0xx_hal.h"
#include "main.h"

//LED vezérlő reteszelő és kimenet engedélyező LED-jének állítása (OE PWM-ezhető itt, hogy ne égjen ki a szemünk) 		ELSŐ SZENZOR
#define LED_LE_F(x)	(HAL_GPIO_WritePin(LS_LED_LE_FRONT_GPIO_Port, LS_LED_LE_FRONT_Pin, x))
#define LED_OE_F(x)	(HAL_GPIO_WritePin(LS_LED_OE_FRONT_GPIO_Port, LS_LED_OE_FRONT_Pin, x))
#define LED_OE_F_L(x) (HAL_TIM_PWM_Start(x,TIM_CHANNEL_3))
#define LED_OE_F_H(x)	(HAL_TIM_PWM_Stop(x,TIM_CHANNEL_3))
//LED vezérlő reteszelő és kimenet engedélyező LED-jének állítása (OE PWM-ezhető itt, hogy ne égjen ki a szemünk)		HÁTSÓ SEZNZOR
#define LED_LE_B(x)	(HAL_GPIO_WritePin(LS_LED_LE_BACK_GPIO_Port, LS_LED_LE_BACK_Pin, x))
#define LED_OE_B(x)	(HAL_GPIO_WritePin(LS_LED_OE_BACK_GPIO_Port, LS_LED_OE_BACK_Pin, x))
#define LED_OE_B_L(x) (HAL_TIM_PWM_Start(x,TIM_CHANNEL_4))
#define LED_OE_B_H(x)	(HAL_TIM_PWM_Stop(x,TIM_CHANNEL_4))

//InfraLED vezérlő reteszelő és kimenet engedélyező LED-jének állítása													ELSŐ SZENZOR
#define INF_LE_F(x)	(HAL_GPIO_WritePin(LS_INF_LE_FRONT_GPIO_Port, LS_INF_LE_FRONT_Pin, x))
#define INF_OE_F(x)	(HAL_GPIO_WritePin(LS_INF_OE_FRONT_GPIO_Port, LS_INF_OE_FRONT_Pin, x))
//InfraLED vezérlő reteszelő és kimenet engedélyező LED-jének állítása													HÁTSÓ SEZNZOR
#define INF_LE_B(x)	(HAL_GPIO_WritePin(LS_INF_LE_BACK_GPIO_Port, LS_INF_LE_BACK_Pin, x))
#define INF_OE_B(x)	(HAL_GPIO_WritePin(LS_INF_OE_BACK_GPIO_Port, LS_INF_OE_BACK_Pin, x))

//Első vonalszenzorhoz tartozó ADC-k Chip Select lábainak vezérlése														ELSŐ SZENZOR
#define CSn_AD1(x)	(HAL_GPIO_WritePin(LS_AD_CS1_GPIO_Port, LS_AD_CS1_Pin, x))
#define CSn_AD2(x)	(HAL_GPIO_WritePin(LS_AD_CS2_GPIO_Port, LS_AD_CS2_Pin, x))
#define CSn_AD3(x)	(HAL_GPIO_WritePin(LS_AD_CS3_GPIO_Port, LS_AD_CS3_Pin, x))
#define CSn_AD4(x)	(HAL_GPIO_WritePin(LS_AD_CS4_GPIO_Port, LS_AD_CS4_Pin, x))
//Hátsó vonalszenzorhoz tartozó ADC-k Chip Select lábainak vezérlése													HÁTSÓ SEZNZOR
#define CSn_AD5(x)	(HAL_GPIO_WritePin(LS_AD_CS5_GPIO_Port, LS_AD_CS5_Pin, x))
#define CSn_AD6(x)	(HAL_GPIO_WritePin(LS_AD_CS6_GPIO_Port, LS_AD_CS6_Pin, x))
#define CSn_AD7(x)	(HAL_GPIO_WritePin(LS_AD_CS7_GPIO_Port, LS_AD_CS7_Pin, x))
#define CSn_AD8(x)	(HAL_GPIO_WritePin(LS_AD_CS8_GPIO_Port, LS_AD_CS8_Pin, x))

//ADC-k különböző input channel címei
#define ADDR_IN0 (0)
#define ADDR_IN1 (8)
#define ADDR_IN2 (16)
#define ADDR_IN3 (24)
#define ADDR_IN4 (32)
#define ADDR_IN5 (40)
#define ADDR_IN6 (48)
#define ADDR_IN7 (56)

#define FRONT 0
#define BACK 1

/* Q csempéhez
#define TRASHOLD_LED 1400
#define TRASHOLD_MEAS 1100
#define MAX_OF_0_LINE 2000 //EZEKET MÉG KI KEL MÉRNI
#define MAX_OF_1_LINE 7500
#define MAX_OF_2_LINE 15500
#define MAX_OF_3_LINE 22000
*/
/* Koli padlóhoz*/
#define TRASHOLD_LED 1200

#define TRASHOLD_MEAS_SKILL 1200
#define SKILL_TH_01 2000 //EZEKET MÉG KI KEL MÉRNI
#define SKILL_TH_49 25000

#define TRASHOLD_MEAS_FAST 1500 //1500 volt
#define FAST_TH_01 2000
#define FAST_TH_13 8500
#define FAST_TH_39 30000



//Infravörös LED-kivilágítások
extern uint8_t stateLED0[];
extern uint8_t stateLED1[];
extern uint8_t stateLED2[];
extern uint8_t stateLED3[];

extern uint16_t adValsFront[]; //első szenzor 32 adcértéknek tárolása
extern uint16_t adValsBack[]; //hátsó szenzor 32 adcértéknek tárolása

void LED_Drive(SPI_HandleTypeDef *hspi); //Eza függvény szemlélteti leírással, hogy működik a LED/INFLED vezérlése az STP08DP05 vezérlővel
void Read_AD(SPI_HandleTypeDef *hspi_adc, UART_HandleTypeDef *huart); //Ez a függvény szemlélteti leíráss, hogyan kell egy adott ADC egy adott Input Channeljéről olvasni
void ReadAD0IN0(SPI_HandleTypeDef *hspi_adc);

//Ezek már a tényleges vonaldetektálást valósítják meg
void Line_Sensor_Init(TIM_HandleTypeDef *htim_pwm);
void INF_LED_Drive(SPI_HandleTypeDef *hspi_inf, uint8_t *infLEDstate); //Egy 4 bytos tömbnek megfelelően meghajtjuk az infarvörös LED-eket
void Read1AD(SPI_HandleTypeDef *hspi_adc, uint8_t ForB, uint8_t INx, uint8_t INy, uint8_t adNo); //kb ugyanaz mint a Read_AD csak alkalmazásspecifikusabb és gyorsabb (A chip select a függvényen kívül van)
void Read_Every_4th(SPI_HandleTypeDef *hspi_adc, uint8_t INx, uint8_t INy); //Mind a 4 db AD-ból kiolvas 2 db Input Channelt
void adVals2LED(SPI_HandleTypeDef *hspi_led,UART_HandleTypeDef *huart); //Az adVals tömb elemei alapján megcsinálja a felső LEDsor kivilágítását, a küszöbérték a TRASHOLD macroval állítható
void Line_Sensor_Read_Task(SPI_HandleTypeDef *hspi_inf, SPI_HandleTypeDef *hspi_adc, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period);



#endif /* INC_LINE_SENSOR_H_ */
