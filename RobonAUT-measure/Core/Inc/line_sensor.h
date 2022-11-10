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

//LED vezérlő reteszelő és kimenet engedélyező LED-jének állítása
#define LED_LE(x)	(HAL_GPIO_WritePin(LS_LED_LE_FRONT_GPIO_Port, LS_LED_LE_FRONT_Pin, x))
#define LED_OE(x)	(HAL_GPIO_WritePin(LS_LED_OE_FRONT_GPIO_Port, LS_LED_OE_FRONT_Pin, x))

//InfraLED vezérlő reteszelő és kimenet engedélyező LED-jének állítása
#define INF_LE(x)	(HAL_GPIO_WritePin(LS_INF_LE_FRONT_GPIO_Port, LS_INF_LE_FRONT_Pin, x))
#define INF_OE(x)	(HAL_GPIO_WritePin(LS_INF_OE_FRONT_GPIO_Port, LS_INF_OE_FRONT_Pin, x))

//Első vonalszenzorhoz tartozó ADC-k Chip Select lábainak vezérlése
#define CSn_AD1(x)	(HAL_GPIO_WritePin(LS_AD_CS1_GPIO_Port, LS_AD_CS1_Pin, x))
#define CSn_AD2(x)	(HAL_GPIO_WritePin(LS_AD_CS2_GPIO_Port, LS_AD_CS2_Pin, x))
#define CSn_AD3(x)	(HAL_GPIO_WritePin(LS_AD_CS3_GPIO_Port, LS_AD_CS3_Pin, x))
#define CSn_AD4(x)	(HAL_GPIO_WritePin(LS_AD_CS4_GPIO_Port, LS_AD_CS4_Pin, x))

//ADC-k különböző input channel címei
#define ADDR_IN0 (0)
#define ADDR_IN1 (8)
#define ADDR_IN2 (16)
#define ADDR_IN3 (24)
#define ADDR_IN4 (32)
#define ADDR_IN5 (40)
#define ADDR_IN6 (48)
#define ADDR_IN7 (56)

#define TRASHOLD (1500)

//Infravörös LED-állapotok
extern uint8_t stateLED0[];
extern uint8_t stateLED1[];
extern uint8_t stateLED2[];
extern uint8_t stateLED3[];

extern uint16_t adVals[];

void LED_Drive(SPI_HandleTypeDef *hspi); //Eza függvény szemlélteti leírással, hogy működik a LED/INFLED vezérlése az STP08DP05 vezérlővel
void Read_AD(SPI_HandleTypeDef *hspi_adc, UART_HandleTypeDef *huart); //Ez a függvény szemlélteti leíráss, hogyan kell egy adott ADC egy adott Input Channeljéről olvasni


//Ezek már a tényleges vonaldetektálást valósítják meg
void Line_Sensor_Init(void);
void INF_LED_Drive(SPI_HandleTypeDef *hspi_inf, uint8_t *infLEDstate); //Egy 4 bytos tömbnek megfelelően meghajtjuk az infarvörös LED-eket
void Read1AD(SPI_HandleTypeDef *hspi_adc, uint8_t INx, uint8_t adNo); //kb ugyanaz mint a Read_AD csak alkalmazásspecifikusabb és gyorsabb (A chip select a függvényen kívül van)
void Read_Every_4th(SPI_HandleTypeDef *hspi_adc, uint8_t INx1, uint8_t INx2); //Mind a 4 db AD-ból kiolvas 2 db Input Channelt
void adVals2LED(SPI_HandleTypeDef *hspi_led,UART_HandleTypeDef *huart); //Az adVals tömb elemei alapján megcsinálja a felső LEDsor kivilágítását, a küszöbérték a TRASHOLD macroval állítható
void Line_Sensor_Read_Task(SPI_HandleTypeDef *hspi_inf, SPI_HandleTypeDef *hspi_adc, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period); //vonaldetektálás->az eredmény a felette lévő soron látható.



#endif /* INC_LINE_SENSOR_H_ */
