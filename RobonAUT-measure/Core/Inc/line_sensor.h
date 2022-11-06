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

void LED_Drive(SPI_HandleTypeDef *hspi); //Eza függvény szemlélteti leírással, hogy működik a LED/INFLED vezérlése az STP08DP05 vezérlővel
void Read_AD(SPI_HandleTypeDef *hspi_adc, UART_HandleTypeDef *huart); //Ez a függvény szemlélteti leíráss, hogyan kell egy adott ADC egy adott Input Channeljéről olvasni


#endif /* INC_LINE_SENSOR_H_ */
