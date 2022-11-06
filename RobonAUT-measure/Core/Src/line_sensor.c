/*
 * line_sensor.c
 *
 *  Created on: Nov 5, 2022
 *      Author: Zsombesz
 */
#include "line_sensor.h"
#include <string.h>

void LED_Drive(SPI_HandleTypeDef *hspi) //az egész LED sort (32 LED) átaírjuk egyszerre
{
	//Az SPI2-t 8 bites módban használjuk -> bytokat lehet vele küldeni
	uint8_t serIn[4]; //az SPI-on bytokat küldünk -> 4 byte = 32 bit ->pont ennyi LED ünk van.

	LED_OE(0);//Kimenet engedélyezése. Ha engedélyezve van a kimenet, akkor
	//a párhuzamos kimeneten vagy a legutóbbi bereteszelt eredmény látható (ha LE=0)
	//a jelenlegi beléptetett állapot látható a kimeneten ha nincs reteszelés (LE=1)

	serIn[0]=0b11000000; //beállítjuk mind a 32 bitet
	serIn[1]=0b00000000;
	serIn[2]=0b00000000;
	serIn[3]=0b00000011; //a két legszélső LED fog felvillanni mindkét oldalon

	LED_LE(0); //a Latch 0 ->most még csak léptetünk, a köztes állapotok ne jelenlenek meg a kimeneten
	//-> ezt úgy oldjuk meg, hogy lereteszeljük a kimenetet, tehát a következő reteszfeloldásig a mostani állpot lesz látható a párhuzamos kimeneten

	HAL_SPI_Transmit(hspi, serIn, 4, 100);//kiléptetjük a 32 bitet (továbbiakban 32bit := bitkígyó)

	LED_LE(1);//a teljes bitkígyó beérkezett -> feloldjuk a reteszelést így az új állapot kerül a kimenetre
	LED_LE(0);//ismét bereteszeljük a kimenetet a következő teljes bitkígyó beérkezéséig
}

void Read_AD(SPI_HandleTypeDef *hspi_adc, UART_HandleTypeDef *huart) //egy adott ADC egy adott Input Channeljéről olvasunk
{
	//0b... bináris formátum
	//0x... hexadecimális formátum
	//számábrázolás: uint8_t a = 43 (decimális) =0b00101011 (bináris) = 0x2B (hexadecimális)

	//Az SPI1-et is 8 bites módban használjuk-> így a 12 bites ADC érték 2 bytból kell majd összerakni
	uint8_t send[2]; //2 byte a küldséhez
	uint8_t receive[2]; //2 byte a fogadáshoz
	uint16_t ADC_value; //a 12 bites ADC érték
	uint8_t string[20];//ezt a kiiratáshoz

	//Az SPI úgy műkdik hogy adott órajel felfutóélre küldünk is adatot (MOSI-n) és ugyanerre az órajelre fogadunk is adatot (MISO-n)
	//Tehát 16 órajel ciklus alatt kiküldünk 2 bytot és olvasunk is 2 bytot (TransmitReceive fgv.)

	CSn_AD1(0); //Az első ADC-t használjuk (Ez 8 db fototranzisztorhoz kapcsolódik ->8 darab Input Channel)
	CSn_AD2(1); //a másik 3-at nem
	CSn_AD3(1);
	CSn_AD4(1);

	//El kell küldeni SPI-on a control regisztert ami tartalmazza, hogy melyik Input Channel olvasunk a 8 közül
	//a control regiszter 1 byte nagyságú (lásd datasheet 2.oldal):
	//|felső 2 bit mind1 h mi (kivéve 11) | ADC-t azonosító 3 bit | alsó 3 bit mind1 h mi (kivéve 001)|
	send[0]=8; //=0b00001000 -> 001 aznosítójú Input Channel-t szeretnénk olvasni (IN1, lásd datasheet 3.oldal)
	send[0]=0; //a második byte már teljesen mind1 hogy mi. Itt csak azért küldünk valamit, mert cserébe MISO-n jön az adat

	//küldés (2 byte) - fogadás (2 byte) egyszerre
	HAL_SPI_TransmitReceive(hspi_adc, send, receive, 2, 100);

	CSn_AD1(1);//vágeztünk a kiolvasással mostmár nem kell kiválasztva legyen az ADC

	//A beérkezett 2 byte-ból (2* 8 bit) össze kell raknunk a 12 bites értékünket ->jön a "bitvarázs"
	uint16_t also8bit, felso8bit; //ezek a segédváltozók amugy nem kellenének, csak azért csinálom a műveletet így, hogy jobban áttekinthető legyen nektek

	also8bit = (uint16_t)receive[1]; //a 8 bites bytot átrakjuk 16 bites változóba 0b........ -> 0b00000000........

	felso8bit= receive[0] & 0b00001111; //az első bytnak csak az alsó 4 bitjére vagyunk kiváncsiak, a többit nullázzuk ki (user manual szerint amugy is 0-ák de biztos ami biztos) 0b........ -> 0b0000....
	felso8bit= felso8bit << 8; //az előbb emíltett 4 bitből lesz 12 bit legfelső 4 bitje -> a helyükre kell őket tolni 0b000000000000.... -> 0b0000....00000000
	//az előző művelet 2^8-al való szorzással egyenrangú de a bitműveleteteket sokkal gyorsabban elvégzi a proci, majd ti is gyorsan megtanuljátok őket használni/érteni

	ADC_value = felso8bit | also8bit; //a 12 bites értékünket összerakjuk a két részből 4bit + 8bit = 12 bit: 0b0000....00000000 | 0b00000000........ = 0b0000............
	//az előző művelet egyébként ugyanaz mint: felso8bit + also8bit, de a bool "vagyolás" gyorasbb mint az összeadás (és biztonságosabb is -> ezt gondold át hogy miért)

	//kiiratjuk az olvasott értéket
	memset(string,0,20);
	//még egy infravörös ledet sem világítottunk meg -> a fototranzisztor teljesen zárt-> Az ADC bemenete 3V3 feszültségen van (lásd RobonAUT-LineSensor.pdf 3.oldal)-> 2^12 körüli ADC értékeket várunk
	sprintf(string,"%d\r\n",ADC_value);
	HAL_UART_Transmit(huart, string, strlen(string), 100);
}

