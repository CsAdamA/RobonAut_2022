/*
 * line_sensor.c
 *
 *  Created on: Nov 5, 2022
 *      Author: Zsombesz
 */
#include "line_sensor.h"
#include <string.h>

uint8_t stateLED3[4];
uint8_t stateLED2[4];
uint8_t stateLED1[4];
uint8_t stateLED0[4];

uint16_t adVals[32];

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
	serIn[3]=0b00000011; //a két legszélső LED fog felvillanni mindkét oldalon ->gondold végig hogy melyik bit felel meg a legelső LED-nek és melyik az utolsónak

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
	uint8_t dummy[]={0,0};//2 bytos tömb tökmiind1 mik az értékek (lásd később mire jó)
	uint8_t send[2]; //2 byte a küldséhez
	uint8_t receive[2]; //2 byte a fogadáshoz
	uint16_t ADC_value; //a 12 bites ADC érték
	uint8_t string[20];//ezt a kiiratáshoz

	//Az SPI úgy műkdik hogy adott órajel felfutóélre küldünk is adatot (MOSI-n) és ugyanerre az órajelre fogadunk is adatot (MISO-n)
	//Tehát 16 órajel ciklus alatt kiküldünk 2 bytot és olvasunk is 2 bytot (TransmitReceive fgv.)

	CSn_AD1(0); //Az első ADC-t használjuk (Ez 8 db fototranzisztorhoz kapcsolódik ->8 darab Input Channel)
	CSn_AD2(1);
	CSn_AD3(1);
	CSn_AD4(1); //a másik 3-at nem

	//El kell küldeni SPI-on a control regisztert ami tartalmazza, hogy melyik Input Channel olvasunk a 8 közül
	//a control regiszter 1 byte nagyságú (lásd datasheet 2.oldal):
	//|felső 2 bit mind1 h mi (kivéve 11) | ADC-t azonosító 3 bit | alsó 3 bit mind1 h mi (kivéve 001)|
	send[0]=ADDR_IN1; //=0b00001000 -> 001 aznosítójú Input Channel-t szeretnénk olvasni (IN1, lásd datasheet 3.oldal)
	send[1]=0; //a második byte már teljesen mind1 hogy mi. Itt csak azért küldünk valamit, mert cserébe MISO-n jön az adat
	//send16[0]=0;

	//az első órajel megérkezésétől 16 órajel kell hogy használható legyen legyen az ADC, azaz, hogy célzottan tudjunk kiolvasni bármelyik IN ről (2 nap volt mire ezt megtaláltam egy fórumon)
	HAL_SPI_Transmit(hspi_adc, dummy, 2, 5);//küldünk két dummy bytot -> ezalatt a CLK-n 16 órajelperiódus történik ->pont erre volt szükségünk ->mostmár olvashatunk célzottan
	//fontos hogy ne csak sima várakozást tegyünk ide (delay) hanem tényleg küldjünk 2 byotot az SPI-on mert csak küldés és fogadás közben működteti a mikrovez az SCK-t
	//a dummy küldés fontos hogy a rendes kiolvasás előtt legyen közvetlen
	HAL_SPI_TransmitReceive(hspi_adc, send, receive, 2, 5); //célzott lvasás az előre kiválasztott Input Chanellről

	CSn_AD1(1);//vágeztünk a kiolvasással mostmár nem kell kiválasztva legyen az ADC

	//A beérkezett 2 byte-ból (2* 8 bit) össze kell raknunk a 12 bites értékünket ->jön a "bitvarázs"
	uint16_t also8bit, felso8bit; //ezek a segédváltozók amugy nem kellenének, csak azért csinálom a műveletet így, hogy jobban áttekinthető legyen nektek

	also8bit = (uint16_t)receive[1]; //a 8 bites bytot átrakjuk 16 bites változóba 0b........ -> 0b00000000........

	felso8bit= receive[0] & 0b00001111; //az első bytnak csak az alsó 4 bitjére vagyunk kiváncsiak, a többit nullázzuk ki (user manual szerint amugy is 0-ák de biztos ami biztos) 0b........ -> 0b0000....
	//A User manual szerint egyébként ha a felső 4 bit nem 0 akkor hibás az olvasott érték-> ezt későbbi fügvénybe hibadetektálásra használjuk
	felso8bit= felso8bit << 8; //az előbb emíltett 4 bitből lesz 12 bit legfelső 4 bitje -> a helyükre kell őket tolni 0b000000000000.... -> 0b0000....00000000
	//az előző művelet 2^8-al való szorzással egyenrangú de a bitműveleteteket sokkal gyorsabban elvégzi a proci, majd ti is gyorsan megtanuljátok őket használni/érteni

	ADC_value = felso8bit | also8bit; //a 12 bites értékünket összerakjuk a két részből 4bit + 8bit = 12 bit: 0b0000....00000000 | 0b00000000........ = 0b0000............
	//az előző művelet egyébként ugyanaz mint: felso8bit + also8bit, de a bool "vagyolás" gyorasbb mint az összeadás (és biztonságosabb is -> ezt gondold át hogy miért)
	//ADC_value=receive16;

	//kiiratjuk az olvasott értéket
	memset(string,0,20);
	//még egy infravörös ledet sem világítottunk meg -> a fototranzisztor teljesen zárt-> Az ADC bemenete 3V3 feszültségen van (lásd RobonAUT-LineSensor.pdf 3.oldal)-> 2^12 körüli ADC értékeket várunk
	sprintf(string,"%d\r\n",ADC_value);
	HAL_UART_Transmit(huart, string, strlen(string), 20);
}

//Vonalszenzor inicializálása
void Line_Sensor_Init(void)
{
	//Még egyik ADC sincs kiválasztva
	CSn_AD1(1);
	CSn_AD2(1);
	CSn_AD3(1);
	CSn_AD4(1);
	int i=0;
	for(i=0;i<32;i++)
	{
		adVals[i]=0;// 4095 az alapérték, mivel ez olyan mintha még semelyik AD-ba nem érkezett volna infrafény
	}
	stateLED3[0]=stateLED3[1]=stateLED3[2]=stateLED3[3]=0b10001000;
	stateLED2[0]=stateLED2[1]=stateLED2[2]=stateLED2[3]=0b01000100;
	stateLED1[0]=stateLED1[1]=stateLED1[2]=stateLED1[3]=0b00100010;
	stateLED0[0]=stateLED0[1]=stateLED0[2]=stateLED0[3]=0b00010001;

}

//Egy 4 bytos tömbnek megfelelően meghajtjuk az infarvörös LED-eket
void INF_LED_Drive(SPI_HandleTypeDef *hspi_inf,uint8_t *infLEDstate)
{
	INF_OE(0);
	INF_LE(0);
	HAL_SPI_Transmit(hspi_inf, infLEDstate, 4, 10);
	INF_LE(1);
	INF_LE(0);
}

//kb ugyanaz mint a Read_AD csak alkalmazásspecifikusabb és gyorsabb (A chip select a függvényen kívül van)
void Read1AD(SPI_HandleTypeDef *hspi_adc, uint8_t INx, uint8_t adNo)
{
	static uint8_t send[]={0,0}; //2 byte a küldséhez
	static uint8_t rcv[]={0,0}; //2 byte a fogadáshoz
	send[0]=INx*8;

	HAL_SPI_Transmit(hspi_adc,send,2,5);
	HAL_SPI_TransmitReceive(hspi_adc, send, rcv, 2, 5);

	if(!(rcv[0]&240)) //ellenőrizzük hogy tényleg 0-e a felső 4 bit
	{
		adVals[INx+adNo*8] = (((uint16_t)rcv[0])<<8)  | ((uint16_t)rcv[1]);
	}
}

//Mind a 4 db AD-ból kiolvas 2 db Input Channelt
void Read_Every_4th(SPI_HandleTypeDef *hspi_adc, uint8_t INx1, uint8_t INx2)
{
	//AD1-ből olvassuk az InAddr1-t és InAddr2-t
	CSn_AD1(0);
	Read1AD(hspi_adc, INx1,0);
	CSn_AD1(1);
	CSn_AD1(0);
	Read1AD(hspi_adc, INx2,0);
	CSn_AD1(1);

	//AD2-ből olvassuk az InAddr1-t és InAddr2-t
	CSn_AD2(0);
	Read1AD(hspi_adc, INx1,1);
	CSn_AD2(1);
	CSn_AD2(0);
	Read1AD(hspi_adc, INx2,1);
	CSn_AD2(1);

	//AD3-ből olvassuk az InAddr1-t és InAddr2-t
	CSn_AD3(0);
	Read1AD(hspi_adc, INx1,2);
	CSn_AD3(1);
	CSn_AD3(0);
	Read1AD(hspi_adc, INx2,2);
	CSn_AD3(1);

	//AD4-ből olvassuk az InAddr1-t és InAddr2-t
	CSn_AD4(0);
	Read1AD(hspi_adc, INx1,3);
	CSn_AD4(1);
	CSn_AD4(0);
	Read1AD(hspi_adc, INx2,3);
	CSn_AD4(1);
}

//Az adVals tömb elemei alapján megcsinálja a felső LEDsor kivilágítását, a küszöbérték a TRASHOLD macroval állítható
void adVals2LED(SPI_HandleTypeDef *hspi_led,UART_HandleTypeDef *huart)
{
#ifdef LS_DEBUG
	uint8_t str[20];
	memset(str,0,20);
#endif
	uint8_t i;
	uint8_t LEDstate[4]={0,0,0,0};
	uint8_t byteNo, bitNo;
/**/
	for(i=0;i<32;i++)
	{
		byteNo = 3- i/8;
		bitNo= i%8;
		if(adVals[i] > TRASHOLD)
		{
			LEDstate[byteNo] |= (1<<bitNo);
		}
		else
		{
			LEDstate[byteNo] &= (~(1<<bitNo)); // ~bittwise negation
		}
#ifdef LS_DEBUG
		sprintf(str,"ADC%2d: %4d\r\n",i,adVals[i]);
		HAL_UART_Transmit(huart, str, strlen(str), 20);
#endif
	}
#ifdef LS_DEBUG
	sprintf(str,"\r\n\n");
	HAL_UART_Transmit(huart, str, strlen(str), 20);
#endif

	LED_OE(0);
	LED_LE(0);
	HAL_SPI_Transmit(hspi_led, LEDstate, 4, 10);
	LED_LE(1);
	LED_LE(0);

}

//vonaldetektálás->az eredmény a felette lévő soron látható.
void Line_Sensor_Read_Task(SPI_HandleTypeDef *hspi_inf, SPI_HandleTypeDef *hspi_adc, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{
	static uint8_t lsState=0;
	static uint32_t lsReadTick=0;

	if(lsReadTick>tick) return;
	lsReadTick=tick+period;

	switch(lsState)
	{
	case 0:
		//Az első infraLED világít utána minden negyedik
		INF_LED_Drive(hspi_inf,stateLED0);
		//Minden ADC-nek az IN0-t és IN4-t ovlassuk
		Read_Every_4th(hspi_adc,0,4);
		lsState++;
		break;

	case 1:
		//A második infraLED világít utána minden negyedik
		INF_LED_Drive(hspi_inf,stateLED1);
		//Minden ADC-nek az IN1-t és IN5-t ovlassuk
		Read_Every_4th(hspi_adc,1,5);
		lsState++;
		break;

	case 2:
		//A harmadik infraLED világít utána minden negyedik
		INF_LED_Drive(hspi_inf,stateLED2);
		//Minden ADC-nek az IN2-t és IN6-t ovlassuk
		Read_Every_4th(hspi_adc,2,6);
		lsState++;
		break;

	case 3:
		//minden negyedik infraLED világít
		INF_LED_Drive(hspi_inf,stateLED3);
		//Minden ADC-nek az IN3-t és IN7-ét ovlassuk
		Read_Every_4th(hspi_adc,3,7);
		lsState++;
		break;

	case 4://kiértékelés
		adVals2LED(hspi_inf,huart);//a felső LED sor kivilágtása az ADC értékek alapján
		lsState=0;
#ifdef LS_DEBUG
		lsReadTick+=2000;
#endif
		break;
	}
}


