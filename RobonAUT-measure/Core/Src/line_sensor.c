/*
 * line_sensor.c
 *
 *  Created on: Nov 5, 2022
 *      Author: Zsombesz
 */
#include "line_sensor.h"
#include <string.h>
#include "configG0.h"

uint8_t stateLED3[4];
uint8_t stateLED2[4];
uint8_t stateLED1[4];
uint8_t stateLED0[4];

uint16_t adValsFront[32];
uint16_t adValsBack[32];

uint8_t lsData[5];

void LED_Drive(SPI_HandleTypeDef *hspi) //az egész LED sort (32 LED) átaírjuk egyszerre
{
	//Az SPI2-t 8 bites módban használjuk -> bytokat lehet vele küldeni
	uint8_t serIn[4]; //az SPI-on bytokat küldünk -> 4 byte = 32 bit ->pont ennyi LED ünk van.

	LED_OE_F(0);//Kimenet engedélyezése. Ha engedélyezve van a kimenet, akkor
	//a párhuzamos kimeneten vagy a legutóbbi bereteszelt eredmény látható (ha LE=0)
	//a jelenlegi beléptetett állapot látható a kimeneten ha nincs reteszelés (LE=1)

	serIn[0]=0b11100111; //beállítjuk mind a 32 bitet
	serIn[1]=0b11111111;
	serIn[2]=0b11111111;
	serIn[3]=0b11111111; //a két legszélső LED fog felvillanni mindkét oldalon ->gondold végig hogy melyik bit felel meg a legelső LED-nek és melyik az utolsónak

	LED_LE_F(0); //a Latch 0 ->most még csak léptetünk, a köztes állapotok ne jelenlenek meg a kimeneten
	//-> ezt úgy oldjuk meg, hogy lereteszeljük a kimenetet, tehát a következő reteszfeloldásig a mostani állpot lesz látható a párhuzamos kimeneten

	HAL_SPI_Transmit(hspi, serIn, 4, 100);//kiléptetjük a 32 bitet (továbbiakban 32bit := bitkígyó)

	LED_LE_F(1);//a teljes bitkígyó beérkezett -> feloldjuk a reteszelést így az új állapot kerül a kimenetre
	LED_LE_F(0);//ismét bereteszeljük a kimenetet a következő teljes bitkígyó beérkezéséig
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
	HAL_SPI_Transmit(hspi_adc, dummy, 2, 2);//küldünk két dummy bytot -> ezalatt a CLK-n 16 órajelperiódus történik ->pont erre volt szükségünk ->mostmár olvashatunk célzottan
	//fontos hogy ne csak sima várakozást tegyünk ide (delay) hanem tényleg küldjünk 2 byotot az SPI-on mert csak küldés és fogadás közben működteti a mikrovez az SCK-t
	//a dummy küldés fontos hogy a rendes kiolvasás előtt legyen közvetlen
	HAL_SPI_TransmitReceive(hspi_adc, send, receive, 2, 2); //célzott lvasás az előre kiválasztott Input Chanellről

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
	HAL_UART_Transmit(huart, string, strlen(string), 2);
}

//Vonalszenzor inicializálása
void Line_Sensor_Init(TIM_HandleTypeDef *htim_pwm)
{
	//Még egyik ADC sincs kiválasztva
	CSn_AD1(1);
	CSn_AD2(1);
	CSn_AD3(1);
	CSn_AD4(1);
	CSn_AD5(1);
	CSn_AD6(1);
	CSn_AD7(1);
	CSn_AD8(1);

	int i=0;
	for(i=0;i<32;i++)
	{
		adValsFront[i]=adValsBack[i]=0;// 4095 az alapérték, mivel ez olyan mintha még semelyik AD-ba nem érkezett volna infrafény
	}
	stateLED3[0]=stateLED3[1]=stateLED3[2]=stateLED3[3]=0b10001000;
	stateLED2[0]=stateLED2[1]=stateLED2[2]=stateLED2[3]=0b01000100;
	stateLED1[0]=stateLED1[1]=stateLED1[2]=stateLED1[3]=0b00100010;
	stateLED0[0]=stateLED0[1]=stateLED0[2]=stateLED0[3]=0b00010001;

	//első és hátsó IR LED-ek kimenetének engedélyezése
	INF_OE_F(0);
	INF_OE_B(0);
	//első és hátsó látható LED-ek kimenetének engedélyezése
	LED_OE_F_L(htim_pwm);//LED_OE(0);
	LED_OE_B_L(htim_pwm);//LED_OE(0);
	//első és hátsó IR LED-ek reteszelése
	INF_LE_F(0);
	INF_LE_B(0);
	//első és hátsó látható LED-ek reteszelése
	LED_LE_F(0);
	LED_LE_B(0);
}

//Egy 4 bytos tömbnek megfelelően meghajtjuk az infarvörös LED-eket
void INF_LED_Drive(SPI_HandleTypeDef *hspi_inf,uint8_t *infLEDstate)
{

	HAL_SPI_Transmit(hspi_inf, infLEDstate, 4, 2);

	//első és hátsó IR LED-ek reteszelésének feloldása
	INF_LE_F(1);
	INF_LE_B(1);
	//első és hátsó IR LED-eket ismét lereteszeljük
	INF_LE_F(0);
	INF_LE_B(0);

}

//kb ugyanaz mint a Read_AD csak alkalmazásspecifikusabb és gyorsabb (A chip select a függvényen kívül van)
void Read1AD(SPI_HandleTypeDef *hspi_adc, uint8_t ForB, uint8_t INx, uint8_t INy, uint8_t adNo)
{
	static uint8_t send[]={0,0,0,0,0,0}; //2 byte a küldséhez
	static uint8_t rcv[]={0,0,0,0,0,0}; //2 byte a fogadáshoz

	send[0]=INx*8+135;
	send[2]=INy*8+135;
	send[4]=INy*8+135;
	HAL_SPI_TransmitReceive(hspi_adc, send, rcv, 6,2);
	/*
	if(adNo==0 && INy==0)
	{
		HAL_SPI_TransmitReceive(hspi_adc, send, rcv, 4,2);
	}
	 */
	if(!(rcv[2]&240) && !(rcv[4]&240)) //ellenőrizzük hogy tényleg 0-e a felső 4 bit
	{
		if(ForB==BACK)
		{
			adValsBack[INx+adNo*8] = (((uint16_t)rcv[2])<<8)  | ((uint16_t)rcv[3]);
			adValsBack[INy+adNo*8] = (((uint16_t)rcv[4])<<8)  | ((uint16_t)rcv[5]);
		}
		else //forB==FRONT
		{
			adValsFront[31-INx-adNo*8] = (((uint16_t)rcv[2])<<8)  | ((uint16_t)rcv[3]);
			adValsFront[31-INy-adNo*8] = (((uint16_t)rcv[4])<<8)  | ((uint16_t)rcv[5]);
		}
	}
}

//Mind a 4 db AD-ból kiolvas 2 db Input Channelt
void Read_Every_4th(SPI_HandleTypeDef *hspi_adc, uint8_t INx, uint8_t INy)
{
	//ELSŐ SZENZOR ad2 és ad1 az első szenzornál fel van cserélve mert csak ig mukodik de hogy miért azt ne kérdezd xddd
	//AD2-ből olvassuk az INx-t és INy-t
	CSn_AD2(0);
	Read1AD(hspi_adc, FRONT,INx,INy,1);
	CSn_AD2(1);

	//AD1-ből olvassuk az az INx-t és INy-t
	CSn_AD1(0);
	Read1AD(hspi_adc, FRONT,INx,INy,0);
	CSn_AD1(1);

	//AD3-ből olvassuk az INx-t és INy-t
	CSn_AD3(0);
	Read1AD(hspi_adc, FRONT,INx,INy,2);
	CSn_AD3(1);

	//AD4-ből olvassuk az az INx-t és INy-t
	CSn_AD4(0);
	Read1AD(hspi_adc, FRONT,INx,INy,3);
	CSn_AD4(1);

	//HÁTSÓ SZENZOR
	//AD1-ből olvassuk az INx-t és INy-t
	CSn_AD5(0);
	Read1AD(hspi_adc, BACK,INx,INy,0);
	CSn_AD5(1);

	//AD2-ből olvassuk az INx-t és INy-t
	CSn_AD6(0);
	Read1AD(hspi_adc, BACK,INx,INy,1);
	CSn_AD6(1);

	//AD3-ből olvassuk az INx-t és INy-t
	CSn_AD7(0);
	Read1AD(hspi_adc, BACK,INx,INy,2);
	CSn_AD7(1);

	//AD4-ből olvassuk az INx-t és INy-t
	CSn_AD8(0);
	Read1AD(hspi_adc, BACK,INx,INy,3);
	CSn_AD8(1);
}

//Az adVals tömb elemei alapján megcsinálja a felső LEDsor kivilágítását, a küszöbérték a TRASHOLD macroval állítható
void adVals2LED(SPI_HandleTypeDef *hspi_led,UART_HandleTypeDef *huart)
{
#ifdef LS_DEBUG
	uint8_t str[80];
	memset(str,0,50);
#endif
	const float alpha=0.4;
	const float invalpha= 0.6;
	static uint8_t LEDstateF[4]={0,0,0,0};
	static uint8_t LEDstateB[4]={0,0,0,0};
	static uint8_t byteNo, bitNo;
	uint32_t sumToCnt=0;
	uint32_t sum[]={0,0,0,0};

	//ideiglenes változók mert az egész számos műveletek gyorsabbak
	uint32_t wAvgNew[]={0,0,0,0};
	uint8_t lineCntNew=0;
	//a zajos értékeket a mérés folyamán szűrjük ehhez memórajelleg és
	static float wAvgOld[]={123,123,123,123};
	static float lineCntOld=0;
	//Végül uintet küldünk uarton
	uint32_t wAvgFiltered[]={0,0,0,0};
	uint8_t lineCntFiltered=0;
	static uint8_t lineCntFilteredPrev=0;

	int i,j=0;
	uint8_t lineDetected=0;

/**/
	for(i=0;i<32;i++)
	{
		/******************Visszajelző LED-ek kivilágítás********************/
		byteNo = 3- i/8;
		bitNo= i%8;
		//első
		if(adValsBack[i] > TRASHOLD_LED)LEDstateB[byteNo] |= (1<<bitNo);
		else LEDstateB[byteNo] &= (~(1<<bitNo)); // ~bittwise negation

		//hátsó
		if(adValsFront[i] > TRASHOLD_LED) LEDstateF[3-byteNo] |= (1<<(7-bitNo));
		else LEDstateF[3-byteNo] &= (~(1<<(7-bitNo))); // ~bittwise negation

		/******************Gyors módban az első és hátsó vonalszenzorok által érzékelt vonalak sulyozott közepét nézzük********************/
		if(mode==FAST)
		{
			//Szabályozó bemenet számolás
			if(adValsFront[i] > TRASHOLD_MEAS)
			{
				sum[0] += (uint32_t) adValsFront[i];
				wAvgNew[0] += (uint32_t)adValsFront[i] *i;
			}
			if(adValsBack[i] > TRASHOLD_MEAS)
			{
				sum[1] += (uint32_t) adValsBack[i];
				wAvgNew[1] += (uint32_t) adValsBack[i] *i;
			}
		}
		/******************Ügyességi módban az első vonalszenzor alatt lévő max 4 vonal pozícióját külön vizsgáljuk********************/
		else if(mode==SKILL)
		{
			if(adValsFront[i] > TRASHOLD_MEAS)
			{
				sumToCnt+= (uint32_t) adValsFront[i];
				if(j<4)
				{
					sum[j] += (uint32_t) adValsFront[i];
					wAvgNew[j] += (uint32_t) adValsFront[i] *i;
				}
				if(!lineDetected)lineCntNew++;
				lineDetected=1;
			}
			else if(lineDetected)
			{
				j++;
				lineDetected=0;
			}
		}
#ifdef LS_DEBUG
		sprintf(str,"ADC%2d: elso-> %4d, hatso-> %4d\r\n",i,adValsFront[i],adValsBack[i]);
		HAL_UART_Transmit(huart, str, strlen(str), 50);
#endif

	}
	/**********************FAST MODE KIÉRTÉKELÉS**************************/
	if(mode==FAST)
	{
		if(sum[0]>0)
		{
			wAvgNew[0] = wAvgNew[0]*8/sum[0];
			wAvgOld[0] = alpha*(float)wAvgNew[0]+invalpha*wAvgOld[0]; //ELSŐ VONALPOZÍCIÓ SZŰRÉS
		}
		wAvgFiltered[0] = (uint32_t)(wAvgOld[0]+0.5);

		if(sum[1]>0)
		{
			wAvgNew[1]  = wAvgNew[1]*8/sum[1];
			wAvgOld[1] = alpha*(float)wAvgNew[1]+invalpha*wAvgOld[1]; //HÁTSÓ VONALPOZÍCIÓ SZŰRÉS
		}
		wAvgFiltered[1] = (uint32_t)(wAvgOld[1]+0.5);

		/**********************VONALSZÁMLÁLÁS***********************/
		if(sum[0] < MAX_OF_0_LINE) lineCntNew = 0;//nincs vonal az első vonalszenzor alatt
		else if(sum[0] < MAX_OF_1_LINE) lineCntNew = 1;//1 vonal van az első vonalszenzor alatt
		else if(sum[0] < MAX_OF_3_LINE) lineCntNew = 3;//3 vonal van az első vonalszenzor alatt
		else if(sum[0] < MAX_OF_4_LINE) lineCntNew = 4;//4 vonal van az első vonalszenzor alatt
		else lineCntNew = 10;

		/***********************VONALSZÁM SZŰRÉS***********************/
		lineCntOld=alpha*lineCntNew+invalpha*lineCntOld;
		if(lineCntOld<0.5) lineCntFiltered=0;
		else if(lineCntOld<2) lineCntFiltered=1;
		else if(lineCntOld<3.5) lineCntFiltered=3;
		else if(lineCntOld<4.5) lineCntFiltered=4;
		else lineCntFiltered=10;

		/*******************KÜLDŐ ADATTÖMBE MÁSOLÁS********************/
		__disable_irq();//uart interrupt letiltás ->amíg írjuka  kiküldendő tömböt addig ne kérjen adatot az F4
		sendByteG0[1]=lineCntFiltered;
		sendByteG0[2]=wAvgFiltered[0];
		sendByteG0[3]=wAvgFiltered[1];
		__enable_irq();//uart interrupt engedélyezés

		/**************DEBUGG KIÍRATÁS ADATTÖMBE MÁSOLÁS***************/
#ifdef LS_DEBUG
		sprintf(str,"sum elso: %d   sum hatso: %d,   vonalszam:%d\n\r",sum[0],sum[1],lineCntNew);
		HAL_UART_Transmit(huart, str, strlen(str), 50);

		sprintf(str,"atl elso: %d,   atl hatso: %d\n\n\r",wAvgNew[0],wAvgNew[1]);
		HAL_UART_Transmit(huart, str, strlen(str), 50);
#endif
	}

	/**********************SKILL MODE KIÉRTÉKELÉS**************************/
	else if(mode==SKILL)
	{
		/**********************VONALSZÁMLÁLÁS***********************/
		if(sumToCnt < MAX_OF_0_LINE)lineCntNew=0;//nincs vonal az első vonalszenzor alatt
		else if(sumToCnt > MAX_OF_4_LINE)lineCntNew=10;

		/***********************VONALSZÁM SZŰRÉS***********************/

		lineCntOld=alpha*(float)lineCntNew+invalpha*lineCntOld;

		if(lineCntOld<0.5) lineCntFiltered=0;
		else if(lineCntOld<1.5) lineCntFiltered=1;
		else if(lineCntOld<2.5) lineCntFiltered=2;
		else if(lineCntOld<3.5) lineCntFiltered=3;
		else if(lineCntOld<4.5) lineCntFiltered=4;
		else lineCntFiltered=10;

		for(j=0;j<4;j++)
		{
			if(sum[j]>0)
			{
				wAvgNew[j]=wAvgNew[j]*8/sum[j];
				if(lineCntFiltered!=lineCntFilteredPrev)
				{
					wAvgOld[j]=(float)wAvgNew[j];
					lineCntFilteredPrev=lineCntFiltered;
				}
				else wAvgOld[j]=alpha*(float)wAvgNew[j]+invalpha*wAvgOld[j];
			}
			wAvgFiltered[j]=(uint32_t)(wAvgOld[j]+0.5);
		}

		/*******************KÜLDŐ ADATTÖMBE MÁSOLÁS********************/
		__disable_irq();//uart interrupt letiltás ->amíg írjuka  kiküldendő tömböt addig ne kérjen adatot az F4
		sendByteG0[1]=lineCntFiltered;
		sendByteG0[2]=wAvgFiltered[0];
		sendByteG0[3]=wAvgFiltered[1];
		sendByteG0[4]=wAvgFiltered[2];
		sendByteG0[5]=wAvgFiltered[3];
		__enable_irq();//uart interrupt engedélyezés

		/**************DEBUGG KIÍRATÁS ADATTÖMBE MÁSOLÁS***************/
#ifdef LS_DEBUG
		sprintf(str,"sum 1.vonal: %d,  sum 2.vonal: %d,  sum 3.vonal: %d,  sum 4.vonal: %d,  sum ALL: %d\n\r",sum[0],sum[1],sum[2],sum[3],sumToCnt);
		HAL_UART_Transmit(huart, str, strlen(str), 50);

		sprintf(str,"atl 1.vonal: %d,  atl 2.vonal: %d,  atl 3.vonal: %d,  atl 4.vonal: %d,  vonalszam: %d\n\n\r",wAvgNew[0],wAvgNew[1],wAvgNew[2],wAvgNew[3],lineCntNew);
		HAL_UART_Transmit(huart, str, strlen(str), 50);
#endif
	}

	HAL_SPI_Transmit(hspi_led, LEDstateF, 4, 2);
	LED_LE_F(1);
	LED_LE_F(0);

	HAL_SPI_Transmit(hspi_led, LEDstateB, 4, 2);
	LED_LE_B(1);
	LED_LE_B(0);
}

//vonaldetektálás->az eredmény a felette lévő soron látható.
void Line_Sensor_Read_Task(SPI_HandleTypeDef *hspi_inf, SPI_HandleTypeDef *hspi_adc, UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{
	static uint32_t lsReadTick=2;
	/*futásidő mérés
	static uint16_t cnt=0;
	static uint16_t cnt_prev=0;
	static uint8_t str[20];
	static int i=0;
	 */
	if(lsReadTick>tick) return;
	lsReadTick=tick+period;

	//Az első infraLED világít utána minden negyedik
	INF_LED_Drive(hspi_inf,stateLED0);
	//Minden ADC-nek az IN0-t és IN4-t ovlassuk
	Read_Every_4th(hspi_adc,4,0);

	//A második infraLED világít utána minden negyedik
	INF_LED_Drive(hspi_inf,stateLED1);
	//Minden ADC-nek az IN1-t és IN5-t ovlassuk
	Read_Every_4th(hspi_adc,1,5);

	//A harmadik infraLED világít utána minden negyedik
	INF_LED_Drive(hspi_inf,stateLED2);
	//Minden ADC-nek az IN2-t és IN6-t ovlassuk
	Read_Every_4th(hspi_adc,2,6);

	//minden negyedik infraLED világít
	INF_LED_Drive(hspi_inf,stateLED3);
	//Minden ADC-nek az IN3-t és IN7-ét ovlassuk
	Read_Every_4th(hspi_adc,3,7);

	adVals2LED(hspi_inf,huart);//a felső LED sor kivilágtása az ADC értékek alapján
	/* futásidő mérés
	lsReadTick=0;
	if(i>=999)
	{
		cnt=TIM6->CNT;
		sprintf(str,"%d\r\n",(cnt-cnt_prev));
		HAL_UART_Transmit(huart, str, strlen(str), 5);
		TIM6->CNT=0;
		cnt_prev=TIM6->CNT;
		i=0;
	}
	else i++;
	 */

#ifdef LS_DEBUG
		lsReadTick+=2000;
#endif

}


