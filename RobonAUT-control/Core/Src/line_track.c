/*
 * line_track.c
 *
 *  Created on: 2022. nov. 29.
 *      Author: Zsombesz
 */

#include "line_track.h"
#include "configF4.h"
#include "dc_driver.h"
#include "main.h"
#include "control.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

uint8_t txBuf[]={CMD_READ_SKILL_FORWARD};
uint8_t rxBuf[10];
uint8_t ignore=0; //ha a node jelölés miatt látunk több vonalat, akkor azt ne kezeljük útelágazásnak (ignoráljuk)

uint16_t boostCnt;


uint8_t G0_Read_Fast(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg)
{

	uint8_t state=0;
	txBuf[0]=CMD_READ_FAST;
	HAL_UART_Transmit(huart_stm, txBuf,1, 2);
	state=HAL_UART_Receive(huart_stm, rxBuf, 8, 4);
	motorEnLineOk=1; //ha van akkor mehet a szabályozás
	if((state==HAL_OK)&&(rxBuf[0]==START_BYTE_FAST) && (rxBuf[7]==STOP_BYTE))//jöt adat a G0 tól és a keret is megfelelő
	{
		return 0;
	}
	else //nem jött szabályos adat a G0-tól
	{
		return 1;
	}
}

uint8_t G0_Read_Skill(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg, uint8_t command)
{
	uint8_t state=0;
	txBuf[0]=command;
	HAL_UART_Transmit(huart_stm, txBuf,1, 2);
	state = HAL_UART_Receive(huart_stm, rxBuf, 10, 4);
	motorEnLineOk=1; //ha van akkor mehet a szabályozás
	if((state==HAL_OK)&&(rxBuf[0]==START_BYTE_SKILL_FORWARD || rxBuf[0]==START_BYTE_SKILL_REVERSE) && (rxBuf[9]==STOP_BYTE))//jöt adat a G0 tól és a keret is megfelelő
	{
		return 0;
	}
	else //nem jött szabályos adat a G0-tól
	{
		return 1;
	}
}


void Line_Track_Task(UART_HandleTypeDef *huart_stm,UART_HandleTypeDef *huart_debugg, uint32_t tick, uint32_t period)
{
	static uint32_t line_track_task_tick=4;
	static int32_t ccr = SERVO_FRONT_CCR_MIDDLE;
	static float PHI;
	static float gamma=0;
	static uint32_t ccr_rear_prev=0;
	static uint32_t tick_prev=0;

	if(line_track_task_tick>tick) return;
	line_track_task_tick = tick + period;

	if(mode == SKILL)
	{
		if(orientation==FORWARD)
		{
			if(G0_Read_Skill(huart_stm, huart_debugg, CMD_READ_SKILL_FORWARD))return;

			uint8_t tmp=Lane_Changer(huart_debugg,tick);
			if(v_control==NORMAL_VEL)v_ref=1100;
			else if(v_control==SLOW_DOWN)v_ref=600;
			else if(v_control==STOP)v_ref=-199;
			else if(v_control==SPEED_UP)v_ref=1200;
			else if(v_control==SLEEP)v_ref=0;
			if(tmp)return;

			if(v_control==SPEED_UP)Detect_Node5(huart_debugg, tick);
			else Detect_Node4(huart_debugg, tick);
			if (LINE_CNT<1 || LINE_CNT > 4) return;//ha nincs vonal a kocsi alatt
			gamma = Skill_Mode(huart_debugg, 0.004, 0.004, tick); //kD 4ms -es futáshoz hangolva

			//ELSŐSZERVÓ ELŐREMENETBEN
			PHI = atan((L/(L+D_FRONT))*tan(gamma));
			ccr = (uint16_t)(SERVO_M * PHI + SERVO_FRONT_CCR_MIDDLE);//balra kanyarodás
			//ne feszítsük neki a mechanikai határnak a szervót
			FRONT_CCR(ccr);

			//HÁTSÓSZERVÓ ELŐREMENETBEN
			PHI = atan((L/(L+D_REAR))*tan(gamma))/3;
			ccr = (uint16_t)(SERVO_M * PHI + SERVO_REAR_CCR_MIDDLE);//balra kanyarodás
			REAR_CCR(ccr);
		}
		else if(orientation==REVERSE)//TOLATÁS
		{
			if(G0_Read_Skill(huart_stm, huart_debugg, CMD_READ_SKILL_REVERSE))return;

			uint8_t tmp=Lane_Changer(huart_debugg,tick);
			if(v_control==NORMAL_VEL)v_ref=-1100;
			else if(v_control==SLOW_DOWN)v_ref=-600;
			else if(v_control==STOP)v_ref=199;
			else if(v_control==SPEED_UP)v_ref=-1200;
			else if(v_control==SLEEP)v_ref=0;
			if(tmp)return;
			if(v_control==SPEED_UP)Detect_Node5(huart_debugg, tick);
			else Detect_Node4(huart_debugg, tick);
			if (LINE_CNT<1 || LINE_CNT > 4) return;//ha nincs vonal a kocsi alatt
			gamma = Skill_Mode(huart_debugg, 0.003, 0.032, tick);

			//HÁTSÓ SZERVÓ HÁTRAMENETBEN
			PHI = atan((L/(L+D_REAR))*tan(gamma));////////////////////kiszámolni kézzel
			ccr = (uint16_t)(SERVO_M * PHI + SERVO_REAR_CCR_MIDDLE);
			REAR_CCR(ccr);

			//ELSŐSZERVÓ HÁTRAMENETBEN
			PHI = atan((L/(L+D_FRONT))*tan(gamma))/3;
			ccr = (uint16_t)(SERVO_M * PHI + SERVO_FRONT_CCR_MIDDLE);//balra kanyarodás
			FRONT_CCR(ccr);
		}
	}
	/*****Gyorsasági pálya üzemmód******/
	else if(mode == FAST)
	{
		static uint8_t fast_mode_state=SC_MODE;
		///////////////////////////////////////////////////////////////////////////////////////

		if(G0_Read_Fast(huart_stm, huart_debugg)) return; //ha sikertelen az olvasás a G0 ból akkor nincs értelme az egésznek
		if((LINE_CNT<1 || LINE_CNT > 3) && !leaveLineEnabled) return;//ha nincs vonal a kocsi alatt
		gamma = Fast_Mode(huart_debugg, &fast_mode_state, tick);
		PHI = atan((L/(L+D_FRONT))*tan(gamma));

		if(fast_mode_state==FREERUN_MODE)
		{
			if(v>2200)//egyenes
			{
				ccr = (uint16_t)(-SERVO_M_STRAIGHT * PHI + SERVO_FRONT_CCR_MIDDLE);
				FRONT_CCR(ccr);
				REAR_CCR(SERVO_REAR_CCR_MIDDLE);
			}
			else//kanyar
			{
				//Első szervó
				ccr =(uint16_t)(-SERVO_M_CORNER * PHI + SERVO_FRONT_CCR_MIDDLE);
				FRONT_CCR(ccr);
				//Hátsó szervó
				PHI/= 2;
				ccr = (uint16_t)(-SERVO_M_CORNER * PHI + SERVO_REAR_CCR_MIDDLE);
				REAR_CCR(ccr);
			}
		}
		else if(fast_mode_state==SC_MODE)
		{
			//Első szervó
			ccr = (uint16_t)(-SERVO_M_SC * PHI + SERVO_FRONT_CCR_MIDDLE);
			FRONT_CCR(ccr);

			//Hátsó szervó
			PHI/= 3;
			ccr = (uint16_t)(-SERVO_M_SC * PHI + SERVO_REAR_CCR_MIDDLE);
			REAR_CCR(ccr);
		}
		else if(fast_mode_state==OVERTAKE_MODE)
		{
			static uint8_t overtake_state=0;
			static uint32_t t_stamp_overtake=0;
			if(overtake_state==0)
			{
				v_ref=2000;
				t_stamp_overtake=tick;
				leaveLineEnabled=1;
				TIM2->CCR1=SERVO_FRONT_CCR_MIDDLE+170;
				TIM1->CCR4=SERVO_REAR_CCR_MIDDLE-160;
				overtake_state=1;
			}
			else if(overtake_state==1 && (tick-t_stamp_overtake)>1900)
			{
				v_ref=3000;
				LED_Y(1);
				//FRONT_CCR(SERVO_FRONT_CCR_MIDDLE-50);
				TIM2->CCR1=SERVO_FRONT_CCR_MIDDLE-50;
				TIM1->CCR4=SERVO_REAR_CCR_MIDDLE+20;
				//REAR_CCR(SERVO_REAR_CCR_MIDDLE+50);
				overtake_state=2;
			}
			else if(overtake_state==2 && LINE_CNT>0)
			{
				v_ref=1200;
				overtake_state=0;
				fast_mode_state=FREERUN_MODE;
				leaveLineEnabled=0;
				LED_Y(0);
			}
		}
	}

	tick_prev=tick;
}

float Fast_Mode(UART_HandleTypeDef *huart_debugg,uint8_t* state_pointer, uint32_t t)
{
	static uint32_t t_prev=0;
	static uint32_t t_stamp_boost=0;
	static uint32_t t_stamp_brake_end=0;
	static uint32_t t_overtake=0;
	static uint8_t ot_delay=0;

	static uint8_t boostOrBrake=1;
	static uint8_t lineCnt_prev=1;
	static uint8_t index=0;

	static float s_brake=0;
	static float ds[]={1000,1000,1000,1000,1000,1000};
	static int straightSpeed[]	={	SC_MODE,OVERTAKE_MODE,4500,		//1.kör
									5000,4500,5000,4500,			//2.kör
									2500,2500,OVERTAKE_MODE,4000,	//3.kör
									5000,4500,5000,5000,			//4.kör
									5000,5000,5000,5500,			//5.kör
									5500,6000,6000,6000,			//6.kör
									6000,-1};

	static int cornerSpeed[]	={	1600,1600,1400,1600,			//1.kör
									1600,1550,1550,1500,			//2.kör
									1200,1200,1400,1500,			//3.kör
									1600,1550,1550,1600,			//4.kör
									1650,1600,1600,1650,			//5.kör
									1700,1650,1650,1700,			//6.kör
									1600,-1};							//levezető ív

	static float k_p = K_P_200;
	static float k_delta = K_DELTA_200;
	static float kD=K_D;
	static float x_elso=0;
	static float x_elso_prev=0;
	static float x_hatso;
	static double delta;
	static float gamma;

	uint8_t state = *state_pointer;

	if(state==OVERTAKE_MODE)return 0;
/**/
	//BOOST detect
	if(LINE_CNT != lineCnt_prev && (LINE_CNT==1 || LINE_CNT==3)  && boostOrBrake==1) //ha változik az alattunk lévő vonalak száma 1 és 3 közt
	{
		ds[index]=fabs(v)*(t-t_stamp_boost)/1000;
		float s_boost = ds[0]+ds[1]+ds[2]+ds[3]+ds[4]+ds[5];
		if(s_boost>250.0 && s_boost<1200.0) // ha 25 és 80 cm közt bekövetkezik 8 vonalszámváltás
		{
			//LED_B(1);
			LED_B_TOGGLE;
			boostOrBrake=2;
			if(straightSpeed[boostCnt]==SC_MODE)state=SC_MODE;
			else if(straightSpeed[boostCnt]==OVERTAKE_MODE && !ot_delay)
			{
				ot_delay=1;
				t_overtake=t;

			}
			else if(straightSpeed[boostCnt]==-1) motorEnLineOk=0;
			else //FREERUN MODE
			{
				state=FREERUN_MODE;
				v_ref=straightSpeed[boostCnt];
			}
			boostCnt++;
		}
		index++;
		if(index>5) index=0;
		t_stamp_boost = t;
	}
	lineCnt_prev = LINE_CNT; //az előző értéket a jelenlegihez hangoljuk

	//felzárkózás 1. előzés után
	if(boostCnt>7 &&  boostCnt<10 && state==FREERUN_MODE && rxBuf[4]==2)state=SC_MODE;

	if(ot_delay && (t-t_overtake)>1300)
	{
		*state_pointer=OVERTAKE_MODE;
		ot_delay=0;
		return 0;
	}

	//BRAKING detect -> erre csak gyors üzemmódban van szükség
	if(LINE_CNT > 1) //ha 3 vonalat érzékelünk
	{
		s_brake += fabs(v)*(t-t_prev)/1000;
		if(s_brake>300) //ha már legalább 30cm óta folyamatosan 3 vonal van alattunk
		{
			//LED_G_TOGGLE;
			if(state == FREERUN_MODE)
			{
				if(cornerSpeed[boostCnt]==-1) motorEnLineOk=0;
				else v_ref = cornerSpeed[boostCnt];

			}
			t_stamp_brake_end=t;
			boostOrBrake=3;
		}
	}
	else //ha 1 vonalat érzékelünk
	{
		s_brake=0;
	}
	t_prev=t;

	float tmp;
	if(fabs(v)>300)
	{
		tmp=1000000/fabs(v);
	}
	else tmp=1000;
	if(boostOrBrake==3 && (t-t_stamp_brake_end)>tmp)
	{
		boostOrBrake=1;
	}

	/*****SC üzemmód******/
	if(state==SC_MODE)
	{
		uint16_t dist=(((uint16_t)rxBuf[5])<<8) | ((uint16_t)rxBuf[6]);
		if(dist>833 || rxBuf[4]==0)
		{
			v_ref=1600; //ha tul messze vana  SC vagy érvénytelen az olvasás
			LED_Y(1);
		}
		else
		{
			v_ref = 3*(dist-300);
			LED_Y(0);
		}
	}

	x_elso=(float)rxBuf[2]*204/248.0-102;//248
	x_hatso=(float)rxBuf[3]*204/248.0-102; //244
	delta=atan((double)(x_elso-x_hatso)/L_SENSOR);
	/**/
	//szabályozóparaméterek ujraszámolása az aktuális sebesség alapján
	if(state==SC_MODE)
	{
		if(fabs(v)>200)k_p= -3/fabs(v);
		else k_p = -0.004;
		kD =-0.03; //-0.004;
		k_delta = 0;
	}

	else //FREERUN modes
	{
		if(v>150 || v<-150)
		{
			if(v<2200) //kanyar
			{
				k_p = -0.0026;//-L/(v*v)*S1MULTS2_SLOW;
				k_delta = 0;//L/v*(S1ADDS2_SLOW-v*k_p);
				kD=-0.07;//-0.06
			}
			else //egyenes
			{
				k_p = -L*S1MULTS2_FAST/(v*v)*1.1;
				k_delta = L*(S1ADDS2_FAST-v*k_p)*0.8/v;
				kD=-0.075;
			}
		}
		else
		{
			k_p=K_P_200;
			k_delta=K_DELTA_200;
			kD=-0.06;
		}
	}
	gamma = -k_p * x_elso -k_delta * delta - kD * (x_elso-x_elso_prev);
	x_elso_prev = x_elso;

	*state_pointer=state;
	return gamma;
}

float Skill_Mode(UART_HandleTypeDef *huart_debugg, float kP, float kD, uint32_t t)
{
	static uint32_t t_prev=0;
	int byte=0;
	static int byte_prev=0;
	int delta_byte;
	float p=0;
	static float p_prev=0;
	static uint8_t estuary=ESTUARY_MODE_INIT;
	static float gamma;
	int i;
	static int tmp1,tmp2;
	static int byte_middle;
	static uint8_t middleLineState=1;

/*	uint8_t str[40];
	sprintf(str,"%d,  %d,  %d,  %d,  %d\n\r",rxBuf[1],rxBuf[2],rxBuf[3],rxBuf[4],rxBuf[5]);
	HAL_UART_Transmit(huart_debugg, str, strlen(str), 10);
*/

	if(LINE_CNT>3 || ignore)//ha éppen node-on vagyunk, akkor az átlagot követjük
	{
		byte=0;
		for(i=0;i<LINE_CNT;i++)
		{
			byte += rxBuf[i+2];
		}

		if(LINE_CNT) byte /= LINE_CNT;
	}
	else if((path==LEFT && orientation==FORWARD) || (path==RIGHT && orientation==REVERSE))
	{
		byte = LINE1; //az első vonalt kell követni
		delta_byte=abs((int)byte-byte_prev);
		/*
		if((delta_byte>ESTUARY_TH && estuary!=ESTUARY_MODE_INIT)|| estuary==ESTUARY_MODE_ON) //torkolatkompenzálás
		{

			if(estuary==ESTUARY_MODE_OFF)t_prev=t;//ha most kapcsoltuk be a torkolatkompenzálást, akkor mostantól mérjük az eltelt időt
			if((t-t_prev)>ESTURAY_TIMEOUT)//400ms után mindenképpen kilépünk a kompenzálásból
			{
				estuary=ESTUARY_MODE_OFF; //ha letelt a timeout kilépünk a kompenzálásból
				LED_G(0);
			}
			else //ha még nem telt le az timout idő
			{
				byte = rxBuf[1+LINE_CNT]; //ilyenkor az utolsó vonalat nézzük az első helyett
				estuary=ESTUARY_MODE_ON; //öntartás
				LED_G(1);
			}
		}
		else if(delta_byte<ESTUARY_EXIT && estuary==ESTUARY_MODE_ON) //ha már eléggé összeszűkült a torkolat, akkor nem kell kompenzálni
		{
			estuary=ESTUARY_MODE_OFF;
			LED_G(0);
		}
*/
	}
	else if((path==RIGHT && orientation==FORWARD) || (path==LEFT && orientation==REVERSE))
	{
		byte = rxBuf[1+LINE_CNT];//az utolsó vonalat kell követni
		delta_byte=abs((int)byte-byte_prev);
		/*
		if((delta_byte>ESTUARY_TH && estuary!=ESTUARY_MODE_INIT)|| estuary==ESTUARY_MODE_ON) //torkolatkompenzálás
		{
				if(estuary==ESTUARY_MODE_OFF)t_prev=t;//ha most kapcsoltuk be a torkolatkompenzálást, akkor mostantól mérjük az eltelt időt
				if((t-t_prev)>ESTURAY_TIMEOUT)//400ms után mindenképpen kilépünk a kompenzálásból
				{
					estuary=ESTUARY_MODE_OFF; //ha letelt a timeout kilépünk a kompenzálásból
					LED_G(0);
				}
				else //ha még nem telt le az idő
				{
					byte = rxBuf[2]; //ilyenkor az első vonalat nézzük az utolsó helyett
					estuary=ESTUARY_MODE_ON; //öntartás
					LED_G(1);
				}
		}
		else if(delta_byte<ESTUARY_EXIT && estuary==ESTUARY_MODE_ON) //ha már eléggé összeszűkült a torkolat, akkor nem kell kompenzálni
		{
			estuary=ESTUARY_MODE_OFF;
			LED_G(0);
		}*/
	}

	else if(path==MIDDLE)
	{
		if(LINE_CNT==1)
		{
			byte = byte_middle = LINE1;//ha csak 1 vonal van, akkor azt követjük
			middleLineState=1;
		}
		else if(LINE_CNT==3)//ha 3 vonal van
		{
			byte = byte_middle = LINE2;//a középsőt követjük
			//folyamatosan nézzük, hogy az 1. és 3.vonal milyen messze van a vonalszenor középontjától
			middleLineState=1;
		}
		else if(LINE_CNT==2)//ha 2 vonal van, az azt jelenti, hogy az elágazás már annyira szétgáazott, hogy csak 2-t látunk a 3 vonalból
		{
			if(middleLineState==1)
			{
				tmp1=abs(byte_middle-LINE1);
				tmp2=abs(byte_middle-LINE2);
				middleLineState=2;
			}
			if(middleLineState==2)
			{
				if(tmp1<tmp2) byte = LINE1; //ha a jobboldali vonalat veszítettük el
				else byte = LINE2; //ha a baloldali vonalat veszítettük el
			}
		}
	}
	if(estuary==ESTUARY_MODE_INIT)estuary=ESTUARY_MODE_OFF;
	//p = (float)byte * 204/248.0-102;
	p = (float)byte * 204/248.0-102;
	gamma = -kP * p  - kD*(p-p_prev);
	p_prev = p;
	byte_prev=byte;

	return gamma;
}

void Detect_Node2(UART_HandleTypeDef *huart_debugg, uint32_t t)
{
	static uint8_t detect_node_state=STEADY;
	static uint8_t val=0;
	static uint32_t dt=0;
	static uint32_t t_prev=0;
/*	static uint8_t str[30];


	sprintf(str,"%2d\r\n",rxBuf[1]);
	HAL_UART_Transmit(huart_debugg, str, 4, 10);
*/
	if (LINE_CNT<1 || LINE_CNT > 4)
	{
		t_prev=t;
		detect_node_state=STEADY;
		val=4;
	}
	switch(detect_node_state)
	{
	case STEADY: //többször futó állapot
		if(rxBuf[1]==4)
		{
			dt = t-t_prev;//mennyi ideje van alattunk 4 vonal
			if(dt > TH_MIN(70))
			{
				ignore=1;
				detect_node_state=QUAD_LINE_DETECTED;
			}
			val=0;
		}
		else
		{
			t_prev=t;
			ignore=0;
		}
		break;

	case QUAD_LINE_DETECTED:
		if(rxBuf[1]==2 && !val) val=1; //horizontal node lehetséges
		else if(rxBuf[1]==4 && val==1) val=2; //horizontal node tuti

		dt=t-t_prev;
		if(dt> TH(170) && rxBuf[1]==4 && !val)val=3;

		if(dt> (TH(200)+100))
		{
			if(val==3 && rxBuf[1]<4)nodeDetected=1;//vert node
			else if(val==2 && rxBuf[1]<4)
			{
				nodeDetected=1; //horizont node
			}
			detect_node_state=STEADY;
			val=0;
		}
		break;
	}
}

void Detect_Node3(UART_HandleTypeDef *huart_debugg, uint32_t t)
{
	static uint32_t dt=0;
	static uint32_t t_prev=0;
	static uint8_t flag=0;

	dt=t-t_prev;
	if(LINE_CNT==4 && dt> 1500)
	{
		flag=1;
		//nodeDetected=1;
		/*
		if(path==0)path=2;
		else if(path==2)path=0;*/
		ignore=1;

		t_prev=t;
	}
	else if(flag==1 && dt>300)
	{
		flag=0;
		nodeDetected=1;
	}

	if(ignore && dt>200)
	{
		ignore=0;
	}
}

void Detect_Node4(UART_HandleTypeDef *huart_debugg, uint32_t t)
{

	static uint32_t t_dont_detect=0;
	static uint32_t t_prev=0;
	static uint32_t t_stamp=0;
	static uint8_t detect_node_state=0;
	static float s=0;

	if(LINE_CNT==4 && !detect_node_state)
	{
		s=0;
		detect_node_state=1;//innentől mérünk
		ignore=1;
		t_stamp=t;

	}
	else if(LINE_CNT==4 && detect_node_state)
	{
		s+=fabs(v)*(t-t_prev)/1000;
	}
	if((t-t_stamp)>150 && detect_node_state)
	{
		detect_node_state=0;
		ignore=0;
		if(s>40 && (t-t_dont_detect)>250)//horizontal node
		{
			nodeDetected=1; //horizont node
			t_dont_detect=t;
			//LED_B_TOGGLE;
		}
	}
	t_prev=t;
}

void Detect_Node5(UART_HandleTypeDef *huart_debugg, uint32_t t)
{
	static uint32_t t_dont_detect=0;
	static uint32_t t_prev=0;
	static uint32_t t_stamp=0;
	static uint8_t detect_node_state=0;
	static float s=0;

	if(LINE_CNT==4 && !detect_node_state)
	{
		s=0;
		detect_node_state=1;//innentől mérünk
		ignore=1;
		t_stamp=t;

	}
	else if(LINE_CNT==4 && detect_node_state)
	{
		s+=fabs(v)*(t-t_prev)/1000;
	}
	if((t-t_stamp)>130 && detect_node_state)
	{
		detect_node_state=0;
		ignore=0;
		if(s>40 && (t-t_dont_detect)>250)//horizontal node
		{
			nodeDetected=1; //horizont node
			t_dont_detect=t;
			//LED_B_TOGGLE;
		}
	}
	t_prev=t;
}

uint8_t Lane_Changer(UART_HandleTypeDef *huart_debugg,uint32_t t)
{
	static uint32_t t_prev=0;
	static uint32_t t_stamp=0;
	static uint32_t timeout=0;
	static uint8_t lineCnt_prev=1;
	static float s=0;
	static int i=0;
	static uint32_t dt[5]={1000,1000,1000,1000,1000,1000,1000};

	if(laneChange<2)return 0;
	if(LINE_CNT != lineCnt_prev && (LINE_CNT==1 || LINE_CNT==2) && laneChange==2) //ha változik az alattunk lévő vonalak száma
	{
		dt[i] = t - t_stamp;
		uint32_t sum=dt[0] + dt[1] + dt[2] + dt[3]+ dt[4]+dt[5]+dt[6];
		if((sum > 250) && (sum < 1000))//ha másfél másodpercen belül van8 váltás
		{
			s=0;
			laneChange=3;
		}
		i++;
		if(i>6) i=0;
		t_stamp = t;
	}
	else if(laneChange==3)
	{
		s+=fabs(v)*(t-t_prev)/1000;
		if(orientation==REVERSE)
		{
			v_control=SLOW_DOWN;
			FRONT_CCR(SERVO_FRONT_CCR_MIDDLE+140);
			REAR_CCR(SERVO_REAR_CCR_MIDDLE-160);
			timeout=2000;
			laneChange=4;
			t_stamp=t;
			return 1;
		}
		else if(orientation==FORWARD && s>1200)
		{
			v_control=SLOW_DOWN;
			FRONT_CCR(CCR_FRONT_MAX);
			REAR_CCR(CCR_REAR_MAX);
			timeout=3000;
			laneChange=4;
			t_stamp=t;
			return 1;
		}
	}
	if(laneChange==4)
	{
		LED_Y(0);
		v_control=SLOW_DOWN;
		if((t-t_stamp)>timeout && LINE_CNT>0)
		{
			ignore=1;
			laneChange=5;
			t_stamp=t;
			return 0;
		}
		else return 1;
	}
	if(laneChange==5 && (t-t_stamp)>2000)
	{

		TIM3->CCR1=474;
		TIM3->CCR2=524;
		B_NUCLEO_ISR(huart_debugg);
	}
	lineCnt_prev=LINE_CNT;
	t_prev=t;
	return 0;
}

