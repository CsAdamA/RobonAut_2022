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

uint16_t boostCnt=0;


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

			uint8_t tmp=Lane_Changer(tick);
			if(v_control==NORMAL_VEL)v_ref=1100;
			else if(v_control==SLOW_DOWN)v_ref=600;
			else if(v_control==STOP)v_ref=100;

			if(tmp)return;

			Detect_Node4(huart_debugg, tick);
			if (LINE_CNT<1 || LINE_CNT > 4) return;//ha nincs vonal a kocsi alatt
			gamma = Skill_Mode(huart_debugg, 0.004, 0.004, tick); //kD 4ms -es futáshoz hangolva

			//ELSŐSZERVÓ ELŐREMENETBEN
			PHI = atan((L/(L+D_FRONT))*tan(gamma));
			ccr = (uint16_t)(SERVO_M * PHI + SERVO_FRONT_CCR_MIDDLE);//balra kanyarodás
			if(ccr > CCR_FRONT_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_FRONT_MAX;
			}
			else if(ccr < CCR_FRONT_MIN)//egyik irányba se
			{
				ccr = CCR_FRONT_MIN;
			}
			TIM2->CCR1 = ccr;
			//HÁTSÓSZERVÓ ELŐREMENETBEN
			PHI = atan((L/(L+D_REAR))*tan(gamma))/3;
			ccr = (uint16_t)(SERVO_M * PHI + SERVO_REAR_CCR_MIDDLE);//balra kanyarodás
			if(ccr > CCR_REAR_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_REAR_MAX;
			}
			else if(ccr < CCR_REAR_MIN)//egyik irányba se
			{
				ccr = CCR_REAR_MIN;
			}
			TIM1->CCR4 = ccr;
		}
		else if(orientation==REVERSE)//TOLATÁS
		{
			if(G0_Read_Skill(huart_stm, huart_debugg, CMD_READ_SKILL_REVERSE))return;

			uint8_t tmp=Lane_Changer(tick);
			if(v_control==NORMAL_VEL)v_ref=-1100;
			else if(v_control==SLOW_DOWN)v_ref=-600;
			else if(v_control==STOP)v_ref=-100;

			if(tmp)return;

			Detect_Node4(huart_debugg, tick);
			if (LINE_CNT<1 || LINE_CNT > 4) return;//ha nincs vonal a kocsi alatt
			gamma = Skill_Mode(huart_debugg, 0.003, 0.032, tick);

			//HÁTSÓ SZERVÓ HÁTRAMENETBEN
			PHI = atan((L/(L+D_REAR))*tan(gamma));////////////////////kiszámolni kézzel
			ccr = (uint16_t)(SERVO_M * PHI + SERVO_REAR_CCR_MIDDLE);
			if(ccr > CCR_REAR_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_REAR_MAX;
			}
			else if(ccr < CCR_REAR_MIN)//egyik irányba se
			{
				ccr = CCR_REAR_MIN;
			}
			TIM1->CCR4 = ccr;
			//ELSŐSZERVÓ HÁTRAMENETBEN
			PHI = atan((L/(L+D_FRONT))*tan(gamma))/3;
			ccr = (uint16_t)(SERVO_M * PHI + SERVO_FRONT_CCR_MIDDLE);//balra kanyarodás
			if(ccr > CCR_FRONT_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_FRONT_MAX;
			}
			else if(ccr < CCR_FRONT_MIN)//egyik irányba se
			{
				ccr = CCR_FRONT_MIN;
			}
			TIM2->CCR1 = ccr;
		}

	}
	/*****Gyorsasági pálya üzemmód******/
	else if(mode == FAST)
	{
		static uint8_t fast_mode_state=SC_MODE;

		if(G0_Read_Fast(huart_stm, huart_debugg)) return; //ha sikertelen az olvasás a G0 ból akkor nincs értelme az egésznek
		if (LINE_CNT<1 || LINE_CNT > 3) return;//ha nincs vonal a kocsi alatt
		gamma = Fast_Mode(huart_debugg, &fast_mode_state, tick);
		PHI = atan((L/(L+D_FRONT))*tan(gamma));

		if(fast_mode_state==FREERUN_MODE)
		{
			if(v>2000) ccr = (uint16_t)(-SERVO_M_STRAIGHT * PHI + SERVO_FRONT_CCR_MIDDLE);
			else ccr =(uint16_t)(-SERVO_M_CORNER * PHI + SERVO_FRONT_CCR_MIDDLE);
			if(ccr > CCR_FRONT_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_FRONT_MAX;
			}
			else if(ccr < CCR_FRONT_MIN)//egyik irányba se
			{
				ccr = CCR_FRONT_MIN;
			}
			TIM2->CCR1 = ccr;
			if(ccr_rear_prev!=SERVO_REAR_CCR_MIDDLE) TIM1->CCR4 = SERVO_REAR_CCR_MIDDLE;
			ccr_rear_prev=SERVO_REAR_CCR_MIDDLE;
		}
		else //fast_mode_state==SC_MODE
		{
			//első szervó
			ccr = (uint16_t)(-SERVO_M_SC * PHI + SERVO_FRONT_CCR_MIDDLE);
			if(ccr > CCR_FRONT_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_FRONT_MAX;
			}
			else if(ccr < CCR_FRONT_MIN)//egyik irányba se
			{
				ccr = CCR_FRONT_MIN;
			}
			TIM2->CCR1 = ccr;
			//Hátsó szervó
			PHI/= -3;
			ccr = (uint16_t)(-SERVO_M_SC * PHI + SERVO_REAR_CCR_MIDDLE);
			if(ccr > CCR_REAR_MAX)//ne feszítsük neki a mechanikai határnak a szervót
			{
				ccr = CCR_REAR_MAX;
			}
			else if(ccr < CCR_REAR_MIN)//egyik irányba se
			{
				ccr = CCR_REAR_MIN;
			}
			TIM1->CCR4=ccr;
		}
	}

	tick_prev=tick;
}

float Fast_Mode(UART_HandleTypeDef *huart_debugg,uint8_t* state_pointer, uint32_t t)
{
	static float s_brake=0;
	static float 	ds[]	={1000,1000,1000,1000,1000,1000,1000,1000};
	static uint32_t t_prev=0;
	static uint32_t t_stamp=0;

	static uint8_t lineCnt_prev=1;
	static uint8_t index=0;

	static float k_p = K_P_200;
	static float k_delta = K_DELTA_200;
	static float x_elso=0;
	static float x_elso_prev=0;
	static float x_hatso;
	static float delta;
	static float gamma;

	static float kD=K_D;

	uint8_t state = *state_pointer;
/**/
	//BOOST detect
	if(LINE_CNT != lineCnt_prev && (LINE_CNT==1 || LINE_CNT==3)) //ha változik az alattunk lévő vonalak száma
	{
		ds[index]=fabs(v)*(t-t_stamp)/1000;
		float s_boost = ds[0]+ds[1]+ds[2]+ds[3]+ds[4]+ds[5]+ds[6]+ds[7];
		if(s_boost>300.0 && s_boost<800.0) // ha 70 és 80 cm közt bekövetkezik 8 vonalszámváltás
		{
			boostCnt++;
			if(state==FREERUN_MODE)
			{
				v_ref = 5000;
				LED_B(1);
			}
			else LED_B_TOGGLE;
		}

		index++;
		if(index>7) index=0;
		t_stamp = t;
	}
	lineCnt_prev = LINE_CNT; //az előző értéket a jelenlegihez hangoljuk

	//BRAKING detect -> erre csak gyors üzemmódban van szükség
	if(LINE_CNT > 1 && state==FREERUN_MODE) //ha 3 vonalat érzékelünk
	{
		s_brake += fabs(v)*(t-t_prev)/1000;
		if(s_brake>300) //ha már legalább 30cm óta folyamatosan 3 vonal van alattunk
		{
			v_ref = 1500;
			LED_B(0);
		}
	}
	else //ha 1 vonalat érzékelünk
	{
		s_brake=0;
	}
	t_prev=t;

	/////////////////////////////////////////////////////////////////////////////////////////

	/*****SC üzemmód******/
	if(state==SC_MODE)
	{
		uint32_t dist=(((uint16_t)rxBuf[5])<<8) | ((uint16_t)rxBuf[6]);
		if(dist>1000 || rxBuf[4]) v_ref=1500; //ha tul messze vana  SC vagy érvénytelen az olvasás
		else v_ref = 2*(float)dist-500;
	}

	x_elso=(float)rxBuf[2]*204/255.0-102;//248
	x_hatso=(float)rxBuf[3]*204/255.0-102; //244
	delta=atan((float)(x_elso-x_hatso)/L_SENSOR);
	/**/
	//szabályozóparaméterek ujraszámolása az aktuális sebesség alapján
	if(state==SC_MODE)
	{
		k_p=0.004;
		kD=0.004;
		k_delta=0;
	}

	else //freerun modes
	{
		if(v>150 || v<-150)
		{
			if(v<2000)
			{
				k_p = -L/(v*v)*S1MULTS2_SLOW;
				k_delta = L/v*(S1ADDS2_SLOW-v*k_p);
				kD=-0.06;
				//kD=0;
			}
			else
			{
				k_p = -L/(v*v)*S1MULTS2_SLOW;
				k_delta = L/v*(S1ADDS2_SLOW-v*k_p);
				kD=-0.05;
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
		/**/
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

	}
	else if((path==RIGHT && orientation==FORWARD) || (path==LEFT && orientation==REVERSE))
	{
		byte = rxBuf[1+LINE_CNT];//az utolsó vonalat kell követni
		delta_byte=abs((int)byte-byte_prev);
		/**/
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
		}

	}

	else if(path==MIDDLE)
	{
		if(LINE_CNT==1)byte = LINE1;//ha csak 1 vonal van, akkor azt követjük
		else if(LINE_CNT==3)//ha 3 vonal van
		{
			byte = rxBuf[3];//a középsőt követjük
			//folyamatosan nézzük, hogy az 1. és 3.vonal milyen messze van a vonalszenor középontjától
			tmp1=abs((int)LINE1-123);
			tmp2=abs((int)LINE3-123);
		}
		else if(LINE_CNT==2)//ha 2 vonal van, az azt jelenti, hogy az elágazás már annyira szétgáazott, hogy csak 2-t látunk a 3 vonalból
		{
			if(tmp1<tmp2) byte = LINE1; //ha a jobboldali vonalat veszítettük el
			else byte = LINE2; //ha a baloldali vonalat veszítettük el
		}
	}
	if(estuary==ESTUARY_MODE_INIT)estuary=ESTUARY_MODE_OFF;
	//p = (float)byte * 204/248.0-102;
	p = (float)byte * 204/255.0-102;
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
	if((t-t_stamp)>230 && detect_node_state)
	{
		detect_node_state=0;
		ignore=0;
		/*
		if(s>140)//vertical node
		{
			nodeDetected=1; //horizont node

		}
		else*/ if(s>50)//horizontal node
		{
			nodeDetected=1; //horizont node
		}
	}
	t_prev=t;
}

uint8_t Lane_Changer(uint32_t t)
{
	static uint32_t t_prev=0;
	static uint32_t t_stamp=0;
	static uint32_t timeout=0;
	static uint8_t lineCnt_prev=1;
	static float s=0;
	static int i=0;
	static uint32_t dt[5]={1000,1000,1000,1000,1000};

	if(laneChange<2)return 0;
	if(LINE_CNT != lineCnt_prev && (LINE_CNT==1 || LINE_CNT==2) && laneChange==2) //ha változik az alattunk lévő vonalak száma
	{
		dt[i] = t - t_stamp;
		uint32_t sum=dt[0] + dt[1] + dt[2] + dt[3]+ dt[4];
		if((sum > 250) && (sum < 1000))//ha másfél másodpercen belül van8 váltás
		{
			s=0;
			laneChange=3;
		}
		i++;
		if(i>4) i=0;
		t_stamp = t;
	}
	else if(laneChange==3)
	{
		s+=fabs(v)*(t-t_prev)/1000;
		if(orientation==FORWARD)
		{
			TIM2->CCR1=CCR_FRONT_MAX;
			TIM1->CCR4=CCR_REAR_MIN;
			timeout=1000;
			laneChange=4;
			t_stamp=t;
			return 1;
		}
		else if(orientation==REVERSE && s>2000)
		{
			TIM2->CCR1=CCR_FRONT_MIN;
			TIM1->CCR4=CCR_REAR_MIN;
			timeout=3000;
			laneChange=4;
			t_stamp=t;
			return 1;
		}
	}
	else if(laneChange==4)
	{
		LED_Y(0);
		v_control=SLOW_DOWN;
		if((t-t_stamp)>timeout && LINE_CNT>0)
		{
			ignore=1;
			return 0;
		}
		else return 1;
	}
	lineCnt_prev=LINE_CNT;
	t_prev=t;
	return 0;
}

