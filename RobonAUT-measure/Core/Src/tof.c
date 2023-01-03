/*
 * tof.c
 *
 *  Created on: 2022. nov. 22.
 *      Author: Zsombesz
 */

#include "tof.h"
#include <string.h>
#include "configG0.h"

uint8_t tofData[3];


VL53L1_RangingMeasurementData_t RangingData;
VL53L1_Dev_t vl53l1_f;
VL53L1_DEV Dev_f=&vl53l1_f;

void Tof_Init(I2C_HandleTypeDef *hi2c, uint16_t period)
{
	Dev_f->I2cHandle=hi2c;
	Dev_f->I2cDevAddr=0x52;
	// initialize vl53l1x communication parameters
	XSHUT1(1);
	HAL_Delay( 2 );
	// VL53L1X Initialization
	VL53L1_WaitDeviceBooted( Dev_f );
	VL53L1_DataInit( Dev_f );
	VL53L1_StaticInit( Dev_f );
	VL53L1_SetDistanceMode( Dev_f, VL53L1_DISTANCEMODE_MEDIUM ); //4 méteres méreés
	VL53L1_SetMeasurementTimingBudgetMicroSeconds( Dev_f, 20000 ); //legalább 20 ms kell a szenzornak hogy mérést produkáljon
	VL53L1_SetInterMeasurementPeriodMilliSeconds( Dev_f, period); //20ms onként mér távolságot a szenzo
	VL53L1_StartMeasurement( Dev_f );
	tofData[0]=tofData[1]=tofData[2]=0;
}

void Tof_Task(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{
	static uint32_t tof_task_tick=0;
	uint8_t status=0;
	uint8_t isReady=0;

#ifdef TOF_DEBUG
	char str[30];
#endif

	if(tof_task_tick>tick) return;
	tof_task_tick= tick+period;

	status = VL53L1_GetMeasurementDataReady(Dev_f, &isReady);
	if(!status)
	{
		if(isReady)
		{
			status = VL53L1_GetRangingMeasurementData(Dev_f, &RangingData);
			if(!status)
			{
				tofData[0]=1;
				tofData[1]=(uint8_t)(RangingData.RangeMilliMeter>>8);
				tofData[2]=(uint8_t)(RangingData.RangeMilliMeter & 0x00ff);
#ifdef TOF_DEBUG
				sprintf(str,"%d,%d,%.2f,%.2f\n\r", RangingData.RangeStatus,RangingData.RangeMilliMeter,
					(RangingData.SignalRateRtnMegaCps/65536.0),RangingData.AmbientRateRtnMegaCps/65336.0);
				HAL_UART_Transmit(huart, str, strlen(str), 20);
#endif
			}
		}
	}
	if(status)
	{
		tofData[0]=0;
#ifdef TOF_DEBUG
		sprintf(str,"No new data yet!\n\r");
		HAL_UART_Transmit(huart, str, (uint16_t)strlen(str), 20);
#endif
	}
#ifdef TOF_DEBUG
	tof_task_tick+=1000;
#endif
	VL53L1_ClearInterruptAndStartMeasurement(Dev_f);

}
