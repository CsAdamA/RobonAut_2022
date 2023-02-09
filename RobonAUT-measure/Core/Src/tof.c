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

//első középső szenzor
VL53L1_RangingMeasurementData_t RangingDataFront;
VL53L1_Dev_t vl53l1Front;
VL53L1_DEV DevFront=&vl53l1Front;

//baloldali szenzor
VL53L0X_RangingMeasurementData_t RangingDataLeft;
VL53L0X_Dev_t  vl53l0xLeft; // center module
VL53L0X_DEV    DevLeft = &vl53l0xLeft;

//jobboldali szenzor
VL53L0X_RangingMeasurementData_t RangingDataRight;
VL53L0X_Dev_t  vl53l0xRight; // center module
VL53L0X_DEV    DevRight = &vl53l0xRight;

uint32_t refSpadCountLeft;
uint8_t isApertureSpadsLeft;
uint8_t VhvSettingsLeft;
uint8_t PhaseCalLeft;

uint32_t refSpadCountRight;
uint8_t isApertureSpadsRight;
uint8_t VhvSettingsRight;
uint8_t PhaseCalRight;


void Tof_Init(I2C_HandleTypeDef *hi2c_front,I2C_HandleTypeDef *hi2c_sides, uint16_t period)
{
	tofData[0]=tofData[1]=tofData[2]=0;
	XSHUT1(0);
	XSHUT3(0);
	XSHUT4(0);
	HAL_Delay(10);

	//FRONT sensor init
	DevFront->I2cHandle=hi2c_front;
	DevFront->I2cDevAddr=ADDR_FRONT;
	DevFront->comms_speed_khz=400;
	XSHUT1(1);
	HAL_Delay(10);
	// VL53L1X Initialization
	//VL53L1_Set
	VL53L1_WaitDeviceBooted( DevFront );
	VL53L1_DataInit( DevFront );
	VL53L1_StaticInit( DevFront );
	HAL_Delay(50);
	VL53L1_SetDistanceMode( DevFront, VL53L1_DISTANCEMODE_MEDIUM ); //4 méteres méreés
	VL53L1_SetPresetMode(DevFront, VL53L1_PRESETMODE_LITE_RANGING);
	VL53L1_SetMeasurementTimingBudgetMicroSeconds( DevFront, 25000 ); //legalább 20 ms kell a szenzornak hogy mérést produkáljon
	VL53L1_SetInterMeasurementPeriodMilliSeconds( DevFront, 25); //20ms onként mér távolságot a szenzor
	VL53L1_StartMeasurement( DevFront );

	//LEFT sensor init
	DevLeft->I2cHandle=hi2c_sides;
	DevLeft->I2cDevAddr=0x52;
	DevLeft->comms_speed_khz=400;
	XSHUT4(1);
	HAL_Delay(10);
	VL53L0X_WaitDeviceBooted( DevLeft );
	VL53L0X_DataInit( DevLeft );
	VL53L0X_StaticInit( DevLeft );
	HAL_Delay(50);
	VL53L0X_PerformRefCalibration(DevLeft, &VhvSettingsLeft, &PhaseCalLeft);
	VL53L0X_PerformRefSpadManagement(DevLeft, &refSpadCountLeft, &isApertureSpadsLeft);
	VL53L0X_SetDeviceMode(DevLeft,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

	VL53L0X_SetLimitCheckEnable(DevLeft, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(DevLeft, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536));
	VL53L0X_SetLimitCheckValue(DevLeft, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(DevLeft, 20000);
	VL53L0X_SetVcselPulsePeriod(DevLeft, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(DevLeft, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	VL53L0X_SetLimitCheckEnable(DevLeft, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,1);
	//VL53L0X_SetLimitCheckValue(DevLeft, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 0.5*65536)

	VL53L0X_SetDeviceAddress(DevLeft, ADDR_LEFT);
	DevLeft->I2cDevAddr=ADDR_LEFT;
	VL53L0X_StartMeasurement(DevLeft);

	//RIGHT sensor init
	DevRight->I2cHandle=hi2c_sides;
	DevRight->I2cDevAddr=0x52;
	DevRight->comms_speed_khz=400;
	XSHUT3(1);
	HAL_Delay(10);
	VL53L0X_WaitDeviceBooted( DevRight );
	VL53L0X_DataInit( DevRight );
	VL53L0X_StaticInit( DevRight );
	HAL_Delay(50);
	VL53L0X_PerformRefCalibration(DevRight, &VhvSettingsRight, &PhaseCalRight);
	VL53L0X_PerformRefSpadManagement(DevRight, &refSpadCountRight, &isApertureSpadsRight);
	VL53L0X_SetDeviceMode(DevRight,VL53L0X_DEVICEMODE_CONTINUOUS_RANGING);

	VL53L0X_SetLimitCheckEnable(DevRight, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckEnable(DevRight, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
	VL53L0X_SetLimitCheckValue(DevRight, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, (FixPoint1616_t)(0.25*65536));
	VL53L0X_SetLimitCheckValue(DevRight, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, (FixPoint1616_t)(32*65536));
	VL53L0X_SetMeasurementTimingBudgetMicroSeconds(DevRight, 20000);
	VL53L0X_SetVcselPulsePeriod(DevRight, VL53L0X_VCSEL_PERIOD_PRE_RANGE, 18);
	VL53L0X_SetVcselPulsePeriod(DevRight, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, 14);

	VL53L0X_SetLimitCheckEnable(DevRight, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,1);

	VL53L0X_SetDeviceAddress(DevRight, ADDR_RIGHT);
	DevRight->I2cDevAddr=ADDR_RIGHT;
	VL53L0X_StartMeasurement(DevRight);
}

void Tof_Task(I2C_HandleTypeDef *hi2c,UART_HandleTypeDef *huart, uint32_t tick, uint32_t period)
{
	static uint32_t tof_task_tick=0;
	uint8_t status;
	uint8_t status_front=0;
	uint8_t status_left=0;
	uint8_t status_right=0;
	uint8_t isReady=0;

	const float alpha = 0.4;
	static float distOld=0;
	uint16_t distFiltered=0;


	uint16_t dist_front;
	uint8_t range_status_front;


	uint16_t dist=1500;

	if(mode!=FAST)return;

	if(tof_task_tick>tick) return;
	tof_task_tick= tick+period;

	//mind 3 szenzor kiolvasása
	status_front = VL53L1_GetMeasurementDataReady(DevFront, &isReady);
	if(!status_front && isReady)
	{
		//Ezeket használja az API,de kvalassan működtek, ezért kézzel olvasom ki a távolságadatot
		//status_front = VL53L1_GetRangingMeasurementData(DevFront, &RangingDataFront);
		//VL53L1_ClearInterruptAndStartMeasurement(DevFront);
		VL53L1_RdByte(DevFront,0x0089, &range_status_front);//ranging status fornt sensor
		range_status_front = range_status_front & 0x1F;
		VL53L1_RdWord(DevFront,0x0096, &dist_front);//get front sensor distance
		VL53L1_WrByte(DevFront, 0x0086, 0x01);
	}


	status_left = VL53L0X_GetMeasurementDataReady(DevLeft, &isReady);
	if(!status_left && isReady) status_left = VL53L0X_GetRangingMeasurementData(DevLeft, &RangingDataLeft);

	status_right = VL53L0X_GetMeasurementDataReady(DevRight, &isReady);
	if(!status_right && isReady) status_right = VL53L0X_GetRangingMeasurementData(DevRight, &RangingDataRight);

	status=1;
	if(!status_front && range_status_front==9)
	{
		dist = dist_front;
		status=0;
	}

	if(!status_left && !RangingDataLeft.RangeStatus && RangingDataLeft.RangeMilliMeter < dist)
	{
		dist= RangingDataLeft.RangeMilliMeter;
		status=0;
	}

	if(!status_right && !RangingDataRight.RangeStatus && RangingDataRight.RangeMilliMeter < dist)
	{
		dist= RangingDataRight.RangeMilliMeter;
		status=0;
	}


	distOld=alpha * dist+ (1-alpha)*distOld;
	distFiltered=(uint16_t)distOld;

	__disable_irq();
	tofData[0]=status;
	tofData[1]=(uint8_t)(distFiltered>>8);
	tofData[2]=(uint8_t)(distFiltered & 0x00ff);
	__enable_irq();

#ifdef TOF_DEBUG
	char str[50];
	sprintf(str,"Left ->status: %d, dist: %4d\r\n", RangingDataLeft.RangeStatus,RangingDataLeft.RangeMilliMeter);
	HAL_UART_Transmit(huart,(uint8_t*)str, strlen(str), 20);

	sprintf(str,"Front->status: %d, dist: %4d\r\n", range_status_front,dist_front);//RangingDataFront.RangeMilliMeter);
	HAL_UART_Transmit(huart,(uint8_t*)str, strlen(str), 20);

	sprintf(str,"Right->status: %d, dist: %4d\n\r", RangingDataRight.RangeStatus,RangingDataRight.RangeMilliMeter);
	HAL_UART_Transmit(huart,(uint8_t*)str, strlen(str), 20);

	sprintf(str,"Sent->status: %d, dist: %4d\r\n\n", status,distFiltered);//RangingDataFront.RangeMilliMeter);
	HAL_UART_Transmit(huart,(uint8_t*)str, strlen(str), 20);
	tof_task_tick+=500;
#endif
}
