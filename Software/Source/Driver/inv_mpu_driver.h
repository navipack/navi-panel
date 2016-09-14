/**
******************************************************************************
* @file    inv_mpu_driver.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   MPU6500 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __INV_MPU_DRIVER_H__
#define __INV_MPU_DRIVER_H__

#include "stm32_lib.h"

//#define USE_DMP

#define ACCEL_FSR 2
#define ACCEL_FSR_CALC  (0x7FFF/ACCEL_FSR)
#ifdef USE_DMP
#define GYRO_FSR        2000
#else
#define GYRO_FSR        500
#endif
#define GYRO_FSR_CALC   (0x7FFF/GYRO_FSR)

#define MPU_DATA_QUAT   0x01
#define MPU_DATA_GYRO   0x02        
#define MPU_DATA_ACCEL  0x04

typedef struct MPUSensorDataStruct
{
    u8 sensors;
    long quat[4];
	s16 accel_x;
	s16 accel_y;
	s16 accel_z;
	
	s16 temp;
	
	s16 gyro_x;
	s16 gyro_y;
	s16 gyro_z;
}MPUSensorData;

void InvMPU_Driver_Init(void);
u8 INVMPU_WriteBytes(u8 addr, u16 len, const u8* data);
u8 INVMPU_ReadBytes(u8 addr, u16 len, u8* data);
void INVMPU_ReadAccel(MPUSensorData* data);
void INVMPU_ReadGyro(MPUSensorData* data);
void INVMPU_DmpReadFifo(MPUSensorData *data);
bool MPU6500_Test(void);

#endif
