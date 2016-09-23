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

/**
  *@name MPU6500相关宏定义
  *@{
*/
#define MPU_SPI (&hspi1)

#define MPU6500_WHO_AM_I_ADDR	0x75
#define MPU6500_WHO_AM_I_VALUE  0x70

#define D_EXT_GYRO_BIAS_X       (61 * 16)
#define D_EXT_GYRO_BIAS_Y       (61 * 16) + 4
#define D_EXT_GYRO_BIAS_Z       (61 * 16) + 8
/**
  *@}
*/
#define ACCEL_FSR 2     /** 加速度传感器满量程范围实际数值 */    
#define ACCEL_FSR_CALC  (0x7FFF/ACCEL_FSR) /** 加速度传感器满量程范围对应数值 */ 
#ifdef USE_DMP
#define GYRO_FSR        2000  /** 陀螺仪满量程范围实际数值 */
#else
#define GYRO_FSR        500   /** 陀螺仪满量程范围实际数值 */
#endif
#define GYRO_FSR_CALC   (0x7FFF/GYRO_FSR) /** 陀螺仪满量程范围对应数值 */ 

#define MPU_DATA_QUAT   0x01    /** 用于区分四元数的对应数值 */
#define MPU_DATA_GYRO   0x02    /** 用于区分陀螺仪的对应数值 */    
#define MPU_DATA_ACCEL  0x04    /** 用于区分加速度的对应数值 */    


typedef struct MPUSensorDataStruct
{
    u8 sensors;         
    long quat[4];   /** 四元数存放数组 */       
	s16 accel_x;    /** 加速度_x存放数值 */   
	s16 accel_y;    /** 加速度_y存放数值 */   
	s16 accel_z;    /** 加速度_z存放数值 */   
	
	s16 temp;
	
	s16 gyro_x;     /** 陀螺仪_x存放数值 */
	s16 gyro_y;     /** 陀螺仪_y存放数值 */
	s16 gyro_z;     /** 陀螺仪_z存放数值 */
}MPUSensorData;

void InvMPU_Driver_Init(void);
u8 INVMPU_WriteBytes(u8 addr, u16 len, const u8* data);
u8 INVMPU_ReadBytes(u8 addr, u16 len, u8* data);
void INVMPU_ReadAccel(MPUSensorData* data);
void INVMPU_ReadGyro(MPUSensorData* data);
void INVMPU_DmpReadFifo(MPUSensorData *data);
bool MPU6500_Test(void);

#endif
