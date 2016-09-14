/**
******************************************************************************
* @file    sensor_update.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   IMU 传感器解算
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __SENSORUPDATE_H__
#define __SENSORUPDATE_H__

#include "stm32_lib.h"

/*
*********************************************************************************************************
*                                            GLOBAL TYPES
*********************************************************************************************************
*/
//惯导传感器原始信息
typedef struct IMURawInfoStruct
{
	s32 accel_pitch;
	s32 accel_roll;
	s32 accel_z;
	s32 gyro_x;
	s32 gyro_y;
	s32 gyro_z;
}IMURawInfo;

//惯导滤波器信息
typedef struct IMUFilterInfoStruct
{
	s32 static_pitch_angle;
	s32 static_roll_angle;
    s32 static_yaw_angle;
	s32 static_vertical_angle;
	s32 fused_pitch_angle;
	s32 fused_roll_angle;
	s32 fused_yaw_angle;
	s32 pitch_anglerate;
	s32 roll_anglerate;
	s32 yaw_anglerate;
}IMUFilterInfo;

typedef struct IMUDebugInfoStruct
{
	float static_pitch_angle;
	float static_roll_angle;
	float static_yaw_angle;
	float fused_pitch_angle;
	float fused_roll_angle;
	float fused_yaw_angle;
	float pitch_anglerate;
	float roll_anglerate;
	float yaw_anglerate; 
}IMUDebugInfo;

typedef struct IMURawDebugInfoStruct
{
	float accel_x;
	float accel_y;
	float accel_z;
	float gyro_x;
	float gyro_y;
	float gyro_z;
}IMURawDebugInfo;

typedef struct ThreeAxisTypeStruct
{
    s32 x;
    s32 y;
    s32 z;
}ThreeAxis_Type;


/*
*********************************************************************************************************
*                                            GLOBAL VARIABLES
*********************************************************************************************************
*/
extern IMUFilterInfo g_SensorSystemStatus;
void SensorUpdateTask(void);


#endif


