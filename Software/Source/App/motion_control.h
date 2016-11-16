/**
******************************************************************************
* @file    motion_control.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   运动控制模块声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __MOTOR_SPEED_LOOP_H__
#define __MOTOR_SPEED_LOOP_H__

#include "stm32_lib.h"
#include <stdlib.h>
#include <string.h>
#include "FIR_Filter.h"
#include "motor.h"
#include "math_lib.h"

#define MOTION_PREQ       1000      //速度环频率

typedef enum ShellyMotoLoopModeTypEnum
{
    open_loop,
    torque_loop,
    speed_torque_loop,
    pos_torque_loop,
    pos_speed_torque_loop,
    speed_loop,
    pos_loop,
    pos_speed_loop,
    ang_vel_speed_loop,
}ShellyMotoLoopModeTyp;

typedef struct CSpeedVWStruct
{
	s32 sV;
	s32 sW;
}CSpeedVW;

typedef struct MotionTargetVWTTypeStruct
{
	s32 sV;
	s32 sW;
    u16 time;
}MotionTargetVWTType;

typedef struct CCarLocationStruct
{
	s32 sAxisX;
	s32 sAxisY;
	s32 sAbsTheta;
}CCarLocation;

typedef struct CTwoDistanceStruct
{
	s32 nearerDis;
	s32 fartherDis;
	 u8 validCNT;
}CTwoDistance;

typedef struct MotionTargetTypeStruct
{ 
    u16 uIndex;
    s32 sAxisX;
    s32 sAxisY;
    s16 sMaxV;
    s32 sMaxW;
    s32 sTheta;
} MotionTargetType;

typedef struct CTwoCarLocationStruct
{
	MotionTargetType point[2];
    u8 valid_location_cnt;
}CTwoCarLocation;

typedef struct CDistanceValueStruct
{
	s32 distance;
	s32 theta;
} CDistanceValue;

typedef struct HeadTargetTypeStruct
{
    s32 position;
    s32 omega;
}HeadTargetType;

void MotionCtrlTask(void);
void SetVWValue(s32 v, s32 w, u16 t);
bool IsWheelRest(void);
void SetCarMotionEnable(bool b);

#endif


