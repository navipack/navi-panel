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

//#define USE_REMOTE_CONTROL

#define LocationLoopV_Max       800 // mm/s
#define LocationLoopW_Max       DEGREE(180)
#define ReachNearerLocation     100 // mm
#define ReachFartherLocation    300 // mm
#define ReachAligmentTheta      DEGREE(5)

//increase 500 times per sec.
#define BasicVelocityInc 800/500
#define BasicAngularVelocityInc DEGREE(120)/500

typedef enum ShellyMotoLoopModeTypEnum
{
    OPEN_LOOP,
    TORQUE_LOOP,
    SPEED_TORQUE_LOOP,
    POS_TORQUE_LOOP,
    POS_SPEED_TORQUE_LOOP,
    SPEED_LOOP,
    POS_LOOP,
    POS_SPEED_LOOP,
    ANG_VEL_SPEED_LOOP,
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
    u8 validlocationCNT;
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
void ResetPosLoopCount(void);
void SetVWValue(s32 v, s32 w, u16 t);
bool IsWheelRest(void);
bool PushTargetLocation( const MotionTargetType* value );
void SetCarMotionEnable(bool b);

extern CCarLocation gCPresentLocation;

void MotorParamsInit(void);
void AngularVelocityController(s32 TargetV, s32 TargetW, s32 velocity, s32 omega);
s32 GetVelocity(s32 target_v);
s32 GetOmega(s32 target_w);
void EncInit(u8);
void MotorPIDInit(void);

#endif


