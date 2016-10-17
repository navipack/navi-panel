/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : InternalParams.h
* Author             : xc 
* Date First Issued  : 21/11/07
* Description        : 内部变量
********************************************************************************/
#ifndef __INTERNAL_PARAMS_H
#define __INTERNAL_PARAMS_H

#include "stm32_lib.h"
#include "PID_regulators.h"
#include "tim.h"
#include "adc.h"

typedef struct RotorCurrentTypStruct
{
	s16 Iq;
	s16 Id;
}RotorCurrentTyp; 

typedef struct MotorParamsTypStruct
{
    //关键参数需要被赋值------------------------------------------//
    TIM_HandleTypeDef *ENC_TIMER;         ///< 编码器时钟
    TIM_HandleTypeDef *PWM_TIMER;         ///< PWM输出时钟
    u16 EncoderGapNum;          ///< 编码器线数
    u8  PairNum;                ///< 极对数
    u8  EncoderDir;             ///< 编码器方向
    u32 SequenceLength;         ///< 注入转换长度
    ADC_HandleTypeDef *CURR_ADC;     ///< 电流采样 ADC
    
    //PID控制参数值-----------------------------------------------//
    s32 MaxSpeed;
    u16 MaxTorque;
    s32 MaxVoltage;
    s32 TurnsCount;
    
    //编码器------------------------------------------------------//
    s64 EncDecPluse;            
    s16 PreviousCnt;
    s32 PreviousOverflowCnt;
    u32 PreviousBasicTimCnt;
    volatile s32 EncTimOverflow; ///< 该变量指示UPDATE中断发生的个数
    bool bIs_First_Measurement; //编码器是否第一次测量
    
    u16 hPhaseAOffset;          ///< A相偏移量
    u16 hPhaseBOffset;          ///< B相偏移量
    //======反馈值======//
    RotorCurrentTyp PresentCurrentDQ;
    s32 PresentSpeed;
    s32 AccumulatedDistance;
    s32 AccumulatedDistanceRemainder;
    s64 PresentPosition;

	//电机初始化参数-----------------------------------------------//
	s16 AlignmentAngle;
    s32 DeadZone;
    s32 DeadZoneMove;
}MotorParamsTyp;

#endif
