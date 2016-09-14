/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : InternalParams.h
* Author             : xc 
* Date First Issued  : 21/11/07
* Description        : 内部变量
********************************************************************************/
#ifndef __INTERNAL_PARAMS_H
#define __INTERNAL_PARAMS_H

#include "stm32_lib.h"
#include "svpwm_types.h"
#include "PID_regulators.h"
#include "tim.h"
#include "adc.h"

typedef struct MotorParamsTypStruct
{
    //关键参数需要被赋值------------------------------------------//
    TIM_HandleTypeDef *ENC_TIMER;         //编码器时钟
    TIM_HandleTypeDef *PWM_TIMER;         //PWM输出时钟
    u16 EncoderGapNum;          //编码器线数
    u8  PairNum;                //极对数
    u8  EncoderDir;             //编码器方向
    u8 PhaseAChannel;           //ADC采样通道A
    u8 PhaseBChannel;           //ADC采样通道B
    u8 PhaseCChannel;           //ADC采样通道C
    u32 PhaseAChannelMask;       //ADC采样通道A屏蔽位
    u32 PhaseBChannelMask;       //ADC采样通道B屏蔽位
    u32 PhaseCChannelMask;       //ADC采样通道C屏蔽位
    u32 SequenceLength;         //注入转换长度
    u64 HallEdgePositive;       //霍尔沿对齐正向角度
    u64 HallEdgeNegative;       //霍尔沿对齐负向角度
    u64 HallStartUp;            //霍尔启动角度
    ADC_HandleTypeDef *CURR_ADC;     //电流采样 ADC
    u32 ExternalTriggerInject;  //外部触发Trigger
    
    //电机状态控制值----------------------------------------------//
    u8 State;
    u8 Enable;
    u8 LoopMode;
    u8 LastLoopMode;
    u8 ZeroLock;
    
    PIDObjTyp PIDTorque;        //力矩PID
    PIDObjTyp PIDFlux;          //磁通PID
    //PIDObjTyp PIDSpeed;         //转速PID
    PIDObjTyp PIDPosition;      //位置PID
    PIDObjTyp PIDSpeedPosition; //速度位置PID
    
    FeedForwardObjTyp FFPositionToSpeed;
    FeedForwardObjTyp FFSpeedToTorque;
    bool bIs_First_Measurement; //编码器是否第一次测量
    
	s32 AccPositiveTime;				//加速时间
	s32 AccNegativeTime;				//减速时间
    
    //PID控制参数值-----------------------------------------------//
    u32 TorqueKp;
    u32 TorqueKi;
    u32 TorqueKff;
    u32 SpeedKp;
    u32 SpeedKi;
    u32 SpeedKff;
    u32 PosKp;
    u32 PosKi;
    s32 MaxSpeed;
    u16 MaxTorque;
    s32 MaxVoltage;
    s32 TurnsCount;
    
    //编码器时钟定义-----------------------------------------------//
    s64 EncDecPluse;            
    s32 EncDecPluse_Speed_Use;  
    s16 PreviousCnt;
    s32 hRot_Speed;
    s32 hObserve_Speed;
    s32 PreviousOverflowCnt;
    u32 PreviousBasicTimCnt;
    volatile s32 EncTimOverflow;//该变量指示UPDATE中断发生的个数，也就是说
							   	//ENC每转一圈，CPU得到 4* PPR(编码器自身线数，自解)个脉冲，中断发生一次
							   	//该变量+1.
    
    //电流采样相关定义---------------------------------------------//
    u16 hPhaseAOffset;          //A相偏移量
    u16 hPhaseBOffset;          //B相偏移量
    s8 hFirstReadOffset;        
    s8 hFirstReadOffsetCount;   
    u8 PWM4Direction;           // = PWM2_MODE;
 
    //SVPWM定义----------------------------------------------------//
    //======中间值======//
    PhaseCurrentTyp CurrentAB;//	 		(*(PhaseCurrentTyp*)&EzCANServoParams.Status.NowCurrAB) 		//定子相电流
    StatorCurrentTyp CurrentAlfaBeta;//		(*(StatorCurrentTyp*)&EzCANServoParams.Status.NowCurrAlfaBeta)	//转换成α，β电流坐标系
    PhaseVoltageTyp VoltageAB;//		 	(*(PhaseVoltageTyp*)&EzCANServoParams.Status.NowVolAB)			//定子相电压
    StatorVoltageTyp VoltageAlfaBeta;//		(*(StatorVoltageTyp*)&EzCANServoParams.Status.NowVolAlfaBeta)	//Vq、Vd经Park逆变换后输出的定子α，β轴系电压 
    TriangleTyp ElecTriangle;// 		(*(TriangleTyp*)&EzCANServoParams.Status.ElectricTriangle)					//电角度（经过三角变换）
    u16 ElecAngle;//			(EzCANServoParams.Status.ElectricAngular)					//电角度
    u32 Sector;//				(EzCANServoParams.Status.Sector)							//当前所在扇区
    s64 InternalTargetPos;//	(EzCANServoParams.Status.InternalTargetPosition)			//内部目标位置
    s32 InternalTargetSpd;//	(EzCANServoParams.Status.InternalTargetSpeed)				//内部目标速度
    RotorCurrentTyp InternalTargetCurrentDQ;//	(*(RotorCurrentTyp*)&EzCANServoParams.Status.InternalTargetCurrentDQ)		//内部目标电流
    
    //======反馈值======//
    RotorCurrentTyp PresentCurrentDQ;//	(*(RotorCurrentTyp*)&EzCANServoParams.Status.NowCurrentDQ)
    s32 PresentSpeed;//		(EzCANServoParams.Status.NowSpeed)
    s32 AccumulatedDistance;
    s32 AccumulatedDistanceRemainder;
    s64 PresentPosition;//		(EzCANServoParams.Status.NowPosition)
    RotorVoltageTyp PresentVoltageDQ;//	(*(RotorVoltageTyp*)&EzCANServoParams.Status.NowVoltageDQ)

    //======目标控制值======//
    RotorCurrentTyp TargetCurrentDQ;//		(*(RotorCurrentTyp*)&EzCANServoParams.Control.TargetCurrent)
    s64 TargetPosition;//		(EzCANServoParams.Control.TargetPosition)
    s32 TargetSpeed;//			(EzCANServoParams.Control.TargetSpeed)
    RotorVoltageTyp TargetVoltageDQ;//		(*(RotorVoltageTyp*)&EzCANServoParams.Control.TargetVoltage)

    //上次目标位置-------------------------------------------------//
	s64 LastTargetPos;			//上次目标位置，位置环电子齿轮时用到
    
	//速度环加减速参数---------------------------------------------//
	s32 TargetSpeedIncRemainder;	//目标速度增量的余数
	s32 TargetSpeedIncModulus;		//目标速度增量的模数
	s32 TargetSpeedDecRemainder;	//目标速度减少的余数
	s32 TargetSpeedDecModulus;		//目标速度减少的模数
	s32 TargetSpeedIncDivisor;		//目标速度增加的除数，用于累加
	s32 TargetSpeedDecDivisor;		//目标速度减少的除数

   	//速度环滤波参数-----------------------------------------------//
	s32 SpeedFeedbackFilterParam;		//速度反馈滤波
	s32 SpeedFeedForwardFilterParam;	//速度前馈滤波
    u8  SpeedFIRFilterLevel;			//速度环FIR滤波器等级

	//转矩环滤波参数-----------------------------------------------//
	s32 TorqueFeedbackFilterParam;		//转矩反馈滤波
	s32 TorqueFeedForwardFilterParam;	//转矩前馈滤波

	//电机初始化参数-----------------------------------------------//
	u8  FinishRecognize;				//完成电机识别
    
	s16 AlignmentAngle;					//对齐电角度（360表示）
	s16 EncoderElecAngle;				//编码器对齐电角度，在向导模式用

    s16 CurrentAttenuation;             //电流限制环输出
    
    s32 DeadZone;
    s32 DeadZoneMove;
}MotorParamsTyp;

#endif
