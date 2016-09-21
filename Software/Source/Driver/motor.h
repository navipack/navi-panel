/**
******************************************************************************
* @file    motor.h
* @author  Jalon
* @date    2016/09/05
* @brief   有刷直流电机 PWM 驱动相关声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32_lib.h"
#include "motor_params_typ.h"

/////////////////////// PWM Peripheral Input clock ////////////////////////////
#define DEADTIME_NS		((u16)600)  //in nsec; range is [0...3500] 

/****	 Pattern type is center aligned  ****/
#define DEADTIME  	        (u16)((u64)PWM_CNT_FREQ * (u64)DEADTIME_NS/1000000000uL)

#define T_NOISE_NS  1500
#define T_RISE_NS   100

#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif

//最大和最小的PWM
//最大PWM：PWM满的情况下，IR2101由于漏电流的关系，MOS端电压下降;PWM很小的情况下
//最小PWM：留出足够的采样时间（A相最小的时候，B相和A相接近的时候）
#define MIN_PWM_NS		T_RISE_NS//((u16)(DEADTIME_NS + T_RISE_NS + SAMPLING_TIME_NS))
#define MIN_PWM_SAMP	((u16)((u64)PWM_CNT_FREQ * (u64)MIN_PWM_NS / (u64)(1000000000uL)))
#define MAX_PWM_NS		((u16)1)
#define MAX_PWM_IR		((u16)(PWM_PERIOD - 150))	//保证开关顺利的最大PWM
#define MAX_PWM_SAMP	((u16)(PWM_PERIOD - MIN_PWM_SAMP))


//根据计算出来的MAX_PWM_SAMP和MAX_PWM_IR确定采样点
#define MAX_PWM	MAX_PWM_SAMP
#define MIN_PWM MIN_PWM_SAMP

#define PWM_FREQ 20000 //不宜低于人类听觉上限，否则可能会有明显噪音

#define SINGLE_MAX 0x1000

void Motor_Init(void);
void Motor_Output(u8 idx, s32 out, u8 stop_flag);

extern MotorParamsTyp MotorParams[2];

#endif
