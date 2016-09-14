/******************** (C) COPYRIGHT 2013 Haviea ********************
* File Name          : CurrentParams.h
* Author             : xc
* Date First Issued  : 13/04/09
* Description        : 电流采样参数
********************************************************************************/
#ifndef __CURRENT_PARAMS_H
#define __CURRENT_PARAMS_H

#include "stm32_lib.h"

/////////////////////// PWM Peripheral Input clock ////////////////////////////
#define DEADTIME_NS		((u16)600)  //in nsec; range is [0...3500] 

/****	 Pattern type is center aligned  ****/
//#define TIM_FACTOR          2 // TIM计数中央对齐为2，其它为1
//#define PWM_PRSC 	        0
//#define PWM_TIM_CNT_FREQ    (CKTIM_APB2 /TIM_FACTOR/(PWM_PRSC+1)) // PWM Timer 每秒计数的个数
//#define PWM_PERIOD 	        (PWM_TIM_CNT_FREQ / PWM_FREQ -1)
#define DEADTIME  	        (u16)((u64)PWM_CNT_FREQ * (u64)DEADTIME_NS/1000000000uL)

//#define SAMPLING_TIME_NS   500      //500ns
//#define SAMPLING_TIME_NS  900     //900ns
//#define SAMPLING_TIME_NS  1333    //1.33us
#define SAMPLING_TIME_NS  2267    //2.27us

// Time = (SampleCycles + 12)/ADC_CLK
#if (SAMPLING_TIME_NS == 500)
#define SAMPLING_TIME_CK  ADC_SAMPLETIME_3CYCLES
#elif (SAMPLING_TIME_NS == 900)
#define SAMPLING_TIME_CK  ADC_SAMPLETIME_15CYCLES
#elif (SAMPLING_TIME_NS == 1333)
#define SAMPLING_TIME_CK  ADC_SAMPLETIME_28CYCLES
#elif (SAMPLING_TIME_NS == 2267)
#define SAMPLING_TIME_CK  ADC_SAMPLETIME_56CYCLES
#else
#warning "Sampling time is not a possible value"
#endif


//#define TNOISE_NS 2550    //2.55usec
//#define TRISE_NS 2550     //2.55usec

#define T_NOISE_NS  1500    //3.00usec
#define T_RISE_NS   100     // DRV8848

#if (TNOISE_NS > TRISE_NS)
  #define MAX_TNTR_NS TNOISE_NS
#else
  #define MAX_TNTR_NS TRISE_NS
#endif

#define TW_AFTER 	((u16)(((DEADTIME_NS+MAX_TNTR_NS)*72ul)/1000ul))       // 已废弃
#define TW_BEFORE 	(((u16)(((((u16)(SAMPLING_TIME_NS)))*72ul)/1000ul))+1)

//最大和最小的PWM
//最大PWM：PWM满的情况下，IR2101由于漏电流的关系，MOS端电压下降;PWM很小的情况下，IR2101
//最小PWM：留出足够的采样时间（A相最小的时候，B相和A相接近的时候）
#define MIN_PWM_NS		T_RISE_NS//((u16)(DEADTIME_NS + T_RISE_NS + SAMPLING_TIME_NS))
#define MIN_PWM_SAMP	((u16)((u64)PWM_CNT_FREQ * (u64)MIN_PWM_NS / (u64)(1000000000uL)))
#define MAX_PWM_NS		((u16)1)
#define MAX_PWM_IR		((u16)(PWM_PERIOD - 150))	//保证开关顺利的最大PWM
#define MAX_PWM_SAMP	((u16)(PWM_PERIOD - MIN_PWM_SAMP))


//#if (MAX_PWM_SAMP < MAX_PWM_IR)
//根据计算出来的MAX_PWM_SAMP和MAX_PWM_IR确定采样点
	#define MAX_PWM	MAX_PWM_SAMP
	#define MIN_PWM MIN_PWM_SAMP
//#else
//	#define MAX_PWM MAX_PWM_IR
//	#define MIN_PWM ((u16)(PWM_PERIOD - MAX_PWM))
//#endif

#endif
/******************* (C) COPYRIGHT 2013 Haviea *****END OF FILE****/
