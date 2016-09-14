
#ifndef __FIR_FILTER_H__
#define __FIR_FILTER_H__

#include "stm32_lib.h"

typedef struct FirFilterIntDefStruct
{
	s32 *delay;		 //暂存数量
	const s32 *factor;
	s32 tap;	
}FirFilterIntDef;

typedef struct FirFirstOrderFilterInt64DefStruct
{
	s64 invar;		//输入X(n)
	s64 delay;		//上次输出Y(n-1)
	s32 factor;		//滤波因数
	s32 reserve;	//保留小数部分
}FirFirstOrderFilterInt64Def;

typedef struct FirFirstOrderFilterInt32DefStruct
{
	s32 invar;		//输入X(n)
	s32 delay;		//上次输出Y(n-1)
	s32 factor;		//滤波因数
	s32 reserve;	//保留小数部分
}FirFirstOrderFilterInt32Def;

typedef struct FirMovingAvgFilterInt64DefStruct
{
	s64 *delay;		//滤波延迟数组
	s64 invar; 		//输入X(n)
	s16 index;		//当前延迟索引
	s16 count_mi;		//数组个数，2的整数幂
	s64 sum;		
}FirMovingAvgFilterInt64Def;

typedef struct FirMovingAvgFilterInt32DefStruct
{
	s32 *delay;		//滤波延迟数组
	s32 invar; 		//输入X(n)
	s16 index;		//当前延迟索引
	//s16 count_mi;		//数组个数
	s16 count;
	s64 sum;
}FirMovingAvgFilterInt32Def;

typedef struct FirFilterFloatDefStruct
{
	float *delay;		 //暂存数量
	const float *factor;
	s32 tap;	
}FirFilterFloatDef;

s32 FIR_Filter_int(FirFilterIntDef *filter, s32 invar) ;
void FIR_Filter_int_Clear(FirFilterIntDef *filter);
#endif

