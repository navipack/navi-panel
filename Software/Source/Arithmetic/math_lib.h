/**
******************************************************************************
* @file    math_lib.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   数学函数库
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __MATH_LIB_H__
#define	__MATH_LIB_H__

/* Includes -------------------------------------------------*/
#include "stm32_lib.h"

/* Defines --------------------------------------------------*/
#define SQUARE(s)			((s32)((s) * (s)))

#define SQRT_3		1.732051
#define PI          3.1416
#define SQRT_2      1.4142

//回绕计数变量的减法计算
#define MINUS_S32(_a,_b) ((s32)((s32)(_a) - (s32)(_b))) //计数变量为 u32 类型
#define MINUS_S64(_a,_b) ((s64)((s64)(_a) - (s64)(_b))) //计数变量为 u64 类型

#define DEGREE_QCALC    0x1000
#define DEGREE(x)       ((s32)(DEGREE_QCALC)*(x))
#define DEGREE_TO_RADIAN(x) (s32)((s64)(x) * 6283 / DEGREE(360)) //倍乘角度转毫弧度
#define RADIAN_TO_DEGREE(x) (s32)((s64)(x) * DEGREE(360) / 6283) //毫弧度转倍乘角度

#define TRIGONOMETRIC_TIMES 4096

typedef struct LineParameterStruct
{
    s64 a;
    s64 b;
    bool flag;
} LineParameter;

typedef struct PointCoordinateStruct
{
    s32 x;
    s32 y;
} PointCoordinate;

/* Exported Functions ---------------------------------------*/
s32 GetAbs(s32 x);
s32 GetSqurt32(s32 x);

s32 GetSin(s32 x);		// 范围为-90 — +90度
s32 GetCos(s32 x);		// 范围为-90 — +90度
s32 GetAsin(s32 x);		// 范围为-180 — +180度	
s32 GetAtan(s32 x , s32 y, s32 flag);		// 范围为0 — +360度

LineParameter CalculateLine( const PointCoordinate* firstpoint, const PointCoordinate* secondpoint );
LineParameter CalculateVerticalLine( const LineParameter* line, const PointCoordinate* foot );


#endif

