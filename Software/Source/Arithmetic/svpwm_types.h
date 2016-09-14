/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_type.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This header file provides structure type definitions that 
*                      are used throughout this motor control library.
********************************************************************************
* History:
* 21/11/07 v1.0
* 29/05/08 v2.0
********************************************************************************/
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SVPWM_TYPES_H
#define __SVPWM_TYPES_H

/* Includes ------------------------------------------------------------------*/
#include "stm32_lib.h"

/* Exported types ------------------------------------------------------------*/
//以下自定义FOC过程中的全局变量
//定子相电流
typedef struct PhaseCurrentTypStruct
{
	s16 Ia;
	s16 Ib;		
}PhaseCurrentTyp;

//转换成α，β电流坐标系
typedef struct StatorCurrentTypStruct
{
	s16 Ialfa;
	s16 Ibeta;		
}StatorCurrentTyp; 

//转换成转子d,q轴电流坐标系
typedef struct RotorCurrentTypStruct
{
	s16 Iq;
	s16 Id;		
}RotorCurrentTyp; 

//定子相电压
typedef struct PhaseVoltageTypStruct
{
	s16 Va;
	s16 Vb;		
}PhaseVoltageTyp; 

//电流Iq、Id PID调节后输出的d,q轴转子电压
typedef struct RotorVoltageTypStruct
{
	s16 Vq;
	s16 Vd;		
}RotorVoltageTyp;

//Vq、Vd经Park逆变换后输出的定子α，β轴系电压
typedef struct StatorVoltageTypStruct
{
	s16 Valfa;
	s16 Vbeta;		
}StatorVoltageTyp;

typedef struct TriangleTypStruct
{
	s16 hCos;
	s16 hSin;
} TriangleTyp; 

typedef struct PWM3ShuntTypStruct
{
	s16 Va;
	s16 Vb;
	s16 Vc;
}PWM3ShuntTyp;
 
typedef struct HEdgeToElecAngleTypStruct
{
	s32 HallState;
	s32 HToElecAngle;
}HEdgeToElecAngleTyp;


/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/

/* Exported functions ------------------------------------------------------- */

#endif /* __MC_TYPE_H */
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
