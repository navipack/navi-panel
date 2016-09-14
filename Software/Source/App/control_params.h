/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_PID_regulators.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : Contains the prototypes of PI(D) related functions.
* 
********************************************************************************/

#ifndef __CONTROL_PARAMS_H
#define __CONTROL_PARAMS_H

//控制相关参数配置----------------------------------------------------------------
#define PWM_FREQ 		20000 //不宜低于人类听觉上限，否则可能会有明显噪音

//闭环调整频率----------------------------------------------------------------------
#define MOTION_PREQ       1000      //速度环频率

#endif
