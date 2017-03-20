/**
******************************************************************************
* @file    speed_loop.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   速度闭环相关声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SPEED_LOOP_H__
#define __SPEED_LOOP_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32_lib.h"
#include "PID_regulators.h"
#include "FIR_Filter.h"
#include "motion_control.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define V_FACTOR 1440
#define V_FULL_FACTOR 10000
#define V_ENC_TO_MM(_v) ((_v) * V_FACTOR / V_FULL_FACTOR)

#define W_FACTOR 72353
#define W_FULL_FACTOR 1000
#define W_ENC_TO_DEG(_w) ((_w) * W_FACTOR / W_FULL_FACTOR)
/* Exported functions ------------------------------------------------------- */
void MotorPIDInit(void);
void AngularVelocityController(s32 TargetV, s32 TargetW, s32 velocity, s32 omega);
void SpeedLoop_SetTargetSpeed(CSpeedVW *s);
void SpeedLoop(void);

#endif
