/**
******************************************************************************
* @file    tim_user.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   定时器相关驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __TIM_USER_H__
#define __TIM_USER_H__

#include "stm32_lib.h"
#include "tim.h"

typedef struct RunFlagTypesStruct
{
    u8 led;
    u8 supervise;
    u8 second;
    u8 boost;
    u8 imu;
    u8 motion;
    u8 us;
    u8 contact;
    
}RunFlagType;

extern RunFlagType RunFlag;

void RunFlagHandler(void);
u32 GetIntervalCnt(u32 *last_capture_cnt, u32 *last_cnt);
void BasicTIM_IRQHandler(TIM_HandleTypeDef *htim);

#endif
