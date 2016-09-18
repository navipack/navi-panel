/**
******************************************************************************
* @file    TMPx75.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   温度检测芯片驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __TMPX75_H__
#define __TMPX75_H__

#include "stm32_lib.h"

bool TMP_Init(void);
u8 TMP_GetTemperature(s32 *pdata);
u8 TMP_GetConfig(u8 *pdata);

#endif
