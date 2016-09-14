/**
******************************************************************************
* @file    infrared_drop.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   地测跌落传感器驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __INFRARED_DROP_H__
#define __INFRARED_DROP_H__

#include "stm32_lib.h"

void InfraredDrop_InitData(bool enable);
u8 InfraredDrop_GetData(void);

#endif
