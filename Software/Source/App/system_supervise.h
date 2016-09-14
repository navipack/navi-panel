/**
******************************************************************************
* @file    system_supervise.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   系统监控声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __SYSTEM_SUPERVISE_H__
#define __SYSTEM_SUPERVISE_H__

#include "stm32_lib.h"

void SystemSuperviseInit(void);
void SystemSuperviseTask(void);
void SendErrorEvent(u32 errcode, u8 errtype);
void ClearErrorEvent(u32 errcode);
void ResetHeartbeat(void);

#endif
