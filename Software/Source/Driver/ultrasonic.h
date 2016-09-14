/**
******************************************************************************
* @file    ultrasonic.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   超声波模块驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __ULTRASONIC_H__
#define __ULTRASONIC_H__

#include "stm32_lib.h"

void Ultrasonic_Init(void);
void UltrsonicTrigTask(void);
void Ultrasonic_IRQHandler(void);
#endif
