/**
******************************************************************************
* @file    gpio_user.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   GPIO 相关驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __GPIO_USER_H__
#define __GPIO_USER_H__

#include "stm32_lib.h"
#include "gpio.h"

void GPIO_DriverInit(void);

bool IsPickUp(void);

void LedToggle(void);
void LidarPower(bool on);
void UltrasonicPower(bool on);
void DropPower(bool on);
void ChassisMotorDriverEnable(bool on);
#endif
