/**
******************************************************************************
* @file    motor.h
* @author  Jalon
* @date    2016/09/05
* @brief   有刷直流电机 PWM 驱动相关声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "stm32_lib.h"
#include "motor_params_typ.h"

#define SINGLE_MAX              0x1000

void Motor_Init(void);
void Motor_Output(u8 idx, s32 out, u8 stop_flag);

extern MotorParamsTyp MotorParams[3];

#endif
