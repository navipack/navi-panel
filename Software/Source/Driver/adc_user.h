/**
******************************************************************************
* @file    adc_user.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   ADC 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __ADC_USER_H__
#define __ADC_USER_H__

#include "stm32_lib.h"
#include "adc.h"

#define RANK_DROP1      0
#define RANK_DROP2      1
#define RANK_DROP3      2
#define RANK_DROP4      3
#define RANK_REF        4
#define RANK_TEMP       5

#define RANK_DROP_MAX   4
#define RANK_MAX        8

// ADC 计算相关常数
#define V25             1430 // 1.43V
#define AVG_SLOPE       43   // 4.3 mV/°C
#define ADC_FULL_VALUE  0x0FFF
#define ADC_FULL_V      3300 //3.3V
#define ADC_REF_V       (1200*ADC_FULL_VALUE/ADC_FULL_V) //1.20V

void ADC_Init(ADC_HandleTypeDef* hadc);
s32 ADC_GetRef(void);
s32 ADC_GetDropValue(u8 idx);
s32 ADC_GetValue(u8 rank);
#define VOL_SIZE        RANK_MAX
extern u16 ADC1_Buffer[VOL_SIZE];

#endif
