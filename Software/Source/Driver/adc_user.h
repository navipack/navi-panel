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

/**
  *@defgroup ADC_USER_DEF ADC 相关定义
  *@{
*/

/**
  *@brief ADC1 的采样通道宏定义
*/
#define RANK_DROP1      0    ///< 地测传感器
#define RANK_DROP2      1
#define RANK_DROP3      2
#define RANK_DROP4      3

#define RANK_REF        4    ///< 参考电压
#define RANK_TEMP       5    ///< 内部温度传感器

#define CONTACT_DROP1   6    ///< 碰撞传感器
#define CONTACT_DROP2   7

#define RANK_DROP_MAX   4    ///< 地测传感器所使用的转换通道数最大值
#define RANK_MAX        8    ///< 转换通道数最大值

/**
  *@brief ADC 计算相关常数
*/
#define V25             1430     ///< 1.43V
#define AVG_SLOPE       43       ///< 4.3 mV/°C
#define ADC_FULL_VALUE  0x0FFF   ///< ADC 最大数值
#define ADC_FULL_V      3300     ///< ADC 最大电压 3.3V
#define ADC_REF_V       (1200*ADC_FULL_VALUE/ADC_FULL_V) ///< ADC 参考电压 1.20V

/**
  *@}
*/

void ADC_Init(ADC_HandleTypeDef* hadc);
s32 ADC_GetRef(void);
s32 ADC_GetDropValue(u8 idx);
s32 ADC_GetValue(u8 rank);
#define VOL_SIZE        RANK_MAX
extern u16 ADC1_Buffer[VOL_SIZE];

#endif
