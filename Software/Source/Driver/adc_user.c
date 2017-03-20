/**
******************************************************************************
* @file    adc_user.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   ADC 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "adc_user.h"

u16 ADC1_Buffer[VOL_SIZE];
u16 RefValue = 1000;

/**
* @brief  初始化ADC1，DMA通道
* @param  hadc：ADC口
* @retval None
*/
void ADC_Init(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {
        HAL_ADCEx_Calibration_Start(hadc);
        HAL_ADC_Start_DMA(hadc, (u32*)ADC1_Buffer, VOL_SIZE);
    }
}

/**
* @brief  采集校准参数
* @retval None
* @note：采集1.2V输入电压，与理论1.2v比较，防止供电电压不稳导致ADC采集的电压不正确
*/
s32 ADC_GetRef(void)
{
    RefValue = (s32)ADC1_Buffer[RANK_REF] * 1000 / ADC_REF_V ;
    return RefValue;
}

/**
* @brief  ADC1 采集电压值
* @param  rank：目标编号
* @retval 目标 ADC 采样值
*/
s32 ADC_GetValue(u8 rank)
{
    if(rank < RANK_MAX)
    {
        return ((s32)ADC1_Buffer[rank]) * 1000 / RefValue;
    }
    return 0;
}

/**
* @brief  地侧ADC采集电压值
* @param  idx: 目标编号
* @retval 目标 ADC 采样值
* @note　因为地侧的ADC只用到四个，单独开来
*/
s32 ADC_GetDropValue(u8 idx)
{
    if(idx < RANK_DROP_MAX)
    {
        return ADC_GetValue((RANK_DROP_MAX-1) - idx);
    }
    return 0;
}
