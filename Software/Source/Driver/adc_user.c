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

void ADC_Init(ADC_HandleTypeDef* hadc)
{
    if(hadc == &hadc1)
    {
        HAL_ADCEx_Calibration_Start(hadc);
        HAL_ADC_Start_DMA(hadc, (u32*)ADC1_Buffer, VOL_SIZE);
    }
}

s32 ADC_GetRef(void)
{
    RefValue = (s32)ADC1_Buffer[RANK_REF] * 1000 / ADC_REF_V;
    return RefValue;
}

s32 ADC_GetValue(u8 rank)
{
    if(rank < RANK_MAX)
    {
      return ((s32)ADC1_Buffer[rank]) * 1000 / RefValue;
    }
    return 0;
}

s32 ADC_GetDropValue(u8 idx)
{
    if(idx < RANK_DROP_MAX)
    {
        return ADC_GetValue(RANK_DROP1 + idx);
    }
    return 0;
}
