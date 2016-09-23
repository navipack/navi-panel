/**
******************************************************************************
* @file    infrared_drop.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   地测跌落传感器驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "infrared_drop.h"
#include "adc_user.h"
#include "stdlib.h"

#define DROP_MAX (RANK_DROP_MAX)

s32 DropData[DROP_MAX];
static const s32 DropRef[DROP_MAX] = {1110, 1110, 1110, 1110};
s32 DropInitData[DROP_MAX] = {0, 0, 0, 0};

void InfraredDrop_InitData(bool enable)
{
    static u32 cnt = 0;
    u8 i;
    
    if(enable)
    {
        if(cnt < 0x7FFFF)
        {
            for(i=0; i<DROP_MAX; i++)
            {
                s32 v = ADC_GetDropValue(i);
                if(abs(DropRef[i] - v) < 600)
                {
                    DropInitData[i] += v;
                }
                else
                {
                    DropInitData[i] += DropRef[i];
                }
            }
            cnt++;
        }
    }
    else if(cnt > 0)
    {
        for(i=0; i<DROP_MAX; i++)
        {
            DropInitData[i] /= cnt;
            DropData[i] = DropInitData[i];
        }
    }
}

u8 InfraredDrop_GetData(void)
{
    u8 data = 0, i;
    
    ADC_GetRef();
    
    for(i=0; i<DROP_MAX; i++)
    {
        s32 vol = DropData[i];
        DropData[i] = ADC_GetDropValue(i);
        vol = DropInitData[i] - (vol + DropData[i]) / 2;
        
        if(vol > 600 || vol < -800)
        {
            data |= 0x01 << i;
        }
    }
    
    return data;
}

