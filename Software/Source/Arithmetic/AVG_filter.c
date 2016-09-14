/**
******************************************************************************
* @file    AVG_Filter.c
* @author  Jalon
* @date    2016/06/29
* @brief   均值滤波器
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "AVG_filter.h"

/**
* @brief  清空滤波器存储值
* @param  filter: 滤波器
* @retval None
*/
void AVG_Filter_Clear_s32(AvgFilterInt32Def *filter)
{
	u16 i;
	for(i = 0 ; i < filter->tap ; i ++)
		filter->delay[i] = 0;	
}

/**
* @brief  滑动平均滤波器(int32)
* @param  filter: 滤波器
* @param  invar : 新值
* @retval 滤波结果值
*/
s32 AVG_Filter_s32(AvgFilterInt32Def *filter, s32 invar)
{
    filter->sum += invar - filter->delay[filter->index];
    filter->delay[filter->index++] = invar;
    
    if(filter->index >= filter->tap)
    {
        filter->index = 0;
    }

	return (filter->sum/filter->tap);
}
