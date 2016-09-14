/**
******************************************************************************
* @file    FIR_filter.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   FIR滤波器
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "FIR_Filter.h"

/**
* @brief   清空延迟量
* @param   filter:FIR滤波器
* @retval  None  
*/
void FIR_Filter_int_Clear(FirFilterIntDef *filter)
{
	u32 i;
	for(i = 0 ; i < filter->tap ; i ++)
		filter->delay[i] = 0;	
}

/**
* @brief   N阶FIR滤波器
* @param   None
* @retval  None  
*/
s32 FIR_Filter_int(FirFilterIntDef *filter, s32 invar) 
{
	s64 sumnum = 0;
	s32 i, *pdelay;
	const s32 *pfactor;
	pdelay = filter->delay;
	pfactor = filter->factor;
	i = filter->tap - 1;
	sumnum = 0;
	while(i--)
	{
		*pdelay = *(pdelay + 1);
		sumnum += ((s64)*pdelay) * ((s64)*pfactor);
		pdelay ++;
		pfactor ++;
	}
	*pdelay	= invar;
	sumnum += ((s64)*pdelay) * ((s64)*pfactor);
	sumnum >>= 12;
	return sumnum;
}
