/**
******************************************************************************
* @file    PID_regulators.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   PID 计算
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

/* Standard include ----------------------------------------------------------*/

#include "control_params.h"
#include "PID_regulator_params.h"
#include "PID_regulators.h"
#include "global_defines.h"
//#include "arm_math.h"

/**
* @brief  设置PID参数
* @param  p : 比例    
* @param  i : 积分
* @param  d : 微分
* @param  ppid : pid结构体
* @retval None
*/
void PIDSetParams(PIDObjTyp *ppid, u32 p, u32 i, u32 d)
{
	ppid->kp = p;
	ppid->ki = i;
	ppid->kd = d;
}

/**
* @brief  初始化PID参数
* @param  p : 比例    
* @param  i : 积分
* @param  d : 微分
* @param  ppid : pid结构体
* @retval None
*/
void PIDInit(PIDObjTyp *ppid, u32 p, u32 i, u32 d)
{
	PIDSetParams(ppid, p, i, d);
	ppid->remainder = 0;
}
/**
* @brief  增量式PID
* @param  aim  : 目标值
* @param  cur  : 当前值
* @param  out  : 上次输出
* @param  ppid : pid结构体
* @retval 本次输出
* @note   输入输出类型为 s32 （int）
*/
s32 PIDRegulatorS32(s32 aim, s32 cur, s16 out, PIDObjTyp *ppid)
{
	s64 out_value;
	s32 err, ret;
	
    err = aim - cur;

	out_value  = (s64)ppid->kp * (s64)(err - ppid->lasterr[0]);							//比例
	out_value += (s64)ppid->ki * (s64)(err);											//积分
    out_value += (s64)ppid->kd * (s64)(err - 2 * ppid->lasterr[0] + ppid->lasterr[1]);	//微分

	ppid->lasterr[1] = ppid->lasterr[0];
	ppid->lasterr[0] = err;

    out_value += ppid->remainder;			 //为减少精度损失，将余数累加
#if 0
	ret = out_value >> 16;

	ppid->remainder = out_value - (s64)ret * 0x10000;

	ret += out;
#else
    if(out_value < 0)
    {
        ret = (out_value >> 16) + 1 + out;
        ppid->remainder = (s32)out_value | 0xffff0000;
    }
    else
    {
        ret = (out_value >> 16) + out;
        ppid->remainder = (s32)out_value & 0x0000ffff;
    }
#endif

	//测试输出是否饱和，超上限
	if (ret > *ppid->outabslimit)	  
	{
		return (*ppid->outabslimit);
	}
    else if (ret < -*ppid->outabslimit) //超下限
	{
		return -(*ppid->outabslimit);
	}
    else 
	{
		return ret; //不超限。输出结果 houtput_32
	}
}
