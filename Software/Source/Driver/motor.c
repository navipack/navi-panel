/**
******************************************************************************
* @file    motor.c
* @author  Jalon
* @date    2016/09/05
* @brief   有刷直流电机 PWM 驱动相关实现
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "motor.h"
#include "global_defines.h"
#include "adc_user.h"


/**
  *@name 有刷直流电机 PWM相关宏定义
  *@{
*/
#define PWM(_v) (PWM_PERIOD*(_v)/1000)

MotorParamsTyp MotorParams[2] = 
{
    /*ENC_TIMER--PWM_TIMER--encoder_gap_num--pair_num--encoder_dir--sequence_length--CURR_ADC*/
    {&htim2,     &htim1,    ENCODER_GAP,   4,       0,          0,              &hadc1},
    {&htim3,     &htim1,    ENCODER_GAP,   4,       1,          0,              &hadc2},
};

void Motor_Init(void)
{
    MotorParams[0].dead_zone = PWM(16);
    MotorParams[0].dead_zone_move = PWM(0);
    MotorParams[1].dead_zone = PWM(16);
    MotorParams[1].dead_zone_move = PWM(0);
}

void Motor_Output(u8 idx, s32 out, u8 stop_flag)
{
    u16 hTimePhA = 0, hTimePhB = 0;
    
    // PWM
    if(stop_flag)
    {
        if(out > 0) out = out * (PWM_PERIOD - MotorParams[idx].dead_zone) / SINGLE_MAX + MotorParams[idx].dead_zone;
        else if(out < 0) out = out * (PWM_PERIOD - MotorParams[idx].dead_zone) / SINGLE_MAX - MotorParams[idx].dead_zone;
    }
    else
    {
        if(out > 0) out = out * (PWM_PERIOD - MotorParams[idx].dead_zone_move) / SINGLE_MAX + MotorParams[idx].dead_zone_move;
        else if(out < 0) out = out * (PWM_PERIOD - MotorParams[idx].dead_zone_move) / SINGLE_MAX - MotorParams[idx].dead_zone_move;
    }
    
    #if 0
    Left_debug = left_out;
    Right_debug = right_out;
    #endif
    
    if(out >= 0)
    {
        hTimePhA = out;
        hTimePhB = 0;
        
        if(hTimePhA > MAX_PWM)
            hTimePhA = PWM_PERIOD;
            
        if(hTimePhA < MIN_PWM)
            hTimePhA = 0;
    }
    else
    {
        hTimePhA = 0;
        hTimePhB = -out;
        
        if(hTimePhB > MAX_PWM)
            hTimePhB = PWM_PERIOD;
            
        if(hTimePhB < MIN_PWM)
            hTimePhB = 0;
    }
    
#if 1
    //赋值顺序影响电机转动方向
    if(idx == 0)
    {
        MotorParams[idx].PWM_TIMER->Instance->CCR1 = hTimePhA;
        MotorParams[idx].PWM_TIMER->Instance->CCR2 = hTimePhB;
    }
    else if(idx == 1)
    {
        MotorParams[idx].PWM_TIMER->Instance->CCR3 = hTimePhB;
        MotorParams[idx].PWM_TIMER->Instance->CCR4 = hTimePhA;
    }
#elif 0 //负载测试临时方法
    {
        #if 0 //前后
        static u32 cnt = 0,s = 0;
        if(s%2 == 0)
        {
            if(cnt++ >= 3*PWM_FREQ)
            {
                cnt = 0;
                if(s == 0)
                {
                    PWM1 = 0;
                    PWM2 = 0;
                    PWM3 = 0;
                    PWM4 = 0;
                    s++;
                }
                else if(s == 2)
                {
                    PWM1 = 0;
                    PWM2 = 0;
                    PWM3 = 0;
                    PWM4 = 0;
                    s++;
                }
            }
        }
        else
        {
            if(cnt++ >= 1*PWM_FREQ)
            {
                cnt = 0;
                if(s == 1)
                {
                    PWM1 = 0;
                    PWM2 = PWM_PERIOD*10/10;
                    PWM3 = 0;
                    PWM4 = PWM_PERIOD*10/10;
                    s++;
                }
                else
                {                
                    PWM1 = PWM_PERIOD*10/10;
                    PWM2 = 0;
                    PWM3 = PWM_PERIOD*10/10;
                    PWM4 = 0;
                    s = 0;
                }
            }
        }
        #elif 0 //缓加 PWM
        static u32 cnt = 0;
        cnt++;
        if(cnt >= 8*PWM_FREQ)
        {
            cnt = 0;
        }
        else if(cnt >= 5*PWM_FREQ)
        {
            PWM1 = 0;
            PWM2 = 0;
            PWM3 = 0;
            PWM4 = 0;
        }
        else
        {
            u32 t;
            t = (cnt/PWM_FREQ) * PWM_FREQ / 3;
            PWM1 = 0;
            PWM2 = t;
            PWM3 = 0;
            PWM4 = t;
        }
        #endif
        
        MotorParams[mt_idx].PWM_TIMER->Instance->CCR1 = PWM1;
        MotorParams[mt_idx].PWM_TIMER->Instance->CCR2 = PWM2;
        MotorParams[mt_idx].PWM_TIMER->Instance->CCR3 = PWM3;
        MotorParams[mt_idx].PWM_TIMER->Instance->CCR4 = PWM4;
    }
#endif
    

}

/**
* @brief  电机驱动芯片工作电流采样及计算
* @param  idx     : 电机编号
* @retval 电流值
*/
#define R_I_SENSE      110 //采样电阻 0.11欧

s16 Motor_CurrentValues(u8 idx)
{
	s32 wAux;
    s16 I;

    wAux = HAL_ADCEx_InjectedGetValue(MotorParams[idx].CURR_ADC, ADC_INJECTED_RANK_1);
    wAux = (s32)(MotorParams[idx].h_phase_a_offset) - wAux;          // 偏置电压换算 
    //采样运放放大2倍
    wAux = wAux * ADC_FULL_V/ADC_FULL_VALUE * 1000/R_I_SENSE / 2; // 电流值计算 mA 
    
	//Saturation of Ia 
    if (wAux < S16_MIN)
    {
		I = S16_MIN;
    }  
    else if (wAux > S16_MAX)
	{ 
		I = S16_MAX;
	}
	else
	{
		I = wAux;
	}
  
	return I;
}

/**
* @brief  电机驱动芯片工作电流采样初值计算
* @param  None
* @retval None
*/
void Motor_CurrentReadingCalibration()
{
    u32 sum[2] = {0, 0};
	u16 bIndex;
    
    //获得零电压偏移值
    for(bIndex=0; bIndex <16; bIndex++)
    {
        while(!__HAL_ADC_GET_FLAG(MotorParams[0].CURR_ADC, ADC_FLAG_JEOC)) { }
        __HAL_ADC_CLEAR_FLAG(MotorParams[0].CURR_ADC, ADC_FLAG_JEOC);
        sum[0] += HAL_ADCEx_InjectedGetValue(MotorParams[0].CURR_ADC, ADC_INJECTED_RANK_1);
        
        while(!__HAL_ADC_GET_FLAG(MotorParams[1].CURR_ADC, ADC_FLAG_JEOC)) { }
        __HAL_ADC_CLEAR_FLAG(MotorParams[1].CURR_ADC, ADC_FLAG_JEOC);
        sum[1] += HAL_ADCEx_InjectedGetValue(MotorParams[1].CURR_ADC, ADC_INJECTED_RANK_1);
    }
    
    MotorParams[0].h_phase_a_offset = sum[0] / bIndex;
    MotorParams[1].h_phase_a_offset = sum[1] / bIndex;
}
