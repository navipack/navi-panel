/**
******************************************************************************
* @file    encoder.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   增量式编码器程序
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32_lib.h"
#include "Encoder.h"
#include "global_defines.h"
#include "speed_loop.h"
#include "FIR_Filter.h"
#include "math_lib.h"

#include "tim.h"

#define USE_ENCODER_0
#define USE_ENCODER_1

#define ENC_TIM_BASE_CLK  CKTIM_APB1

/* Private typedef -----------------------------------------------------------*/

/* PERIPHERAL SET-UP ---------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
s16 ENC_Calc_Rot_Speed(void);

/* Private variables ---------------------------------------------------------*/	

/**
* @brief  初始化编码器Channel0
* @param  None
* @retval None
*/
void EncInitForChannel0()
{
    u8 idx = 0;
    MotorParams[idx].turns_count = 0;
    MotorParams[idx].enc_tim_overflow = 0;
}

/**
* @brief  初始化编码器Channel1
* @param  None
* @retval None
*/
void EncInitForChannel1()
{
    u8 idx = 1;
    MotorParams[idx].turns_count = 0;
    MotorParams[idx].enc_tim_overflow = 0;
}

/**
* @brief  初始化编码器Channel2
* @param  None
* @retval None
*/
void EncInitForChannel2()
{
    u8 idx = 2;
    MotorParams[idx].turns_count = 0;
    MotorParams[idx].enc_tim_overflow = 0;
}

/**
* @brief  General Purpose Timer x set-up for encoder speed/position 
*         sensors//普通的定时器用作速度角度的测量(自解)
* @param  None
* @retval None
*/
void EncInit(u8 idx)
{
    switch(idx)
    {
    case 0: EncInitForChannel0();  break;
    case 1: EncInitForChannel1();  break;
    }
}

#ifdef _DEBUG
s64 EncCnt[2] = {0,0};
void EncoderDataAccumulator(u8 i, s64 delta_cnt)
{
    if(i < 2) EncCnt[i] += delta_cnt;
}
#else
#define EncoderDataAccumulator(...) ((void)0)
#endif

/**
* @brief  Compute return latest speed measurement 
* @param  None
* @retval Return motor speed in 0.1 Hz resolution. Since the encoder is
                   used as speed sensor, this routine will return the mechanical
                   speed of the motor (NOT the electrical frequency)
                   Mechanical frequency is equal to electrical frequency/(number 
                   of pair poles).
*/
void EncCalcRotSpeed(u8 enc_idx, s32* feedback_spd, s32* p_encoder_delta)
{
    s32 encoder_delta;
	s32 basic_tim_cnt, basic_tim_cnt2;
    s32 encoder_Overflow_cnt, encoder_Overflow_cnt2;
    s32 encoder_cnt, encoder_cnt2;
	s32 temp1;
	
	if (!MotorParams[enc_idx].bIs_First_Measurement)
	{
		// 1st reading of overflow counter
		encoder_Overflow_cnt = MotorParams[enc_idx].enc_tim_overflow;
		// 1st reading of encoder timer counter
		encoder_cnt = __HAL_TIM_GET_COUNTER(MotorParams[enc_idx].ENC_TIMER);
		// basic timer counter
		basic_tim_cnt = BasicTimer->CNT;
		
		// 2nd reading of overflow counter
		encoder_Overflow_cnt2 = MotorParams[enc_idx].enc_tim_overflow;
		// 2nd reading of encoder timer counter
		encoder_cnt2 = __HAL_TIM_GET_COUNTER(MotorParams[enc_idx].ENC_TIMER);
		// basic timer counter
		basic_tim_cnt2 = BasicTimer->CNT;
        
		if (encoder_Overflow_cnt != encoder_Overflow_cnt2)
		{
			//Compare sample 1 & 2 and check if an overflow has been generated right 
			//after the reading of encoder timer. If yes, copy sample 2 result in 
			//sample 1 for next process 
			encoder_cnt = encoder_cnt2;
			encoder_Overflow_cnt = encoder_Overflow_cnt2;
			basic_tim_cnt = basic_tim_cnt2;
		}
		
		//haux = encoder_cnt;
        encoder_delta = (s32)(encoder_Overflow_cnt - MotorParams[enc_idx].previous_overflow_cnt)
                        * (4*MotorParams[enc_idx].encoder_gap_num)
                        + (encoder_cnt - MotorParams[enc_idx].previous_cnt);

        EncoderDataAccumulator(enc_idx, encoder_delta);
        // 由于时基中断触发后才调用该函数，所以time_base_cnt记录的是时基TIM重装之后的差值，所以差值要加上时基周期才是真实时间
        // speed computation as delta angle * 1/(speed sampling time)
        temp1 = (s64)encoder_delta * ENC_TIM_BASE_CLK 
                / (basic_tim_cnt - MotorParams[enc_idx].previous_basic_tim_cnt + (ENC_TIM_BASE_CLK/ENCODER_SAMPLING_FREQ));
		MotorParams[enc_idx].previous_basic_tim_cnt = basic_tim_cnt;
	}   //is first measurement, discard it	    
	else
	{
        MotorParams[enc_idx].previous_basic_tim_cnt = BasicTimer->CNT;
		MotorParams[enc_idx].bIs_First_Measurement = false;
		temp1 = 0;
		MotorParams[enc_idx].enc_tim_overflow = 0;
		encoder_cnt = __HAL_TIM_GET_COUNTER(MotorParams[enc_idx].ENC_TIMER);
		// Check if Encoder_Timer_Overflow is still zero. In case an overflow IT 
		// occured it resets overflow counter and wPWM_Counter_Angular_Velocity 
		if (MotorParams[enc_idx].enc_tim_overflow != 0)
		{
			encoder_cnt = __HAL_TIM_GET_COUNTER(MotorParams[enc_idx].ENC_TIMER);
			MotorParams[enc_idx].enc_tim_overflow = 0;
		}
        encoder_Overflow_cnt = MotorParams[enc_idx].enc_tim_overflow;
        
        encoder_delta = encoder_cnt;
	}
    
	MotorParams[enc_idx].previous_cnt = encoder_cnt;
	MotorParams[enc_idx].previous_overflow_cnt = encoder_Overflow_cnt;
	
	*feedback_spd = temp1;
    *p_encoder_delta = encoder_delta;
}

/**
* @brief  Export the value of the smoothed motor speed computed in 
*         ENC_Calc_Average_Speed function  
* @param  None
* @retval Return motor speed in 0.1 Hz resolution. This routine 
          will return the average mechanical speed of the motor.
*/
void EncGetMachanicalSpeed(u8 enc_idx, s32* p_encoder_speed, s32* p_encoder_delta)
{	 
	s32 wtemp1, wtemp2;

	EncCalcRotSpeed(enc_idx, (s32*)&wtemp1, (s32*)&wtemp2);

	if(MotorParams[enc_idx].encoder_dir == 0)
	{
		*p_encoder_speed = wtemp1;
		*p_encoder_delta = wtemp2;
	}
	else
	{
		*p_encoder_speed = -wtemp1;
		*p_encoder_delta = -wtemp2;
	}
}

/**
* @brief  This function handles TIMx Update interrupt request.
          Encoder unit connected to TIMx (x = 2,3 or 4) 
* @param  None
* @retval None
* @return None
*/
void Encoder_IRQHandler(u8 index)
{
	if(__HAL_TIM_IS_TIM_COUNTING_DOWN(MotorParams[index].ENC_TIMER))
	{
		MotorParams[index].turns_count--;
        MotorParams[index].enc_tim_overflow--;
	}
	else
	{
		MotorParams[index].turns_count++;
        MotorParams[index].enc_tim_overflow++;
	}
}

/******************* (C) COPYRIGHT 2013 Haviea *****END OF FILE****/
