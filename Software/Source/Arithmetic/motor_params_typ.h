/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : InternalParams.h
* Author             : xc 
* Date First Issued  : 21/11/07
* Description        : 内部变量
********************************************************************************/
#ifndef __INTERNAL_PARAMS_H
#define __INTERNAL_PARAMS_H

#include "stm32_lib.h"
#include "PID_regulators.h"
#include "tim.h"
#include "adc.h"

typedef struct RotorCurrentTypStruct
{
	s16 Iq;
	s16 Id;
}RotorCurrentTyp; 

typedef struct MotorParamsTypStruct
{
    //关键参数需要被赋值------------------------------------------//
    TIM_HandleTypeDef *ENC_TIMER;         //编码器时钟
    TIM_HandleTypeDef *PWM_TIMER;         //PWM输出时钟
    u16 encoder_gap_num;          //编码器线数
    u8  pair_num;                //极对数
    u8  encoder_dir;             //编码器方向
    u32 sequence_length;         //注入转换长度
    ADC_HandleTypeDef *CURR_ADC;     //电流采样 ADC
    
    //PID控制参数值-----------------------------------------------//
    s32 max_speed;
    u16 max_torque;
    s32 max_voltage;
    s32 turns_count;
    
    //编码器时钟定义-----------------------------------------------//
    s64 enc_dec_pluse;            
    s16 previous_cnt;
    s32 previous_overflow_cnt;
    u32 previous_basic_tim_cnt;
    volatile s32 enc_tim_overflow;///< 该变量指示UPDATE中断发生的个数
    bool bIs_First_Measurement; //编码器是否第一次测量
    
    u16 h_phase_a_offset;          //A相偏移量
    u16 h_phase_b_offset;          //B相偏移量

    //======反馈值======//
    RotorCurrentTyp present_current_dq;
    s32 present_speed;
    s32 acccumulated_distance;
    s32 accumulated_distance_remainder;
    s64 present_position;

	//电机初始化参数-----------------------------------------------//
	s16 alignment_angle;
    s32 dead_zone;
    s32 dead_zone_move;
}MotorParamsTyp;

#endif
