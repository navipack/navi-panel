/**
******************************************************************************
* @file    speed_loop.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   速度闭环相关处理
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "speed_loop.h"
#include "global_defines.h"
#include "FIR_Filter.h"
#include <math.h>
#include "sensor_update.h"
#include "motion_control.h"
#include "comm.h"
#include "adc_user.h"
#include "PID_regulators.h"
#include "AVG_filter.h"
#include "motor.h"

static PIDObjTyp SpeedLoopWPID;
static PIDObjTyp SpeedLoopVPID;
static CSpeedVW LoopTargetSpeed = {0, 0};

/* Private functions ---------------------------------------------------------*/
/* Private variable ----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

s32 MaxOut = SINGLE_MAX*2;

/**
* @brief  初始化电机PID
* @param  None
* @retval None
*/
void MotorPIDInit(void)
{
    PIDInit(&SpeedLoopVPID, 8000, 50, 0);
    PIDInit(&SpeedLoopWPID, 3000, 20, 0);
    
    SpeedLoopVPID.outabslimit = &MaxOut;
    SpeedLoopWPID.outabslimit = &MaxOut;
}

#ifdef _DEBUG
void GetSpeedKpi()
{
    if(UserReg.vkp != 0) SpeedLoopVPID.kp = UserReg.vkp;
    if(UserReg.vki != 0) SpeedLoopVPID.ki = UserReg.vki;
    if(UserReg.wkp != 0) SpeedLoopWPID.kp = UserReg.wkp;
    if(UserReg.wki != 0) SpeedLoopWPID.ki = UserReg.wki;
}
#else
#define GetSpeedKpi()
#endif

/**
* @brief  获得线速度
* @param  target_v: 目标线速度，用来预估检测值
* @retval 线速度，单位：mm/s
*/
s32 FMVASpeedFilterArray[2][32];
AvgFilterInt32Def FMVASpeedFilter[3] = {
    {FMVASpeedFilterArray[0], 32, 0, 0},
    {FMVASpeedFilterArray[1], 32, 0, 0},
};

s32 GetVelocity()
{
    s64 velocity;
    
    velocity = AVG_Filter_s32(&FMVASpeedFilter[0], 
        (MotorParams[0].present_speed + MotorParams[1].present_speed) / 2);

    return V_ENC_TO_MM(velocity); //脉冲每秒换算成毫米每秒

}

/**
* @brief  获得角速度
* @param  target_w: 目标角速度，用来预估检测值
* @retval 角速度，单位：倍乘角度，相关宏 DEGREE()
*/
s32 GetOmega()
{
    return AVG_Filter_s32(&FMVASpeedFilter[1], g_SensorSystemStatus.yaw_anglerate);
}

void SpeedLoop_SetTargetSpeed(CSpeedVW *s)
{
    LoopTargetSpeed = *s;
}

void SpeedLoop(void)
{
    GlobalParams.presentVW.sV = GetVelocity();
    GlobalParams.presentVW.sW = GetOmega();
    
    AngularVelocityController(
        LoopTargetSpeed.sV, LoopTargetSpeed.sW, 
        GlobalParams.presentVW.sV, GlobalParams.presentVW.sW);
}

/**
* @brief  角速度线速度环
* @param  TargetV:  目标线速度
* @param  TargetW:  目标角速度
* @param  velocity: 当前线速度
* @param  omega:    当前角速度
* @retval 
*/
s32 Left_debug = 0;
s32 Right_debug = 0;
void AngularVelocityController(s32 TargetV, s32 TargetW, s32 velocity, s32 omega)
{
    bool stop_flag;
    s32 outV, outA;
    s32 left_out, right_out;

    static bool loop_en = false;
    static s32 last_outV = 0;
    static s32 last_outA = 0;
    
    GetSpeedKpi();
    
    if(TargetW != 0 || TargetV != 0)
    {
        loop_en = true;
    }
    
    stop_flag = abs(omega) < DEGREE(1) && abs(velocity) < 10;
    // 停止策略
    if(abs(TargetW) < DEGREE(1) && abs(TargetV) < 10 && stop_flag)
    {
        outA = 0;
        outV = 0;
        loop_en = false;
    }
    
    if(loop_en)
    {
        // 线速度 PID
        outV = PIDRegulatorS32(TargetV<<8, velocity<<8, last_outV, &SpeedLoopVPID);
        
        // 角速度 PID
        outA = PIDRegulatorS32(TargetW>>3, omega>>3, last_outA, &SpeedLoopWPID);
        
        // 左右轮输出值限幅
        left_out = (outV - outA) / 2;
        if(left_out > SINGLE_MAX)
        {
            outV = 2*SINGLE_MAX + outA;
        }
        else if(left_out < -SINGLE_MAX)
        {
            outV = 2*(-SINGLE_MAX) + outA;
        }
                                                                                
        right_out = (outV + outA) / 2;
        if(right_out > SINGLE_MAX)
        {
            outV = 2*SINGLE_MAX - outA;             
        }
        else if(right_out < -SINGLE_MAX)
        {
            outV = 2*(-SINGLE_MAX) - outA;              
        }
    }
    
    left_out = (outV - outA) / 2; //限幅后计算输出值
    right_out = (outV + outA) / 2;
    
    last_outV = outV; //记录新的输出
    last_outA = outA;
    
    Motor_Output(0, left_out, stop_flag);
    Motor_Output(1, right_out, stop_flag);
}
