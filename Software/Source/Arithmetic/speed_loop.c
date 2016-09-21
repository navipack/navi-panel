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
#include "control_params.h"
#include "current_params.h"
#include "global_defines.h"
#include "FIR_Filter.h"
#include <math.h>
#include "sensor_update.h"
#include "motion_control.h"
#include "comm.h"
#include "adc_user.h"
#include "PID_regulators.h"

#include "motor.h"

extern u8 MotionCmdProcessFlag;
//CMotionXYCmdMsg PreMotionCmdMsg;

static PIDObjTyp SpeedLoopWPID;
static PIDObjTyp SpeedLoopVPID;

/* Private functions ---------------------------------------------------------*/
/* Private variable ----------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

s32 MaxOut = SINGLE_MAX*2;
void MotorPIDInit(void)
{
    static s32 max = PWM_PERIOD;
    
    PIDInit(&SpeedLoopVPID, 8000, 50, 0);
    PIDInit(&SpeedLoopWPID, 3000, 20, 0);
    
    SpeedLoopVPID.outabslimit = &MaxOut;
    SpeedLoopWPID.outabslimit = &MaxOut;
    
    MotorParams[0].LoopMode = SPEED_LOOP;
    MotorParams[1].LoopMode = SPEED_LOOP;    
    
    if(MotorParams[0].LoopMode == SPEED_TORQUE_LOOP)
    {
        MotorParams[0].PIDTorque.kp = 1000;
        //MotorParams[0].PIDTorque.ki = 150;
        MotorParams[0].PIDTorque.ki = 10;
        MotorParams[0].PIDTorque.kd = 0;
        MotorParams[0].PIDTorque.outabslimit = &max;
        
        MotorParams[1].PIDTorque.kp = 1000;
        //MotorParams[1].PIDTorque.ki = 150;
        MotorParams[1].PIDTorque.ki = 10;
        MotorParams[1].PIDTorque.kd = 0;
        MotorParams[1].PIDTorque.outabslimit = &max;
        
        SpeedLoopVPID.kp = 13000;
        SpeedLoopVPID.ki = 20;
        SpeedLoopVPID.kd = 0;
        SpeedLoopWPID.kp = 15000;
        SpeedLoopWPID.ki = 110;
        SpeedLoopWPID.kd = 0;
    }
    else
    {
        MotorParams[0].PIDTorque.kp = 15000;
        MotorParams[0].PIDTorque.ki = 100;
        MotorParams[0].PIDTorque.kd = 1;
        MotorParams[0].PIDTorque.outabslimit = &max;
        
        MotorParams[1].PIDTorque.kp = 15000;
        MotorParams[1].PIDTorque.ki = 100;
        MotorParams[1].PIDTorque.kd = 1;
        MotorParams[1].PIDTorque.outabslimit = &max;
    }
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
* @brief  角速度线速度环
* @param  TargetV:  目标线速度，用来预估检测值
* @param  TargetW:  目标角速度，用来预估检测值
* @param  velocity: 采集的线速度
* @param  omega:    采集的角速度
* @retval 
*/
s32 Left_debug = 0;
s32 Right_debug = 0;
void AngularVelocityController(s32 TargetV, s32 TargetW, s32 velocity, s32 omega)
{
    bool stop_flag;
    s32 outV, outA;
    s32 left_out, right_out;
    s32 left_dead_zone, right_dead_zone;

    static s32 last_outV = 0;
    static s32 last_outA = 0;
    
    GetSpeedKpi();
    
    stop_flag = abs(omega) < DEGREE(1) && abs(velocity) < 15;
        // 停止策略
    if(abs(TargetW) < DEGREE(1) && abs(TargetV) < 15 && stop_flag)
    {
        outA = 0;
        outV = 0;
    }
    else
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
