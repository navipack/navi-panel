/**
******************************************************************************
* @file    system_supervise.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   系统监控
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include <stdlib.h>
#include <string.h>
#include "system_supervise.h"
#include "global_defines.h"
#include "control_params.h"
#include "sensor_update.h"
//#include "ShellyTalkInternal.h"
#include "motion_control.h"
#include "TMPx75.h"
//#include "lidar.h"
#include "comm.h"

#include "tim_user.h"
#include "adc_user.h"
#include "gpio_user.h"

#define SUPERVISE_PREQ  100

static u32 Heartbeat = 0;
#define HeartbeatTimeOut (Heartbeat > SUPERVISE_PREQ*300/1000) //300ms

/**
* @brief  心跳恢复重置
* @param  None
* @retval None
*/
void ResetHeartbeat(void)
{
    Heartbeat = 0;
    if(CHECK_ERR( DRV_ERR_COMM_TIMEOUT))
    {
        CLEAR_ERR(DRV_ERR_COMM_TIMEOUT);
        SetCarMotionEnable(true);
    }
}

/**
* @brief  系统监控数据初始化
* @param  None
* @retval None
*/
void SystemSuperviseInit(void)
{
    SET_ERR(DRV_ERR_IMU);
    SET_ERR(DRV_ERR_BATTERY_I2C);
    SET_ERR(DRV_ERR_COMM_TIMEOUT);
    SET_ERR(DRV_ERR_TILT);
}

/**
* @brief  系统监控 Task
* @param  None
* @retval None
*/
void SystemSuperviseTask(void)		//100Hz
{
    static u8  led_on_cnt = 0;
    static u8  led_on_times = 0;
    static u16 hang_cnt = 0;
    static bool pickup_en;
    static bool ready = false;
    static u8 last_collision = 0, last_drop;
    static bool stop;
    
    //BQ20ZX_Init();

    
    pickup_en = !IsPickUp();

	if(!RunFlag.supervise) return;
    RunFlag.supervise = 0;
        
    if(!CHECK_ERR(DRV_ERR_IMU))
    {
        if(RunFlag.led)
        {
            RunFlag.led = 0;
            
            LedToggle();
        }
        
        if(!ready)
        {
            LidarPower(true);
            ready = true;
        }
    }
        
    // 通讯心跳
    if(!HeartbeatTimeOut)
    {
        Heartbeat++;
    }
    else
    {
#ifdef _DEBUG
        if(UserReg.debugFlag & 0x01)
        {
            ResetHeartbeat();
        }
        else
#endif
        {
            SET_ERR(DRV_ERR_COMM_TIMEOUT);
            SetCarMotionEnable(false);
        }
    }

#ifdef _DEBUG
    if(!(UserReg.debugFlag & 0x01))
#endif
    {
        // 倾斜角度
        if(abs(g_SensorSystemStatus.fused_pitch_angle) > DEGREE(12)
            || abs(g_SensorSystemStatus.fused_roll_angle) > DEGREE(15))
        {
            SET_ERR( DRV_ERR_TILT );
            ChassisMotorDriverEnable(false);
            SetCarMotionEnable(false);
        }
        else if(CHECK_ERR( DRV_ERR_TILT))
        {
            CLEAR_ERR( DRV_ERR_TILT );
            ChassisMotorDriverEnable(true);
            SetCarMotionEnable(true);
        }
    }
    
    // 一秒间隔
    if(RunFlag.second)
    {
        RunFlag.second = 0;
    }

}
