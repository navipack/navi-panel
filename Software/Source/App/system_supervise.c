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
#include "sensor_update.h"
#include "motion_control.h"
#include "TMPx75.h"
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
    SET_ERR(DRV_ERR_I2C);
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
    //static bool pickup_en;
    static bool ready = false;
    static u8 tmp_init_flag = 1;


	if(!RunFlag.supervise) return;
    RunFlag.supervise = 0;
    
    if(tmp_init_flag)
    {
        static u8 tmp_init_cnt = 0;
        if(TMP_Init())
        {
            tmp_init_flag = 0;
        }
        else if(tmp_init_cnt++ > 10)
        {
            tmp_init_flag = 0;
        }
    }
    
        
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
        SET_ERR(DRV_ERR_COMM_TIMEOUT);
    }

    if(!(UserReg.debug_flag & 0x01))
    {
        // 倾斜角度
        if(abs(g_SensorSystemStatus.fused_pitch_angle) > DEGREE(12)
            || abs(g_SensorSystemStatus.fused_roll_angle) > DEGREE(15))
        {
            SET_ERR( DRV_ERR_TILT );
        }
        else if(CHECK_ERR( DRV_ERR_TILT))
        {
            CLEAR_ERR( DRV_ERR_TILT );
        }
    }
    
    // 外部测温
//    if(!CHECK_ERR(DRV_ERR_TMP_I2C) && ++temp_cnt >= SUPERVISE_PREQ/4)
//    {
//        temp_cnt = 0;            
//        
//        TMP_GetTemperature(&GlobalParams.temperature);
//    }
    
    // 一秒间隔
    if(RunFlag.second)
    {
        RunFlag.second = 0;
    }

}
