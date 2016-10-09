/**
******************************************************************************
* @file    global_defines.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   一些全局定义
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __GLOBAL_DEFINES_H__
#define __GLOBAL_DEFINES_H__

#include "motor.h"
#include "motion_control.h"

#define BasicTimer          (TIM7)

#define IMU_UPDATE_FREQ  500

// For GlobalParams
#define DRV_ERR_NONE            0
#define DRV_ERR_COMM_TIMEOUT    0x01
#define DRV_ERR_IMU             0x02
#define DRV_ERR_I2C             0x04
#define DRV_ERR_TMP_I2C         0x08
#define DRV_ERR_TILT            0x10
#define DRV_ERR_PICKUP          0x20
#define CHECK_ERR(_ERR) (GlobalParams.err_status & (_ERR))
#define SET_ERR(_ERR) (GlobalParams.err_status |= (_ERR))
#define CLEAR_ERR(_ERR) (GlobalParams.err_status &= ~(_ERR))

#define COMM_UART   0
#define COMM_USB    1  

typedef struct GlobalParamsTypeStruct
{
    u8 comm_mode;
    u32 err_status;
    u32 last_tim_cnt;         ///< 计时 Count 保存
    u32 last_capture_cnt;     ///< 计时溢出 Count 保存
    s32 temperature;        ///< 环境温度
    s32 temperature_in;     ///< 芯片内部温度
    
    CSpeedVW presentVW;
}GlobalParamsType;
extern GlobalParamsType GlobalParams;

#endif
/******************* (C) COPYRIGHT 2013 Haviea *****END OF FILE****/
