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

#include "svpwm_types.h"
#include "motor.h"

#define BasicTimer          (TIM7)

#define IMU_UPDATE_FREQ  500

// For GlobalParams
#define DRV_ERR_NONE            0
#define DRV_ERR_COMM_TIMEOUT    0x01
#define DRV_ERR_IMU             0x02
#define DRV_ERR_BATTERY_I2C     0x04
#define DRV_ERR_TMP_I2C         0x08
#define DRV_ERR_TILT            0x10
#define DRV_ERR_PICKUP          0x20
#define CHECK_ERR(_ERR) (GlobalParams.errStatus & (_ERR))
#define SET_ERR(_ERR) (GlobalParams.errStatus |= (_ERR))
#define CLEAR_ERR(_ERR) (GlobalParams.errStatus &= ~(_ERR))

#define COMM_UART   0
#define COMM_USB    1  

typedef struct GlobalParamsTypeStruct
{
    u8 commMode;
    u32 errStatus;
    u32 lastTimCnt;         ///< 计时 Count 保存
    u32 lastCaptureCnt;     ///< 计时溢出 Count 保存
    s32 temperature;        ///< 环境温度
    s32 temperature_in;     ///< 芯片内部温度
    
    s32 lineVelocity;
    s32 angularVelocity;
}GlobalParamsType;
extern GlobalParamsType GlobalParams;

extern MotorParamsTyp MotorParams[3];

#endif
/******************* (C) COPYRIGHT 2013 Haviea *****END OF FILE****/
