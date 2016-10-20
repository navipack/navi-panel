/**
******************************************************************************
* @file    motion_control.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   运动控制模块，包含了一些加减速和初始化之类的算法
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "motion_control.h"
#include "global_defines.h"
#include "system_supervise.h"
#include "math.h"
#include "Queue.h"
#include "math_lib.h"
#include "gpio_user.h"
#include "sensor_update.h"
#include "comm.h"
#include "infrared_drop.h"
#include "tim_user.h"
#include "speed_loop.h"
#include "AVG_filter.h"

#define FREQ(cnt,hz)  (++(cnt) >= MOTION_PREQ/(hz))

extern u8 MotionCmdProcessFlag;

/////////////////// Local Variable ////////////////////////
static CSpeedVW TargetSpeed = {0, 0};
static u8 VW_Update = 0;

static bool CarMotionEnable = true;

void MotorParamsInit(void)
{
    MotorParams[0].acccumulated_distance = 0;
    MotorParams[0].accumulated_distance_remainder = 0;
    
    MotorParams[1].acccumulated_distance = 0;
    MotorParams[1].accumulated_distance_remainder = 0;
}

/**
* @brief  一阶滞后融合滤波
* @param  target_value   : 目标值
* @param  sample_value   : 采样值
* @param  factor         : 滞后程度
* @retval 结果
*/
s32 FirstFilterS32(s32 target_value, s32 sample_value, s16 factor)
{    
    return ((s64)target_value * factor + (s64)sample_value * (1000 - factor))/1000;    
}

/**
* @brief  获得线速度
* @param  target_v: 目标线速度，用来预估检测值
* @retval 线速度，单位：mm/s
*/
s32 FMVASpeedFilterArray[3][32];
AvgFilterInt32Def FMVASpeedFilter[3] = {
    {FMVASpeedFilterArray[0], 32, 0, 0},
    {FMVASpeedFilterArray[1], 32, 0, 0},
    {FMVASpeedFilterArray[2], 32, 0, 0},
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

/**
* @brief  底盘位姿更新
* @param  present_location        : 融合位姿数据
* @param  no_fusion_distance      : 不融合位姿数据
* @param  present_speed           : 速度
* @param  period                  : 周期时间，单位微秒
* @retval None
*/
#define W_NOISE_TH  (DEGREE(1)/4)
void CarLocationUpdate(CDistanceValue* distance, const CSpeedVW *present_speed, s32 period )
{
    s32 delta_theta;
    s32 delta_distance = 0;
    static s32 delta_theta_remainder;
    static s32 delta_distance_remainder;
    
    //位移
    delta_distance = present_speed->sV * period + delta_distance_remainder;
    delta_distance_remainder = delta_distance % 1000000;
    delta_distance /= 1000000;
    
    //角度变化，左转弯为角速度正方向
    if(abs(present_speed->sW) < W_NOISE_TH)
    {
        delta_theta = 0;
    }
    else
    {
        delta_theta = present_speed->sW * period + delta_theta_remainder;
        delta_theta_remainder = delta_theta % 1000000;
        delta_theta /= 1000000;
    }
    
    distance->theta += delta_theta;

    distance->theta %= DEGREE(360);
    if(distance->theta < 0)
	{
		distance->theta += DEGREE(360);
	}
    
    distance->distance += delta_distance;
}

/**
* @brief  设置VW模式的速度值
* @param  v: 线速度
* @param  w: 角速度
* @param  t: 运行时间
* @retval None
*/
void SetVWValue(s32 v, s32 w, u16 t)
{
//    if(v == 0 && w != 0 && abs(w) < 150)
//    {
//        w = ((w >> 8) | 0x01) * 150;
//    }
    
    TargetSpeed.sV = v;
    TargetSpeed.sW = w;
    //TargetSpeed.time = t * (MOTION_PREQ/1000);
    VW_Update = 1;
}

void SetCarMotionEnable(bool b)
{
    CarMotionEnable = b;
}


/**
* @brief  传感器触发的停止动作处理
* @param  target: 输出速度值
* @param  time_threshold : 保护超时门限
* @retval 是否处于危险保护状态
*/
bool DropAndCollisionSensorHandler(CSpeedVW *target, u16 time_threshold)
{
    static bool drop_stop = false, collision_stop = false, is_protect = false;
    static u8 last_collision = 0;
    static u16 time_cnt = 0;
    u8 drop, collision;
    
    // 跌落传感器数据
    //drop = 0;
    drop = InfraredDrop_GetData();
    //drop = UltrasonicDrop_GetData();
    
    // 碰撞传感器数据
    collision = 0;
    
    if(last_collision == collision)
    {
        if(collision != 0)
        {
            collision_stop = true;
            NavipackComm.status.collisionSensor  = collision;
        }
        else
        {
            collision_stop = false;
            NavipackComm.status.collisionSensor = 0;
        }
    }
    last_collision = collision;
    
    // 跌落
    NavipackComm.status.dropSensor = drop;
    drop_stop = drop != 0;
    
    // 保护模式的超时机制，防止一直保护
    if(is_protect && ++time_cnt > time_threshold)
    {
        time_cnt = 0;
        drop_stop = false;
        collision_stop = false;
    }
    
    if(drop_stop || collision_stop)
    {
        if(target->sV > 0)
        {
            is_protect = true;
            target->sV = -500;
        }
    }
    else
    {
        if(is_protect)
        {
            is_protect = false;
            target->sV = 0;
            target->sW = 0;
        }
    }
    
    return is_protect;
}

/**
* @brief  判断主动轮是否在转动
* @param  None
* @retval 是否转动
*/
bool IsWheelRest()
{
    if(MotorParams[0].present_speed == 0 && MotorParams[1].present_speed == 0 && //编码器输出判断
        MotorParams[0].present_current_dq.Iq < 100 && MotorParams[1].present_current_dq.Iq < 100) //电机输出电流判断，毫伏
    {
        return true;
    }
    return false;
}

/**
* @brief  底盘运动控制
* @param  None
* @retval None
*/
void ChassisMovingController()
{
    static bool is_protect = false;
    static CDistanceValue present_posture = {0,0};
    static CSpeedVW present_vw = {0,0};
    static CSpeedVW target_vw = {0,0};
    static NaviPack_StatusType* status = &NavipackComm.status;
    
    present_vw = GlobalParams.presentVW;
    
    // 车当前位姿更新
    CarLocationUpdate(&present_posture, &present_vw, 1000000/MOTION_PREQ);
    
    // 通讯反馈
    if(Navipack_LockReg(REG_ID_STATUS))
    {
        status->angularPos = DEGREE_TO_RADIAN(present_posture.theta);
        status->leftEncoderPos = MotorParams[0].acccumulated_distance;
        status->rightEncoderPos = MotorParams[1].acccumulated_distance;
        status->lineVelocity = present_vw.sV;
        status->angularVelocity = DEGREE_TO_RADIAN(present_vw.sW);
        Navipack_UnlockReg(REG_ID_STATUS);
    }

//#ifdef _DEBUG
//    if(UserReg.debug_flag & 0x02)
//    {
//        is_protect = false;
//    }
//    else if(FREQ(stop_cnt, 500))
//#else
//    if(FREQ(stop_cnt, 500))
//#endif
//    {
//        stop_cnt = 0;
//        is_protect = DropAndCollisionSensorHandler(&target_VW, 300); // 碰撞及跌落传感器触发刹车策略
//    }
    
    if(!is_protect)
    {
        if(VW_Update)
        {
            VW_Update = 0;
            target_vw = TargetSpeed;
        }
        
        if(!CarMotionEnable)
        {
            target_vw.sV = 0;
            target_vw.sW = 0;
        }
    }
    
    SpeedLoop_SetTargetSpeed(&target_vw);
}

/**
* @brief  运动控制 Task
* @param  p_arg: 参数
* @retval None
*/
void MotionCtrlTask(void)
{
    static u8 motor_enable_flag = 1, drop_init_flag = 1;
    static u16 drop_init_cnt = 0;
    
    if(!RunFlag.motion) return;
    RunFlag.motion = 0;
    
    if(motor_enable_flag)
    {
        motor_enable_flag = 0;
        MotorPIDInit();
        ChassisMotorDriverEnable( true );
    }
    
    if(drop_init_flag)
    {
        if(drop_init_cnt < 500)
        {            
            InfraredDrop_InitData(true);
            drop_init_cnt++;
            return;
        }
        else
        {
            InfraredDrop_InitData(false);
            drop_init_flag = 0;
        }
    }

    ChassisMovingController();
}
