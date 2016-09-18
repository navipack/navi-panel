/**
******************************************************************************
* @file    ultrasonic.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   超声波模块驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "ultrasonic.h"
#include "tim_user.h"
#include "comm.h"
#include "gpio_user.h"

#define HALF_V_SOUND 170 //音速除以2 = 170m/s

#define DIST_MAX    8

static void Ultrasonic_detection(void);
    
typedef struct UltrasonicDataTypeStruct
{
    u32 lastTimCnt;
    u32 lastCaptureCnt;
    u8 currentChannel;
}UltrasonicDataType;

static u8 StartPulse = 0;
UltrasonicDataType UltrasonicData;

static u8 DebugChannel = 0;

// 通道转换表
static const u8 ChannelTable[DIST_MAX] = {7,6,4,1,0,2,3,5};
static u8 DetectionChannel[DIST_MAX] = {0};
/**
* @brief  通道切换
* @param  channel: 通道号
* @retval None
*/
static u8 SelectChannel( u16 channel )
{
    assert_param(channel < DIST_MAX);
    
    HAL_GPIO_WritePin(OE1_GPIO_Port, OE1_Pin, GPIO_PIN_SET);
    
    channel = ChannelTable[channel];    
    
    HAL_GPIO_WritePin(S0_GPIO_Port, S0_Pin, (GPIO_PinState)(channel & 0x01));
    channel = channel >> 1;
    HAL_GPIO_WritePin(S1_GPIO_Port, S1_Pin, (GPIO_PinState)(channel & 0x01));
    channel = channel >> 1;
    HAL_GPIO_WritePin(S2_GPIO_Port, S2_Pin, (GPIO_PinState)(channel & 0x01));
    
    HAL_GPIO_WritePin(OE1_GPIO_Port, OE1_Pin, GPIO_PIN_RESET);
    return 0;
}

/**
* @brief  初始化
* @param  None
* @retval None
*/
void Ultrasonic_Init(void)
{
    u8 i;
    
    SelectChannel(0);
    
    Ultrasonic_detection();
 
}

/**
* @brief  控制触发信号电平
* @param  on: true为高电平 false为低电平
* @retval None
*/
static void Trigger(bool on)
{
    if(on)
    {
        HAL_GPIO_WritePin(US_TRIGGER_GPIO_Port, US_TRIGGER_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(US_TRIGGER_GPIO_Port, US_TRIGGER_Pin, GPIO_PIN_RESET);
    }
}


void UltrasonicFeedbackData(u8 cha, u16 data)
{
    if(cha < 8)
    {
        //SetUltrasonicData(cha, data);
        //if(NaviPack_LockReg(REG_ID_STATUS))
        {
            NavipackComm.status.ultrasound[cha] = data;
            //NaviPack_UnlockReg(REG_ID_STATUS);
        }
    }
}

void UltrsonicTrigTask(void)
{
    static u8 channel;
    static u32 last_tick = 0;
    static bool is_trigger_on = false;
    
    u8 err;
    UltrasonicDataType *data;
    
    if(is_trigger_on && HAL_GetTick() - last_tick > 0)
    {
        is_trigger_on = false;
        Trigger(false);
    }
    
    if(!RunFlag.us) return;
    RunFlag.us = 0;
        
    // 关外部中断
    EXTI->IMR &= ~US_ECHO_Pin;
    
    // 超时数据处理
    if(StartPulse)
    {
        UltrasonicFeedbackData(channel, 0xFFFF);
    }
    StartPulse = 1;
    
    // 通道切换
    //channel = (channel+3) & 0x07;
    channel = (channel+2) % 5;
    
    //channel = DebugChannel;
    SelectChannel(channel);
    UltrasonicData.currentChannel = channel;
    
    // 开外部中断
    __HAL_GPIO_EXTI_CLEAR_IT(US_ECHO_Pin);
    EXTI->IMR |= US_ECHO_Pin;
    
    // Trigger
    last_tick = HAL_GetTick();
    Trigger(true);
    is_trigger_on = true;
}

/***********************************************************
* Description   : 根据中断间隔时间计算测量距离
* Input         : 
* Output        : 
* Return        : 
************************************************************/
static void Ultrasonic_DistanceCalc(UltrasonicDataType *data, u8 pin_value)
{
    u32 pluse_cnt;
    u32 distance;
    
    pluse_cnt = GetIntervalCnt(&data->lastCaptureCnt, &data->lastTimCnt);
    
    if(pin_value == 0)
    {
        distance = pluse_cnt * HALF_V_SOUND / (BASIC_TIM_CNT_FREQ/1000); //距离，毫米
        if(distance > 0x0FFFF)
        {
            distance = 0x0FFFF;
        }
        else if(distance == NavipackComm.status.ultrasound[data->currentChannel])
        {
            distance ^= 0x0001;
        }
        UltrasonicFeedbackData(data->currentChannel, distance);
    }
    
    StartPulse = pin_value;
}

/***********************************************************
* Description   : 超声波中断
* Input         : 
* Output        : 
* Return        : 
************************************************************/
void Ultrasonic_IRQHandler(void)
{
    Ultrasonic_DistanceCalc(&UltrasonicData, HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin));
}

/**
* @brief  超声波自检
* @param  None
* @retval None
*/
static void Ultrasonic_detection(void)
{
    static u8 channel = 0;
    u8 a;
    u32 tick;
    
    // 关外部中断
    EXTI->IMR &= ~US_ECHO_Pin;

    UltrasonicPower(true);
    
    HAL_Delay(55);
   
    for(a=0; a < DIST_MAX; a++)
    {
        while(channel > 4)
        {
            channel = (channel+1) & 0x07;
        }
        SelectChannel(channel);

        Trigger(true);
        HAL_Delay(1);
        Trigger(false);
        
        tick = HAL_GetTick();
        
        while(HAL_GetTick() - tick < 30)
        {
            if(HAL_GPIO_ReadPin(US_ECHO_GPIO_Port, US_ECHO_Pin) == GPIO_PIN_SET)
            {
                DetectionChannel [a] = 1;
                break;
            }
        }
        
        channel = (channel+1) & 0x07;
        NavipackComm.status.ultrasound[a] = 0xFFFF;
    }
    
    // 开外部中断
    __HAL_GPIO_EXTI_CLEAR_IT(US_ECHO_Pin);
    EXTI->IMR |= US_ECHO_Pin;
}
