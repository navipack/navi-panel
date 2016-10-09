/**
******************************************************************************
* @file    contact_detection.c
* @author  Inmotion NaviPanel team
* @date    2016/09/19
* @brief   碰撞传感器驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "adc_user.h"
#include "contact_detection.h"
#include "tim_user.h"

#include "comm.h"

/** 
    *@碰撞模块结构体
    *{
*/
typedef struct contactTypeStruct
{
    u32 contact_r;   /**< 右测碰撞模块adc模块采样数值*/
    u32 contact_l;   /**< 左测碰撞模块adc模块采样数值*/
    u32 R_r;         /**< 右测碰撞模块所产生的电阻值大小*/
    u32 R_l;         /**< 左测碰撞模块所产生的电阻值大小*/
}contactType;
/**
    *}
*/
contactType  contactData;


/**
* @brief  碰撞传感器测量
* @param  None
* @retval None
*/
void Contact_detection(void)
{
    
    ADC_GetRef();
    u32 contact1,contact2;
    u32 i;
    
    if(!RunFlag.contact) return;
    RunFlag.contact = 0;
    
    for(i=0,contact1=0,contact2=0;i<8;i++)            //累计32次数据进行均值滤波
    {
        contact1 = contact1 + ADC_GetValue(CONTACT_DROP1);
        contact2 = contact2 + ADC_GetValue(CONTACT_DROP2);
        HAL_Delay(1);
    }
    contactData.contact_l = contact1 >> 3;
    contactData.contact_r = contact2 >> 3;
    contactData.R_l = (4096000 / contactData.contact_l) - 1000;    //分压电阻为1000欧姆
    contactData.R_r = (4096000 / contactData.contact_r) - 1000;    //分压电阻为1000欧姆
    
}

