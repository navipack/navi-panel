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

/** 
    *@碰撞模块结构体
    *{
*/
typedef struct contactTypeStruct
{
    double contact_r;   /**< 右测碰撞模块adc模块采样数值*/
    double contact_l;   /**< 左测碰撞模块adc模块采样数值*/
    double R_r;         /**< 右测碰撞模块所产生的电阻值大小*/
    double R_l;         /**< 左测碰撞模块所产生的电阻值大小*/
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
    int contact1,contact2;
    int i;
//    double a[10];
    for(i=0,contact1=0,contact2=0;i<32;i++)            //累计32次数据进行均值滤波
    {
        contact1 = contact1 + ADC_GetValue(CONTACT_DROP1);
        contact2 = contact2 + ADC_GetValue(CONTACT_DROP2);
        HAL_Delay(1);
    }
    contactData.contact_l = contact1 >> 5;
    contactData.contact_r = contact2 >> 5;
    contactData.R_l = (4096000 / contactData.contact_l) - 1000;    //分压电阻为1000欧姆
    contactData.R_r = (4096000 / contactData.contact_r) - 1000;    //分压电阻为1000欧姆
    
}

