/**
******************************************************************************
* @file    adc_user.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   ADC 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __ADC_USER_H__
#define __ADC_USER_H__

#include "stm32_lib.h"
#include "adc.h"

/**
  *@name ADC1的DMA相关宏定义
  *@{
*/
#define RANK_DROP1      0    /** 地测传感器所使用的转换通道1 */   
#define RANK_DROP2      1    /** 地测传感器所使用的转换通道2 */ 
#define RANK_DROP3      2    /** 地测传感器所使用的转换通道3 */ 
#define RANK_DROP4      3    /** 地测传感器所使用的转换通道4 */ 

#define RANK_REF        4    /** 采集参考电压的所使用的转换通道 */   
#define RANK_TEMP       5    /** 采集内部温度所使用的转换通道 */    

#define CONTACT_DROP1   6    /** 碰撞传感器所使用的转换通道6 */ 
#define CONTACT_DROP2   7    /** 碰撞传感器所使用的转换通道7 */ 

#define RANK_DROP_MAX   4    /** 地测传感器所使用的转换通道数最大值 */
#define RANK_MAX        8    /** 转换通道数最大值 */
/**
  *@}
*/

/**
  *@name ADC 计算相关常数
  *@{
*/
#define V25             1430     /** 1.43V */
#define AVG_SLOPE       43       /** 4.3 mV/°C */
#define ADC_FULL_VALUE  0x0FFF   /** ADC采集的溢出数值 */
#define ADC_FULL_V      3300     /** ADC采集的溢出电压 3.3V */
#define ADC_REF_V       (1200*ADC_FULL_VALUE/ADC_FULL_V) /** ADC采集的参考电压 1.20V */
/**
  *@}
*/

void ADC_Init(ADC_HandleTypeDef* hadc);
s32 ADC_GetRef(void);
s32 ADC_GetDropValue(u8 idx);
s32 ADC_GetValue(u8 rank);
#define VOL_SIZE        RANK_MAX
extern u16 ADC1_Buffer[VOL_SIZE];

#endif
