/**
******************************************************************************
* @file    v_detection.c
* @author  Inmotion NaviPanel team
* @date    2016/09/20
* @brief   电源12V电压检测，当低于11.9V时会使指示灯闪烁
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "v_detection.h"

double supply_v = 0;

/**
* @brief  电源12V电压检测，当低于11.9V时会使指示灯闪烁
* @param  None
* @retval None
*/
void supply_voltage_detection (void)
{
    supply_v = HAL_ADC_GetValue(&hadc2) * 4.985 / ADC_GetRef() ;   
    if(supply_v < V_Alarm)
    {
        HAL_GPIO_TogglePin(V_Alarm_alarm_GPIO_Port, V_Alarm_Pin);
    }
    else
    {
        HAL_GPIO_WritePin(V_Alarm_alarm_GPIO_Port, V_Alarm_Pin, GPIO_PIN_RESET);
    }
}
