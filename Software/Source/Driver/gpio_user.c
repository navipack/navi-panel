/**
******************************************************************************
* @file    gpio_user.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   GPIO 相关驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "gpio_user.h"

void LedToggle(void)
{
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

void LidarPower(bool on)
{
    if( on )
    {
        HAL_GPIO_WritePin(RADAR_PWR_BUTTON_GPIO_Port, RADAR_PWR_BUTTON_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(RADAR_PWR_BUTTON_GPIO_Port, RADAR_PWR_BUTTON_Pin, GPIO_PIN_RESET);
    }
}

void UltrasonicPower(bool on)
{
    if( on )
    {
        HAL_GPIO_WritePin(UR_PWR_BUTTON_GPIO_Port, UR_PWR_BUTTON_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(UR_PWR_BUTTON_GPIO_Port, UR_PWR_BUTTON_Pin, GPIO_PIN_RESET);
    }
}

void DropPower(bool on)
{
    if( on )
    {
        HAL_GPIO_WritePin(DROP_POWE_EN_GPIO_Port, DROP_POWE_EN_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(DROP_POWE_EN_GPIO_Port, DROP_POWE_EN_Pin, GPIO_PIN_RESET);
    }
}

void ChassisMotorDriverEnable(bool on)
{
    //HAL_GPIO_WritePin(AB_nSLEEP_GPIO_Port, AB_nSLEEP_Pin, (GPIO_PinState)(enable));
    if( on )
    {
        HAL_GPIO_WritePin(SHDNB_12_GPIO_Port, SHDNB_12_Pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(SHDNB_12_GPIO_Port, SHDNB_12_Pin, GPIO_PIN_RESET);
    }
    
}

bool IsPickUp(void)
{
    return false;
}

bool GetCommMode(void)
{
    return HAL_GPIO_ReadPin(COMM_MODE_GPIO_Port, COMM_MODE_Pin) == GPIO_PIN_SET;
}
