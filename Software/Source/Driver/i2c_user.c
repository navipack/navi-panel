/**
******************************************************************************
* @file    i2c_user.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   I2C 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "i2c_user.h"

HAL_StatusTypeDef USER_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, 
        uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    u32 tick = 0;
    
    while(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
        //OSTimeDly(1);
        tick++;
        if(tick > Timeout)
        {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}

HAL_StatusTypeDef USER_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, 
        uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    u32 tick = 0;
    
    while(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
        //OSTimeDly(1);
        tick++;
        if(tick > Timeout)
        {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}
