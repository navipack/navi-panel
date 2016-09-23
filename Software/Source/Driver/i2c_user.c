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

/**
  * @brief  Read an amount of data in blocking mode from a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  * @note   此函数与HAL_I2C_Mem_Read函数的使用方法基本一致，但是加入对指定的i2c标志是否设置的判断
            并进行了超时时间的判断
  */
HAL_StatusTypeDef USER_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, 
        uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    u32 tick = 0;
    
    while(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
        HAL_Delay(1);
        tick++;
        if(tick > Timeout)
        {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_I2C_Mem_Read(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}

/**
  * @brief  Write an amount of data in blocking mode to a specific memory address
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  DevAddress Target device address
  * @param  MemAddress Internal memory address
  * @param  MemAddSize Size of internal memory address
  * @param  pData Pointer to data buffer
  * @param  Size Amount of data to be sent
  * @param  Timeout Timeout duration
  * @retval HAL status
  * @note   此函数与HAL_I2C_Mem_Write函数的使用方法基本一致，但是加入对指定的i2c标志是否设置的判断
            并进行了超时时间的判断
  */
HAL_StatusTypeDef USER_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, 
        uint8_t *pData, uint16_t Size, uint32_t Timeout)
{
    u32 tick = 0;
    
    while(__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BUSY) == SET)
    {
        HAL_Delay(1);
        tick++;
        if(tick > Timeout)
        {
            return HAL_TIMEOUT;
        }
    }
    
    return HAL_I2C_Mem_Write(hi2c, DevAddress, MemAddress, MemAddSize, pData, Size, Timeout);
}
