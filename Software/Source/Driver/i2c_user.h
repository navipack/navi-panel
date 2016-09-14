/**
******************************************************************************
* @file    i2c_user.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   I2C 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __I2C_USER_H__
#define __I2C_USER_H__

#include "stm32_lib.h"
#include "i2c.h"

HAL_StatusTypeDef USER_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, 
        uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef USER_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, 
        uint8_t *pData, uint16_t Size, uint32_t Timeout);

#endif
