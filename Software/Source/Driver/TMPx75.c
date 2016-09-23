/**
******************************************************************************
* @file    TMPx75.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   温度检测芯片驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "TMPx75.h"
#include "i2c_user.h"
#include "global_defines.h"

#define I2Cx (&hi2c1)

#define TMPX75_ADDRESS7     0x90
#define ADDR_TEMPERATURE    0x00
#define ADDR_CONFIGURATION  0x01

#define TMP_DATA_BITS       9 // 默认配置，若更改则须对应

// Communication Function
#define WriteBytes(addr, pdata, num)    USER_I2C_Mem_Write(I2Cx, TMPX75_ADDRESS7, addr, I2C_MEMADD_SIZE_8BIT, (u8*)(pdata), num, 5)
#define WriteByte(addr, pdata)       WriteBytes(addr, pdata, 1)
#define ReadBytes(addr, pdata, num) USER_I2C_Mem_Read(I2Cx, TMPX75_ADDRESS7, addr, I2C_MEMADD_SIZE_8BIT, (u8*)(pdata), num, 5)
#define ReadByte(addr, pdata)       ReadBytes(addr, pdata, 1)

/**
* @brief  温度传感器初始化
* @param  None
* @retval None
*/
bool TMP_Init(void)
{
    u16 i, temperature;
    u8 data = 0x60;
    
    SET_ERR(DRV_ERR_TMP_I2C);
    
    if(__HAL_I2C_GET_FLAG(I2Cx, I2C_FLAG_BUSY))
    {
        return false;
    }
    
    if(ReadBytes(ADDR_TEMPERATURE, &temperature, 2) != HAL_OK)
    {
        // Error
        return false;
    }    
    
    CLEAR_ERR(DRV_ERR_TMP_I2C);
    
    WriteByte(ADDR_CONFIGURATION, &data);
    
    return true;
}

/**
* @brief  获得温度数据
* @param  pdata ：所指向数据缓冲区
* @retval None
*/
u8 TMP_GetTemperature(s32 *pdata)
{
    u8 tmp, ret;
    u8 data[2];
    ret = ReadBytes(ADDR_TEMPERATURE, data, 2);
    if(ret == HAL_OK)
    {        
        *pdata = data[0] << 8 | data[1];        
        *pdata = (*pdata * 1000) >> 8; // 取得温度1000倍
    }
    return ret;
}

/**
* @brief  获得温度传感器配置情况
* @param  pdata ：所指向数据缓冲区
* @retval None
*/
u8 TMP_GetConfig(u8 *pdata)
{
    return ReadByte(ADDR_CONFIGURATION, pdata);
}
