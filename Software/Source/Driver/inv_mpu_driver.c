/**
******************************************************************************
* @file    inv_mpu_driver.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   MPU6500 驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "inv_mpu_driver.h"
#include "spi.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"


static void MPU_GPIOConfig(void);
static void MPU_SPISelect(void);
static void MPU_SPIDeselect(void);

u8 INVMPU_WriteByte(u8 addr, u8 data);
u8 INVMPU_ReadByte(u8 addr);

/**
* @brief  MPU （6500）初始化
* @param  None
* @retval None
*/
void InvMPU_Driver_Init(void)
{
    u8 ret = 0;
    unsigned char more;
    long accel[3], quat[4], temperature;
    struct int_param_s int_param;
    
    assert_param(MPU_SPI == &hspi1);
    
    MPU_SPIDeselect();
    
    ret += mpu_init(&int_param);
    ret += mpu_set_sensors(INV_XYZ_GYRO | INV_XYZ_ACCEL);
    ret += mpu_set_sample_rate(1000);
    ret += mpu_set_gyro_fsr( GYRO_FSR);
    ret += mpu_set_accel_fsr( ACCEL_FSR);
    
    accel[0] = 46;
    accel[1] = 78;
    accel[2] = 0;
    mpu_set_accel_bias_6500_reg(accel);
    
    #ifdef USE_DMP

    dmp_load_motion_driver_firmware();
    dmp_enable_feature(DMP_FEATURE_6X_LP_QUAT | DMP_FEATURE_GYRO_CAL |
                    DMP_FEATURE_SEND_CAL_GYRO | DMP_FEATURE_SEND_RAW_ACCEL);
    dmp_set_fifo_rate(200);
    
    u8 offset[12] = {0,0, 0xd2, 0x2b, 0x00, 0x01, 0xba, 0xd8, 0x00, 0x00, 0x9f, 0xa0};
    mpu_write_mem(D_EXT_GYRO_BIAS_X, 4, &offset[0]);
    mpu_write_mem(D_EXT_GYRO_BIAS_Y, 4, &offset[4]);
    mpu_write_mem(D_EXT_GYRO_BIAS_Z, 4, &offset[8]);
    
    mpu_set_dmp_state(1);
    #endif
}

bool MPU6500_Test(void)
{
    return MPU6500_WHO_AM_I_VALUE == INVMPU_ReadByte(MPU6500_WHO_AM_I_ADDR);
}

/**
* @brief  MPU 片选使能
* @param  None
* @retval None
*/
static void MPU_SPISelect(void)
{
    __HAL_SPI_ENABLE(MPU_SPI);
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
}

/**
* @brief  MPU 片选失能
* @param  None
* @retval None
*/
static void MPU_SPIDeselect(void)
{
    HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
    __HAL_SPI_DISABLE(MPU_SPI);
}

/**
* @brief  Write MPU60x0 register
* @param  addr: register address, data: data to write
* @retval None
*/
u8 INVMPU_WriteBytes(u8 addr, u16 len, const u8* data)
{
    u8 i;
    
    MPU_SPISelect();
    if(HAL_SPI_Transmit(MPU_SPI, &addr, 1, 5) == HAL_OK)
    {
        HAL_SPI_Transmit(MPU_SPI, (u8*)data, len, 5);
    }
	MPU_SPIDeselect();
    return 0;
}

u8 INVMPU_WriteByte(u8 addr, u8 data)
{
	return INVMPU_WriteBytes(addr, 1, &data);
}

/**
* @brief  Read MPU60x0 register
* @param  addr: register address
* @retval single register data
*/
u8 INVMPU_ReadBytes(u8 addr, u16 len, u8* data)
{
    u8 i;
    
    MPU_SPISelect();
    addr |= 0x80;
    if(HAL_SPI_Transmit(MPU_SPI, &addr, 1, 5) == HAL_OK)
    {
        HAL_SPI_Receive(MPU_SPI, data, len, 5);
    }
	MPU_SPIDeselect();
	return 0;
}

u8 INVMPU_ReadByte(u8 addr)
{
    u8 data;
    INVMPU_ReadBytes(addr, 1, &data);
    return data;
}

void INVMPU_ReadAccel(MPUSensorData* data)
{
    unsigned long sensor_timestamp;
    short accel[3];
    mpu_get_accel_reg(accel, &sensor_timestamp);
    
    //芯片安装情况的不同影响这里赋值的顺序和正负，y轴为俯仰
    data->accel_y = accel[1];
    data->accel_x = -accel[0];
    data->accel_z = accel[2];
}

void INVMPU_ReadGyro(MPUSensorData* data)
{
    unsigned long sensor_timestamp;
    short gyro[3];
    mpu_get_gyro_reg(gyro, &sensor_timestamp);
    
    //芯片安装情况的不同影响这里赋值的顺序和正负，x轴为俯仰
    data->gyro_y = gyro[1];
    data->gyro_x = gyro[0];
    data->gyro_z = gyro[2];
}

void INVMPU_DmpReadFifo(MPUSensorData *data)
{
    short gyro[3], accel[3], sensors;
    unsigned long sensor_timestamp;
    unsigned char more;
    
    data->sensors = 0;
    
    dmp_read_fifo(gyro, accel, data->quat, &sensor_timestamp, &sensors, &more);
    
    if(sensors & INV_WXYZ_QUAT) data->sensors |= MPU_DATA_QUAT;
    
    //芯片安装情况的不同影响这里赋值的顺序和正负，x轴为俯仰
    if(sensors & INV_XYZ_GYRO)
    {
        data->sensors |= MPU_DATA_GYRO;
        data->gyro_y = gyro[0];
        data->gyro_x = -gyro[1];
        data->gyro_z = gyro[2];
    }
    
    //芯片安装情况的不同影响这里赋值的顺序和正负，y轴为俯仰
    if (sensors & INV_XYZ_ACCEL)
    {
        data->sensors |= MPU_DATA_ACCEL;
        data->accel_y = accel[0];
        data->accel_x = accel[1];
        data->accel_z = accel[2];
    }
}

u8 Offset[12] = {0,0, 0xd2, 0x2b, 0x00, 0x01, 0xba, 0xd8, 0x00, 0x00, 0x9f, 0xa0};


