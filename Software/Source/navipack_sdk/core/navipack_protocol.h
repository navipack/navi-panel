/**
******************************************************************************
* @file    navipack_protocol.h
* @author  Jalon
* @date    2016.06.16
* @brief   通讯协议会话层相关数据结构定义
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#ifndef __NAVIPACK_PROTOCOL_H__
#define __NAVIPACK_PROTOCOL_H__

#pragma pack(push, 1)

typedef struct
{
    u8 deviceAddr;
    u8 functionCode;
    u16 startAddr;
    u32 len;
}NacipackProtocolHeader;

typedef struct
{
    u16 ultrasound[8];
    u8 dropSensor;
    u16 irSensor;
    u8 collisionSensor;
    s16 angularPos;
    s32 leftEncoderPos;      ///<当前左边里程计的积分位置
    s32 rightEncoderPos;     ///<当前右边里程计的积分位置
    s32 lineVelocity;
    s32 angularVelocity;
    u8 chargeStatus;
    u8 batteryStatus;
    u8 pickupStatus;
    u16 errorState;
}ChassisStatusRegister;

typedef struct
{
    s32 lineVelocity;
    s32 angularVelocity;
}ChassisControlRegister;

typedef struct
{
    u16 ultrasoundPos;
    u16 ultrasoundPerspective;
}UltraSound_S;

typedef struct
{
    u16 dropSensorPos;
    u16 dropSensorPerspective;
}DropSensor_S;

typedef struct
{
    u16 irSensoPos;
    u16 irSensoPerspective;
}IrSensor_S;

typedef struct
{
    u16 collisionSensorAngle;
    u16 collisionSensorPerspective;
}CollisionSensorAngle_S;

typedef struct
{
    //超声波传感器
    UltraSound_S ultrasound[8];
    //防跌落传感器
    DropSensor_S dropsensor[8];
    //红外传感器
    IrSensor_S irSensor[16];
    //碰撞传感器
    CollisionSensorAngle_S collisionSensor[16];
}ChassisParamRegister;

#pragma pack(pop)

// Device Address
#define MCU_ADDRESS 0x12

// Function Code
#define MCU_CONTROL_REG_READ   0x01
#define MCU_CONTROL_REG        0x02
#define MCU_STATUS_REG         0x03
#define MCU_PARAM_REG          0x04
#define MCU_USER_REG_READ      0x05
#define MCU_USER_REG_WRITE     0x06

#ifdef CHASSIS_PLATFORM

#define NAVIPACK_SLAVE_ID MCU_ADDRESS

#define FUNC_ID_READ_CONTROL  MCU_CONTROL_REG_READ
#define FUNC_ID_WRITE_CONTROL MCU_CONTROL_REG
#define FUNC_ID_READ_STATUS   MCU_STATUS_REG
#define FUNC_ID_READ_CONFIG   MCU_PARAM_REG
#define FUNC_ID_READ_USER     MCU_USER_REG_READ
#define FUNC_ID_WRITE_USER    MCU_USER_REG_WRITE

typedef ChassisControlRegister NaviPack_CtrlType;
typedef ChassisStatusRegister NaviPack_StatusType;
typedef ChassisParamRegister NaviPack_ConfigType;
typedef NacipackProtocolHeader NaviPack_HeadType;

#endif

#endif
