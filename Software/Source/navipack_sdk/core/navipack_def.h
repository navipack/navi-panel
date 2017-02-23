/**
******************************************************************************
* @file    navipack_def.h
* @author  Jalon
* @date    2016.06.16
* @brief   通讯协议内部数据类型定义
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#ifndef __NAVIPACK_DEF_H__
#define __NAVIPACK_DEF_H__

#include "navipack_type.h"

#define CHASSIS_PLATFORM
#include "navipack_protocol.h"

#define REG_ID_STATUS   0
#define REG_ID_COTROL   1
#define REG_ID_CONFIG   2
#define REG_ID_USER     3
#define REG_ID_UID      4

typedef struct
{
    bool recvFlag;      ///< 接收标志
    bool ctrlFlag;      ///< 转义符标志
    u8 offset;          ///< 当前buf位置
    u8 lastByte;        ///< 上次数据
    u8 checkSum;        ///< 校验和
    u32 errorCount;     ///< 错误统计
}TransportFrame_Type;

typedef struct
{
    // 需要初始化
    u8 *rxBuffer;   ///< 接收 Buffer 指针
    u8 *txBuffer;   ///< 发送 Buffer 指针
    u16 rxSize;     ///< 接收 Buffer 尺寸
    u16 txSize;     ///< 发送 Buffer 尺寸
    
    u16 rxDataLen;
    u16 txDataLen;
    TransportFrame_Type rxFrame;
    TransportFrame_Type txFrame;
    
    NaviPack_CtrlType control;  ///< 控制寄存器
    NaviPack_StatusType status; ///< 状态寄存器
    NaviPack_ConfigType config; ///< 参数寄存器
}NavipackComm_Type;

#endif
