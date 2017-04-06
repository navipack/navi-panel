/**
******************************************************************************
* @file    navipack_session_layer.h
* @author  Jalon
* @date    2016.06.16
* @brief   通讯协议会话层解析相关声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#ifndef __NAVIPACK_SESSION_LAYER_H__
#define __NAVIPACK_SESSION_LAYER_H__
/** @addtogroup Session_Layer
* @{
*/

#include "navipack_type.h"
#include "navipack_transport_layer.h"

#define CHASSIS_PLATFORM
#include "navipack_protocol.h"

#define REG_ID_STATUS   1
#define REG_ID_COTROL   2
#define REG_ID_CONFIG   3
#define REG_ID_USER     4
#define REG_ID_UID      5

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
    u32 FirmwareRespond;
}NavipackComm_Type;

bool NaviPack_SessionRxProcessor(NavipackComm_Type *comm, u8 data);
bool RegisterWrite(NaviPack_HeadType *head, u8 *reg, u32 size, u8 reg_id);
bool RegisterRead(NavipackComm_Type *comm, NaviPack_HeadType *head, u8 err_id, u8 *reg, u32 reg_size, u8 reg_id);
bool Navipack_TransportUnpacking(NavipackComm_Type *comm, u8 data);
bool Navipack_TransportPacking(NavipackComm_Type *comm, u8 *in_buf, u16 len, u8 pack_flag);

bool Navipack_TxCallback(u8* pbuf, u16 len);
bool Navipack_RxCallback(NavipackComm_Type *comm, NaviPack_HeadType *head);

/**
* @}
*/
#endif
