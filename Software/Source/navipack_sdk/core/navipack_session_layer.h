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

#include "navipack_def.h"

bool NaviPack_SessionTxProcessor(NavipackComm_Type *comm, NaviPack_HeadType *head);
bool NaviPack_SessionRxProcessor(NavipackComm_Type *comm, u8 data);
bool RegisterWrite(NaviPack_HeadType *head, u8 *reg, u32 size, u8 reg_id);
bool RegisterRead(NavipackComm_Type *comm, NaviPack_HeadType *head, u8 err_id, u8 *reg, u32 reg_size, u8 reg_id);

bool Navipack_TxCallback(u8* pbuf, u16 len);
bool Navipack_RxCallback(NavipackComm_Type *comm, NaviPack_HeadType *head);

#endif
