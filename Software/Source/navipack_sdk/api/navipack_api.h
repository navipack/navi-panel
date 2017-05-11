/**
******************************************************************************
* @file    navipack_api.h
* @author  Jalon
* @date    2016.02.01
* @brief   通讯协议相关声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#ifndef  __NAVIPACK_API_H__
#define  __NAVIPACK_API_H__
/** @addtogroup MCU_SDK_API
* @{
*/

#include "navipack_session_layer.h"

bool NaviPack_Init(void);
bool NaviPack_TxProcessor(NavipackComm_Type *comm, NaviPack_HeadType *head);
bool NaviPack_RxProcessor(NavipackComm_Type *comm, u8 data);
bool Navipack_CheckLength(NaviPack_HeadType *head, u16 len);
bool Navipack_LockReg(u8 reg_id);
void Navipack_UnlockReg(u8 reg_id);

/**
* @}
*/
#endif
