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

#include "navipack_def.h"

#define REG_ID_STATUS   0
#define REG_ID_COTROL   1
#define REG_ID_CONFIG   2
#define REG_ID_USER     3

bool NaviPack_Init(void);
bool NaviPack_TxProcessor(NavipackComm_Type *comm, NaviPack_HeadType *head);
bool NaviPack_RxProcessor(NavipackComm_Type *comm, u8 data);
bool Navipack_LockReg(u8 reg_id);
void Navipack_UnlockReg(u8 reg_id);

#endif
