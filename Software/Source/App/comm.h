/**
******************************************************************************
* @file    comm.h
* @author  Jalon
* @date    2016.02.01
* @brief   通讯协议解包封包等相关声明
* @attention Copyright (C) 2016 Inmotion
******************************************************************************
*/
#ifndef  __COMM_H__
#define  __COMM_H__

#include "stm32_lib.h"
#include "usbd_cdc_if.h"
#include "navipack_api.h"

#define TALK_TX_MEM_SIZE		   1024		  //收发内存区总大小
#define TALK_RX_MEM_SIZE		   1024		  //收发内存区总大小

typedef struct TalkFuncTypeStruct					   
{
    bool  firtpackageflag;
    bool  firstoutflag;

    u8   *ptxpackagebuf;				   //包数据发送指针

//    OS_MEM *pTxMem;
//    OS_MEM *pRxMem;

	u8 RxMemBuf[TALK_RX_MEM_SIZE];				  
	u8 TxMemBuf[TALK_TX_MEM_SIZE];
    
    NavipackComm_Type* comm;
    
//    UART_HandleTypeDef *comx;
//    OS_EVENT *pUsart_Lock;			     //串口锁
//    struct USART_FUNC_PARAM_STRUCT *func_param;

//	  OS_EVENT *pTxQ;
//	  OS_EVENT *pRxQ;

//    void *TxPool[TX_Q_SIZE];			 //指向发送队列消息的指针数组
//    void *RxPool[RX_Q_SIZE];			 //指向接收队列消息的指针数组
}TalkFuncType;

#pragma pack(push, 1)
typedef struct NavipackUserTypeStruct
{
    u8 debugFlag;
    u16 vkp;
    u16 vki;
    u16 wkp;
    u16 wki;
}NavipackUserType;
#pragma pack(pop)
    
extern NavipackComm_Type NavipackComm;
extern NavipackUserType UserReg;

bool Comm_Init(void);
bool Comm_PostTxEvent(NaviPack_HeadType *head);

void Comm_RxTask(void *p_arg);
void Comm_TxTask(void);
void Comm_BoostTask(void);

void Comm_RecvPackage(u8 data);

#endif
