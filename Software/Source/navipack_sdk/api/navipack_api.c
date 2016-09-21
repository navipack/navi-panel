/**
******************************************************************************
* @file    navipack_api.c
* @author  Jalon
* @date    2016.07.01
* @brief   通讯协议接口，该文件在移植过程中需要根据实际情况更改，
           部分函数需要用户实现
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#include "navipack_api.h"
#include "navipack_session_layer.h"

// 引入需要的头文件
#include "comm_usart.h"
#include "comm.h"
#include "motion_control.h"
#include "system_supervise.h"
#include "global_defines.h"

u32 DataCount = 0;

/**
* @brief  初始化
* @retval 是否成功
*/
bool NaviPack_Init(void)
{
    // TODO: 用户可添加初始化处理
    return true;
}

/**
* @brief  通讯接收数据处理函数
* @param  comm : 通讯对象
* @param  data : 接收数据，1 byte
* @retval 是否成功处理了数据包
*/
bool NaviPack_RxProcessor(NavipackComm_Type *comm, u8 data)
{
    return NaviPack_SessionRxProcessor(comm, data);
}

/**
* @brief  接收到合法的寄存器请求时回调
* @param  comm : 通讯结构指针
* @param  head : 数据指针
* @retval 是否处理了请求
*/
bool Navipack_RxCallback(NavipackComm_Type *comm, NaviPack_HeadType *head)
{
    ResetHeartbeat();
    DataCount++;
    
    switch(head->functionCode)
    {
    case FUNC_ID_READ_STATUS:
    case FUNC_ID_READ_CONTROL:
    case FUNC_ID_READ_CONFIG:
    case FUNC_ID_READ_USER:
        Comm_PostTxEvent(head);
        break;
    case FUNC_ID_WRITE_CONTROL:
        // 获得接收到的新值
        SetVWValue(
            comm->control.lineVelocity,
            RADIAN_TO_DEGREE(comm->control.angularVelocity),
            0);
        break;
    case FUNC_ID_WRITE_USER:
        // 写用户寄存器
        RegisterWrite(head, (u8*)&UserReg, sizeof(UserReg), REG_ID_USER);
        break;
    default:
        return false;
    }
    return true;
}

/**
* @brief  通讯发送数据处理函数
* @param  comm : 通讯对象
* @param  head : 接收数据，单 byte
* @retval 是否成功处理了数据包
*/
bool NaviPack_TxProcessor(NavipackComm_Type *comm, NaviPack_HeadType *head)
{
    switch(head->functionCode)
    {
    case FUNC_ID_READ_USER:
        // 用户寄存器数据发送
        return RegisterRead(comm, head, 0, (u8*)&UserReg, sizeof(UserReg), REG_ID_USER);
    default:
        return NaviPack_SessionTxProcessor(comm, head);
    }
}

/**
* @brief  实际发送数据的函数
* @param  pbuf : 数据指针
* @param  len  : 数据长度
* @retval 是否成功发送
*/
bool Navipack_TxCallback(u8* pbuf, u16 len)
{
    // 实际数据发送
    if(GlobalParams.commMode == COMM_UART)
    {
        return CommUsart_SendData(pbuf, len);
    }
    else
    {
        return CDC_TransmitData(pbuf, len) == USBD_OK;
    }
}

/**
* @brief  寄存器操作上锁
* @param  reg_id : 寄存器 id，指示需要上锁的寄存器
* @retval 是否成功上锁
*/
bool Navipack_LockReg(u8 reg_id)
{
    // TODO: 上锁处理，若无多线程操作可留空
    switch(reg_id)
    {
    case REG_ID_STATUS:
        break;
    case REG_ID_COTROL:
        break;
    case REG_ID_CONFIG:
        break;
    case REG_ID_USER:
        break;
    default:
        return false;
    }
    
    return true;
}

/**
* @brief  寄存器操作解锁
* @param  reg_id : 寄存器 id，指示需要解锁的寄存器
* @retval None
*/
void Navipack_UnlockReg(u8 reg_id)
{
    // TODO: 解锁处理，若无多线程操作可留空
    switch(reg_id)
    {
    case REG_ID_STATUS:
        break;
    case REG_ID_COTROL:
        break;
    case REG_ID_CONFIG:
        break;
    case REG_ID_USER:
        break;
    default:
        break;
    }
}
