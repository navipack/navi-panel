/**
******************************************************************************
* @file    navipack_session_layer.c
* @author  Jalon
* @date    2016.06.16
* @brief   通讯协议会话层解析相关函数
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#include <string.h>
#include "navipack_session_layer.h"
#include "navipack_api.h"

/**
* @brief  NaviPack 解包函数
* @param  comm : 通讯对象
* @param  data : 接收的数据，单 byte
* @retval 是否成功解包
*/
bool Navipack_TransportUnpacking(NavipackComm_Type *comm, u8 data)
{
    return TransportUnpacking(&comm->rxFrame, comm->rxBuffer, comm->rxSize, data);
}

/**
* @brief  NaviPack 打包函数
* @param  comm      : 通讯对象
* @param  in_buf    : 打包数据指针
* @param  len       : 打包数据长度
* @param  pack_flag : 打包模式 @ref PACK_FLAG_define 按 bit 设置
* @retval 打包错误则返回 false
*/
bool Navipack_TransportPacking(NavipackComm_Type *comm, u8 *in_buf, u16 len, u8 pack_flag)
{
    return TransportPacking(&comm->txFrame, comm->txBuffer, comm->txSize, in_buf, len, pack_flag);
}

/**
* @brief  执行写寄存器操作
* @param  head     : 数据头指针
* @param  reg      : 寄存器地址
* @param  reg_size : 寄存器尺寸
* @param  reg_id   : 寄存器 ID
* @retval 是否成功
*/
bool RegisterWrite(NaviPack_HeadType *head, u8 *reg, u32 reg_size, u8 reg_id)
{
    if(head->startAddr >= reg_size || head->startAddr + head->len > reg_size)
    {
        return false;
    }
    
    if(Navipack_LockReg(reg_id))
    {
        memcpy(reg + head->startAddr, (u8*)(head + 1), head->len);
        Navipack_UnlockReg(reg_id);
        return true;
    }
    
    return false;
}

/**
* @brief  执行读寄存器操作
* @param  comm     : 通讯对象
* @param  head     : 数据头指针
* @param  err_id   : 错误 ID
* @param  reg      : 寄存器地址
* @param  reg_size : 寄存器尺寸
* @param  reg_id   : 寄存器 ID
* @retval 是否成功
*/
bool RegisterRead(NavipackComm_Type *comm, NaviPack_HeadType *head, u8 err_id, u8 *reg, u32 reg_size, u8 reg_id)
{
    bool ret;
    
    if(head->startAddr >= reg_size || head->startAddr + head->len > reg_size)
    {
        return false;
    }
    
    head->functionCode |= 0x80;
    
    if(Navipack_LockReg(reg_id))
    {
        ret = Navipack_TransportPacking(comm, (u8*)head, sizeof(NaviPack_HeadType), PACK_FLAG_BEGIN);
        if(!ret) return false;
        ret = Navipack_TransportPacking(comm, reg + head->startAddr, head->len, 0);
        if(!ret) return false;
        ret = Navipack_TransportPacking(comm, &err_id, 1, PACK_FLAG_END);
        if(!ret) return false;
        comm->txDataLen = comm->txFrame.offset;
        
        Navipack_UnlockReg(reg_id);
        
        return Navipack_TxCallback(comm->txBuffer, comm->txDataLen);
    }
    
    return false;
}

/**
* @brief  通讯接收数据解析函数
* @param  comm : 通讯结构指针
* @param  head : 数据指针
* @param  len  : 数据长度
* @retval None
*/
static void RxProcessor(NavipackComm_Type *comm, NaviPack_HeadType *head, u16 len)
{
    if(head->deviceAddr == NAVIPACK_SLAVE_ID)
    {
        if(head->len != len - sizeof(NaviPack_HeadType))
        {
            return;
        }
        
        switch(head->functionCode)
        {
        case FUNC_ID_WRITE_CONTROL:
            if(!RegisterWrite(head, (u8*)&comm->control, sizeof(comm->control), REG_ID_COTROL))
            {
                return;
            }
            break;
        }
        
        Navipack_RxCallback(comm, head);
    }
}

/**
* @brief  通讯接收数据处理函数
* @param  comm : 通讯对象
* @param  data : 接收数据，单 byte
* @retval 是否成功处理了数据包
*/
bool NaviPack_SessionRxProcessor(NavipackComm_Type *comm, u8 data)
{   
    if(Navipack_TransportUnpacking(comm, data))
    {
        //解包处理成功
        comm->rxDataLen = comm->rxFrame.offset;
        
        RxProcessor(comm, (NaviPack_HeadType*)comm->rxBuffer, comm->rxDataLen);
        return true;
    }
    return false;
}

/**
* @brief  通讯发送数据处理函数
* @param  comm : 通讯对象
* @param  head : 接收数据，单 byte
* @retval 是否成功处理了数据包
*/
bool NaviPack_SessionTxProcessor(NavipackComm_Type *comm, NaviPack_HeadType *head)
{
    switch(head->functionCode)
    {
    case FUNC_ID_READ_STATUS:
        return RegisterRead(comm, head, 0, (u8*)&comm->status, sizeof(comm->status), REG_ID_STATUS);
    case FUNC_ID_READ_CONTROL:
        return RegisterRead(comm, head, 0, (u8*)&comm->control, sizeof(comm->control), REG_ID_COTROL);
    case FUNC_ID_READ_CONFIG:
        return RegisterRead(comm, head, 0, (u8*)&comm->config, sizeof(comm->config), REG_ID_CONFIG);
    case FUNC_ID_WRITE_CONTROL:
        break;
    default:
        return false;
    }
    
    return true;
}
