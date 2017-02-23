/**
******************************************************************************
* @file    navipack_transport_layer.c
* @author  Jalon
* @date    2016.02.01
* @brief   通讯协议传输层解包封包等相关函数
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#include "navipack_transport_layer.h"

//传输层协议标志
#define FRAMECTRL 0xA5
#define FRAMEHEAD 0xAA
#define FRAMETAIL 0x55

/**
* @brief  传输层解包函数
* @param  pframe : 数据帧对象
* @param  buffer : 解包结果存储缓冲区
* @param  size   : 解包缓冲区尺寸
* @param  data   : 接收的数据，单 byte
* @retval 是否成功解包
*/
bool TransportUnpacking(TransportFrame_Type *pframe, u8* buffer, u16 size, u8 data)
{
    if(data == FRAMEHEAD && pframe->lastByte == FRAMEHEAD)
    {
        //复位
        pframe->offset = 0;
        pframe->recvFlag = true;
        pframe->checkSum = 0;
        return false;
    }
    
    if(pframe->offset >= size) //当接收的数据长度超过接收SIZE
    {
        //复位
        pframe->offset = 0;
        pframe->recvFlag = true;
        pframe->checkSum = 0;
        pframe->errorCount++;
        return false;
    }

    if( (data == FRAMETAIL) && (pframe->lastByte == FRAMETAIL) && (pframe->recvFlag) )
    { 
        //收到结束符
        pframe->offset -= 2;
        pframe->checkSum -= (FRAMETAIL + buffer[pframe->offset]);

        if(pframe->checkSum == buffer[pframe->offset])
        {                                       
            pframe->recvFlag = false;
            return true;
        }
        else
        {
            pframe->offset = 0;
            pframe->checkSum = 0;
            pframe->recvFlag = false;
            pframe->errorCount++;
            return false;
        }
    }

    if(pframe->recvFlag)
    {
        if(pframe->ctrlFlag)
        {
            if( (data == FRAMEHEAD) || (data == FRAMETAIL) || (data == FRAMECTRL) )
            {
                buffer[pframe->offset++] = data;
                pframe->ctrlFlag = false;
                pframe->checkSum += data;
                data = FRAMECTRL;
            }
            else
            {
                pframe->offset = 0;
                pframe->checkSum = 0;
                pframe->recvFlag = false;
            }
        }
        else
        {
            if(data == FRAMECTRL)
            {
                pframe->ctrlFlag = true;
            }
            else
            {
                buffer[pframe->offset++] = data;
                pframe->checkSum += data;
            }
        }
    }

    pframe->lastByte = data;

    return false;
}

/**
* @brief  传输层打包函数
* @param  pframe    : 数据帧对象
* @param  buffer    : 打包结果存储缓冲区
* @param  size      : 打包缓冲区尺寸
* @param  in_buf    : 打包数据指针
* @param  len       : 打包数据长度
* @param  pack_flag : 打包模式 @ref PACK_FLAG_define 按 bit 设置
* @retval 打包错误则返回 false
*/
bool TransportPacking(TransportFrame_Type *pframe, u8* buffer, u16 size, u8 *in_buf, u16 len, u8 pack_flag)
{
    u16 i;

    if((pack_flag & PACK_FLAG_BEGIN) != 0)
    {
        pframe->offset = 0;
        pframe->checkSum = 0;
        
        if(len > size - 5)      //当发送的数据长度超过发送SIZE - 包头字节数 - 包尾字节数 - 校验位字节数
            return false;
        
        buffer[pframe->offset++] = FRAMEHEAD;      //加头
        buffer[pframe->offset++] = FRAMEHEAD;
        
    }
    else
    {
        if(len + pframe->offset > size - 3)      //当发送的数据长度超过发送SIZE - 包尾字节数 - 校验位字节数
            return false;
    }

    for (i = 0; i < len; i++)
    {
        if( (*in_buf == FRAMECTRL) || (*in_buf == FRAMEHEAD)|| (*in_buf == FRAMETAIL) )
        {
            buffer[pframe->offset++] = FRAMECTRL;
            if((len - i) + pframe->offset > size - 3)    //溢出判断
                return false;
        }
        buffer[pframe->offset++] = *in_buf;
        pframe->checkSum += *in_buf;
        in_buf++;
    }

    if((pack_flag & PACK_FLAG_END) != 0)
    {
        //校验和
        if ( (pframe->checkSum == FRAMECTRL) || (pframe->checkSum == FRAMEHEAD) || (pframe->checkSum == FRAMETAIL) )
        {
            buffer[pframe->offset++] = FRAMECTRL;
            if(pframe->offset > size - 3)    //溢出判断
                return false;
        }
        buffer[pframe->offset++] = pframe->checkSum;

        buffer[pframe->offset++] = FRAMETAIL;      //加尾
        buffer[pframe->offset++] = FRAMETAIL;
    }

    return true;
}
