/**
******************************************************************************
* @file    navipack_transport_layer.h
* @author  Jalon
* @date    2016.02.01
* @brief   通讯协议解包封包等相关声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#ifndef  __NAVIPACK_TRANSPORT_LAYER_H__
#define  __NAVIPACK_TRANSPORT_LAYER_H__

#include "navipack_def.h"

/** @defgroup PACK_FLAG_define 传输层打包模式标志
  * @{
  */
#define PACK_FLAG_BEGIN     0x01
#define PACK_FLAG_END       0x02
/**
  * @}
  */

bool TransportUnpacking(TransportFrame_Type *pframe, u8* buffer, u16 size, u8 data);
bool TransportPacking(TransportFrame_Type *pframe, u8* buffer, u16 size, u8 *in_buf, u16 len, u8 pack_flag);

#endif
