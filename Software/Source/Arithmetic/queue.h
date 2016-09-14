/**
******************************************************************************
* @file    queue.h
* @author  Jalon
* @date    2016/08/19
* @brief   循环队列
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#ifndef __QUEUE_H__
#define __QUEUE_H__

#include  "stm32_lib.h"

typedef struct QueueTypeStruct
{
    u32 front;
    u32 rear;
    u32 itemSize;
    u32 itemCount;
    u8* pool;
}QueueType;

bool Queue_Init(QueueType* q, void* buffer, u32 buffer_size, u32 item_size);
bool Queue_Put(QueueType* q, void* pdata);
bool Queue_Get(QueueType* q, void* pdata);
bool Queue_Query(QueueType* q, void* pdata);

#endif
