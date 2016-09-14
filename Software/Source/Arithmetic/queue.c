/**
******************************************************************************
* @file    queue.c
* @author  Jalon
* @date    2016/08/19
* @brief   循环队列
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/
#include "queue.h"
#include <string.h>

bool Queue_Init(QueueType* q, void* pool, u32 buffer_size, u32 item_size)
{
    if(pool == NULL || item_size == 0 || buffer_size/item_size < 2)
    {
        return false;
    }
    
    q->front=0;
    q->rear=0;
    q->itemSize = item_size;
    q->itemCount = buffer_size/item_size;
    q->pool = (u8*)pool;
    
    return true;
}

bool Queue_Put(QueueType* q, void* pdata)
{
    u32 id = (q->rear + 1) % q->itemCount;
    if(q->front == id)
    {
        return false;
    }
    
    memcpy(q->pool+(q->rear*q->itemSize), pdata, q->itemSize);
    
    q->rear = id;
    return true;
}

bool Queue_Get(QueueType* q, void* pdata)
{
    if(q->front == q->rear)
    {
        return false;
    }
    
    if(pdata != NULL)
    {
        memcpy(pdata, q->pool+(q->front*q->itemSize), q->itemSize);
    }
    
    q->front = (q->front + 1) % q->itemCount;
    return true;
}

bool Queue_Query(QueueType* q, void* pdata)
{
    if(q->front == q->rear)
    {
        return false;
    }
    
    memcpy(pdata, q->pool+(q->front*q->itemSize), q->itemSize);
    
    return true;
}
