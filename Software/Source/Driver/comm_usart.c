/**
******************************************************************************
* @file    comm_usart.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   通讯用串口驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "comm_usart.h"
#include "usart_user.h"

static UART_HandleTypeDef *COMM_UART;
#define UART_BUFF_SIZE 0x100

static u32 TxTcFlag;
static u8 UartBuffer[UART_BUFF_SIZE];
static bool SendIdle = true;

/**
* @brief  通讯用串口初始化
* @param  huart : 串口句柄
* @retval None
*/
void CommUsart_Init(UART_HandleTypeDef *huart)
{ 
    COMM_UART = huart;
    
    TxTcFlag = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmatx);
    
    __HAL_UART_CLEAR_OREFLAG(huart);
    //__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    //__HAL_UART_ENABLE(huart);
    HAL_DMA_Start(huart->hdmarx, (u32)&huart->Instance->DR, (u32)UartBuffer, UART_BUFF_SIZE);
    huart->Instance->CR3 |= USART_CR3_DMAR;
}

/**
* @brief  通讯串口发送数据
* @param  data    : 数据指针
* @param  len     : 数据长度
* @param  timeout : 超时，单位 ms，0 为一直等待
* @retval None
*/
u8 CommUsart_SendData(u8 *data, u16 len)
{
    if(USER_UART_Transmit_DMA(COMM_UART, data, len, TxTcFlag) == HAL_OK)
    {
        SendIdle = false;
        return true;
    }

    return false;
}

bool CommUsart_CanSendData(void)
{
    if(__HAL_DMA_GET_FLAG(COMM_UART->hdmatx, TxTcFlag))
    {
        SendIdle = true;
    }
    return SendIdle;
}

/**
* @brief  通讯串口接收数据
* @param  pbuf : 返回接收到的数据指针
* @param  plen : 返回接收到的数据长度
* @retval 是否有数据
*/
bool CommUsart_RecvData(u8 **pbuf, u32* plen)
{
    static u32 offset = 0;
    
    u16 data_cnt = UART_BUFF_SIZE - __HAL_DMA_GET_COUNTER(COMM_UART->hdmarx);
    *pbuf = &UartBuffer[offset];
    if(data_cnt < offset)
    {
        *plen = UART_BUFF_SIZE - offset;
        offset = 0;
    }
    else
    {
        *plen = data_cnt - offset;
        offset = data_cnt;
    }
    
    if(*plen > 0) return true;
    
    return false;
}

void CommUsart_EnableIT(bool en)
{
    if(en)
    {
        __HAL_UART_ENABLE_IT(COMM_UART, UART_IT_RXNE);
    }
    else
    {
        __HAL_UART_DISABLE_IT(COMM_UART, UART_IT_RXNE);
    }
}

