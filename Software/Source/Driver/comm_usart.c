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

/**
* @brief  通讯用串口初始化
* @param  huart : 串口句柄
* @retval None
*/
void CommUsart_Init(CommUsartType *hcomm, UART_HandleTypeDef *huart)
{
    hcomm->huart = huart;
    hcomm->tx_tc_flag = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmatx);
    hcomm->tx_idle = false;
    
    __HAL_UART_CLEAR_OREFLAG(huart);
    //__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    //__HAL_UART_ENABLE(huart);
    HAL_DMA_Start(huart->hdmarx, (u32)&huart->Instance->DR, (u32)hcomm->dma_rx_buffer, hcomm->buffer_size);
    huart->Instance->CR3 |= USART_CR3_DMAR;
}

/**
* @brief  通讯串口发送数据
* @param  data    : 数据指针
* @param  len     : 数据长度
* @param  timeout : 超时，单位 ms，0 为一直等待
* @retval None
*/
u8 CommUsart_SendData(CommUsartType *hcomm, u8 *data, u16 len)
{
    if(USER_UART_Transmit_DMA(hcomm->huart, data, len, hcomm->tx_tc_flag) == HAL_OK)
    {
        hcomm->tx_idle = false;
        return true;
    }

    return false;
}

bool CommUsart_CanSendData(CommUsartType *hcomm)
{
    if(__HAL_DMA_GET_FLAG(hcomm->huart->hdmatx, hcomm->tx_tc_flag))
    {
        hcomm->tx_idle = true;
    }
    return hcomm->tx_idle;
}

/**
* @brief  通讯串口接收数据
* @param  pbuf : 返回接收到的数据指针
* @param  plen : 返回接收到的数据长度
* @retval 是否有数据
*/
bool CommUsart_RecvData(CommUsartType *hcomm, u8 **pbuf, u32* plen)
{
    static u32 offset = 0;
    
    u16 data_cnt = hcomm->buffer_size - __HAL_DMA_GET_COUNTER(hcomm->huart->hdmarx);
    *pbuf = &hcomm->dma_rx_buffer[offset];
    if(data_cnt < offset)
    {
        *plen = hcomm->buffer_size - offset;
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

void CommUsart_EnableIT(CommUsartType *hcomm, bool en)
{
    if(en)
    {
        __HAL_UART_ENABLE_IT(hcomm->huart, UART_IT_RXNE);
    }
    else
    {
        __HAL_UART_DISABLE_IT(hcomm->huart, UART_IT_RXNE);
    }
}

/**
* @brief  串口 DMA 发送接口
* @param  huart      : 串口句柄
* @param  pData      : 数据指针
* @param  Size       : 数据长度
* @param  tx_tc_flag : 发送完成 flag 偏移
* @retval None
*/
HAL_StatusTypeDef USER_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, u32 tx_tc_flag)
{
    if(__HAL_DMA_GET_FLAG(huart->hdmatx, tx_tc_flag))
    {
        if(huart->State == HAL_UART_STATE_BUSY_TX_RX)
        {
            huart->State = HAL_UART_STATE_BUSY_TX;
        }
        else
        {
            huart->State = HAL_UART_STATE_READY;
        }
        
        __HAL_UNLOCK(huart->hdmatx);
        __HAL_DMA_CLEAR_FLAG(huart->hdmatx, tx_tc_flag);
    }
    return HAL_UART_Transmit_DMA(huart, pData, Size);
}
