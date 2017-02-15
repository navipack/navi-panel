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
* @param  hcomm : 句柄
* @param  huart : 串口句柄
* @retval None
*/
void CommUsart_Init(CommUsartType *hcomm, UART_HandleTypeDef *huart)
{
    hcomm->huart = huart;
    hcomm->tx_tc_flag = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmatx);
    hcomm->offset = 0;
    
    __HAL_UART_CLEAR_OREFLAG(huart);
    //__HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    //__HAL_UART_ENABLE(huart);
    HAL_DMA_Start(huart->hdmarx, (u32)&huart->Instance->DR, (u32)hcomm->dma_rx_buffer, hcomm->buffer_size);
    huart->Instance->CR3 |= USART_CR3_DMAR;
}

/**
* @brief  通讯串口发送数据
* @param  hcomm : 句柄
* @param  data  : 数据指针
* @param  len   : 数据长度
* @retval 1 发送成功
*/
u8 CommUsart_SendData(CommUsartType *hcomm, u8 *data, u16 len)
{
    return USER_UART_Transmit_DMA(hcomm->huart, data, len, hcomm->tx_tc_flag) == HAL_OK;
}

/**
* @brief  通讯串口是否允许发送
* @param  hcomm : 句柄
* @retval true 可以发送
*/
bool CommUsart_CanSendData(CommUsartType *hcomm)
{
    if(hcomm->huart->hdmatx->State != HAL_DMA_STATE_BUSY 
        || __HAL_DMA_GET_FLAG(hcomm->huart->hdmatx, hcomm->tx_tc_flag))
    {
        return true;
    }
    return false;
}

/**
* @brief  通讯串口接收数据
* @param  hcomm : 句柄
* @param  pbuf  : 返回接收到的数据指针
* @param  plen  : 返回接收到的数据长度
* @retval 是否有数据
*/
bool CommUsart_RecvData(CommUsartType *hcomm, u8 **pbuf, u32* plen)
{
    u16 data_cnt = hcomm->buffer_size - __HAL_DMA_GET_COUNTER(hcomm->huart->hdmarx);
    *pbuf = &hcomm->dma_rx_buffer[hcomm->offset];
    if(data_cnt < hcomm->offset)
    {
        *plen = hcomm->buffer_size - hcomm->offset;
        hcomm->offset = 0;
    }
    else
    {
        *plen = data_cnt - hcomm->offset;
        hcomm->offset = data_cnt;
    }
    
    if(*plen > 0) return true;
    
    return false;
}

/**
* @brief  使能串口接收中断
* @param  hcomm : 句柄
* @param  en    : true 使能
* @retval None
*/
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
