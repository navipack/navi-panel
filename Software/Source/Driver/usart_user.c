/**
******************************************************************************
* @file    usart_user.c
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   串口驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#include "usart_user.h"

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
