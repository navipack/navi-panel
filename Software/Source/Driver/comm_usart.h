/**
******************************************************************************
* @file    comm_usart.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   通讯用串口驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __COMM_USART_H__
#define __COMM_USART_H__

#include "stm32_lib.h"
#include "usart.h"

typedef struct{
    UART_HandleTypeDef *huart;
    u32 tx_tc_flag;
    u8* dma_rx_buffer;
    u16 buffer_size;
    u32 offset;
}CommUsartType;

void CommUsart_Init(CommUsartType *hcomm, UART_HandleTypeDef *huart);
u8 CommUsart_SendData(CommUsartType *hcomm, u8 *data, u16 len);
bool CommUsart_CanSendData(CommUsartType *hcomm);
bool CommUsart_RecvData(CommUsartType *hcomm, u8 **pbuf, u32* plen);
void CommUsart_EnableIT(CommUsartType *hcomm, bool en);

HAL_StatusTypeDef USER_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, u32 tx_tc_flag);

#endif
