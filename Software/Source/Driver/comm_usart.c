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
    
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
    
    __HAL_UART_CLEAR_OREFLAG(huart);
    __HAL_UART_ENABLE(huart);
}

/**
* @brief  通讯串口发送数据
* @param  data    : 数据指针
* @param  len     : 数据长度
* @param  timeout : 超时，单位 ms，0 为一直等待
* @retval None
*/
u8 CommUsart_SendData(u8 *data, u16 len, u32 timeout)
{
    u32 i;
    
    //for(i=0; timeout == 0 || i < timeout; i++)
    {
        if(USER_UART_Transmit_DMA(COMM_UART, data, len, TxTcFlag) == HAL_OK)
        {
            SendIdle = false;
            return true;
        }
        //OSTimeDly(1);
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

