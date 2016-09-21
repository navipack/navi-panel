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

void CommUsart_Init(UART_HandleTypeDef *huart);
u8 CommUsart_SendData(u8 *data, u16 len);
bool CommUsart_CanSendData(void);
void CommUsart_EnableIT(bool en);

#endif
