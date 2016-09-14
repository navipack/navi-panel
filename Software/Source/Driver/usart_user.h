/**
******************************************************************************
* @file    usart_user.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   串口驱动
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

#ifndef __USART_USER_H__
#define __USART_USER_H__

#include "stm32_lib.h"
#include "usart.h"

HAL_StatusTypeDef USER_UART_Transmit_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, u32 tx_tc_flag);

#endif
