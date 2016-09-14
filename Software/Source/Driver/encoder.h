/**
******************************************************************************
* @file    encoder.h
* @author  Inmotion NaviPanel team
* @date    2016/09/14
* @brief   增量式编码器程序声明
* @attention Copyright (C) 2016 Inmotion Corporation
******************************************************************************
*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _RT_ENCODER_H
#define _RT_ENCODER_H

#include "stm32_lib.h"

/* Type Defines ------------------------------------------------------------------*/
#define ENCODER_SAMPLING_FREQ		500

/* Includes ------------------------------------------------------------------*/
/*****************************  Encoder settings ******************************/

/* Exported functions ------------------------------------------------------- */
void Encoder_IRQHandler(u8 index);

void EncInit(u8);
void EncClearSpeedBuffer(u8);
void EncResetEncoder(u8, u16);
void EncGetMachanicalSpeed(u8, s32* feedback_spd, s32* observe_spd);
bool EncErrorOnFeedback(u8);

#endif  /*__STM32F10x_ENCODER_H*/
/******************* (C) COPYRIGHT 2008 STMicroelectronics *****END OF FILE****/
