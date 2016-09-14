/**
  ******************************************************************************
  * File Name          : mxconstants.h
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MXCONSTANT_H
#define __MXCONSTANT_H
  /* Includes ------------------------------------------------------------------*/

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/
#define CKTIM_APB1 72000000
#define CKTIM_APB2 72000000
#define TIM_FACTOR 2
#define PWM_PRSC 0
#define PWM_CNT_FREQ (CKTIM_APB2 / TIM_FACTOR / (PWM_PRSC + 1))
#define PWM_FREQ 20000
#define PWM_PERIOD (PWM_CNT_FREQ / PWM_FREQ - 1)
#define ENCODER_GAP 1024
#define BASIC_TIM_FREQ 2000
#define BASIC_TIM_PRSC 71
#define BASIC_TIM_CNT_FREQ (CKTIM_APB1 / (BASIC_TIM_PRSC + 1))
#define BASIC_TIM_PERIOD (BASIC_TIM_CNT_FREQ / BASIC_TIM_FREQ - 1)

#define ALERT_Pin GPIO_PIN_2
#define ALERT_GPIO_Port GPIOE
#define DROP_IN4_Pin GPIO_PIN_0
#define DROP_IN4_GPIO_Port GPIOC
#define AD_12V_Pin GPIO_PIN_1
#define AD_12V_GPIO_Port GPIOC
#define I4_SEN_Pin GPIO_PIN_2
#define I4_SEN_GPIO_Port GPIOC
#define I5_SEN_Pin GPIO_PIN_3
#define I5_SEN_GPIO_Port GPIOC
#define I1_SEN_Pin GPIO_PIN_2
#define I1_SEN_GPIO_Port GPIOA
#define I2_SEN_Pin GPIO_PIN_3
#define I2_SEN_GPIO_Port GPIOA
#define DROP_IN3_Pin GPIO_PIN_4
#define DROP_IN3_GPIO_Port GPIOA
#define DROP_IN_T_Pin GPIO_PIN_4
#define DROP_IN_T_GPIO_Port GPIOC
#define DROP_POWE_EN_Pin GPIO_PIN_5
#define DROP_POWE_EN_GPIO_Port GPIOC
#define DROP_IN1_Pin GPIO_PIN_0
#define DROP_IN1_GPIO_Port GPIOB
#define DROP_IN2_Pin GPIO_PIN_1
#define DROP_IN2_GPIO_Port GPIOB
#define FLT2B_1_Pin GPIO_PIN_7
#define FLT2B_1_GPIO_Port GPIOE
#define FLT3B_1_Pin GPIO_PIN_8
#define FLT3B_1_GPIO_Port GPIOE
#define MOTOR1_PWM1_Pin GPIO_PIN_9
#define MOTOR1_PWM1_GPIO_Port GPIOE
#define MOTOR1_PWM2_Pin GPIO_PIN_11
#define MOTOR1_PWM2_GPIO_Port GPIOE
#define MOTOR2_PWM1_Pin GPIO_PIN_13
#define MOTOR2_PWM1_GPIO_Port GPIOE
#define MOTOR2_PWM2_Pin GPIO_PIN_14
#define MOTOR2_PWM2_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_15
#define LED_GPIO_Port GPIOE
#define SPI_CS_Pin GPIO_PIN_12
#define SPI_CS_GPIO_Port GPIOB
#define FLT2B_2_Pin GPIO_PIN_14
#define FLT2B_2_GPIO_Port GPIOB
#define MPU_6500_INT_Pin GPIO_PIN_8
#define MPU_6500_INT_GPIO_Port GPIOD
#define STBYB_1_Pin GPIO_PIN_9
#define STBYB_1_GPIO_Port GPIOD
#define STBYB_2_Pin GPIO_PIN_10
#define STBYB_2_GPIO_Port GPIOD
#define FLT3B_2_Pin GPIO_PIN_10
#define FLT3B_2_GPIO_Port GPIOA
#define MOTOR1_ENCODER_A_Pin GPIO_PIN_15
#define MOTOR1_ENCODER_A_GPIO_Port GPIOA
#define US_TRIGGER_Pin GPIO_PIN_10
#define US_TRIGGER_GPIO_Port GPIOC
#define US_ECHO_Pin GPIO_PIN_11
#define US_ECHO_GPIO_Port GPIOC
#define SHDNB_12_Pin GPIO_PIN_0
#define SHDNB_12_GPIO_Port GPIOD
#define UR_PWR_BUTTON_Pin GPIO_PIN_1
#define UR_PWR_BUTTON_GPIO_Port GPIOD
#define RADAR_PWR_BUTTON_Pin GPIO_PIN_4
#define RADAR_PWR_BUTTON_GPIO_Port GPIOD
#define MOTOR1_ENCODER_B_Pin GPIO_PIN_3
#define MOTOR1_ENCODER_B_GPIO_Port GPIOB
#define MOTOR2_ENCODER_A_Pin GPIO_PIN_4
#define MOTOR2_ENCODER_A_GPIO_Port GPIOB
#define MOTOR2_ENCODER_B_Pin GPIO_PIN_5
#define MOTOR2_ENCODER_B_GPIO_Port GPIOB
#define S1_Pin GPIO_PIN_8
#define S1_GPIO_Port GPIOB
#define S2_Pin GPIO_PIN_9
#define S2_GPIO_Port GPIOB
#define S0_Pin GPIO_PIN_0
#define S0_GPIO_Port GPIOE
#define OE1_Pin GPIO_PIN_1
#define OE1_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MXCONSTANT_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
