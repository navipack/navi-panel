/******************** (C) COPYRIGHT 2008 STMicroelectronics ********************
* File Name          : MC_Control_Param.h
* Author             : IMS Systems Lab 
* Date First Issued  : 21/11/07
* Description        : This file gathers parameters related to:
*                      power devices, speed regulation frequency, PID controllers
*                      setpoints and constants, start-up ramp, lowest values for
*                      speed reading validation.
********************************************************************************/
/******************** SPEED PID-CONTROLLER INIT VALUES************************/

/* default values for Speed control loop */
#define PID_SPEED_REFERENCE_RPM   (s16)200	//速度
#define PID_SPEED_KP_DEFAULT      (s16)10
#define PID_SPEED_KI_DEFAULT      (s16)5
#define PID_SPEED_KD_DEFAULT      (s16)0000

/* Speed PID parameter dividers          */
#define SP_KPDIV ((u16)(16))
#define SP_KIDIV ((u16)(256))
#define SP_KDDIV ((u16)(16))

/************** QUADRATURE CURRENTS PID-CONTROLLERS INIT VALUES **************/

// With MB459 phase current (A)= (PID_X_REFERENCE * 0.29)/(32767 * Rshunt)

/* default values for Torque control loop */   //额定电流为：5A，5 * 1241 = 6205
#define PID_TORQUE_REFERENCE   (s16)1000   //(N.b: that's the reference init  
                                       //value in both torque and speed control)
//#define PID_TORQUE_KP_DEFAULT  (s16)3314      
//#define PID_TORQUE_KI_DEFAULT  (s16)274
//#define PID_TORQUE_KD_DEFAULT  (s16)100
#define PID_TORQUE_KP_DEFAULT  (s16)1100       
#define PID_TORQUE_KI_DEFAULT  (s16)300
#define PID_TORQUE_KD_DEFAULT  (s16)20

/* default values for Flux control loop */
#define PID_FLUX_REFERENCE   (s16)0
//#define PID_FLUX_KP_DEFAULT  (s16)3314
//#define PID_FLUX_KI_DEFAULT  (s16)274
//#define PID_FLUX_KD_DEFAULT  (s16)100
#define PID_FLUX_KP_DEFAULT  (s16)1100  
#define PID_FLUX_KI_DEFAULT  (s16)300
#define PID_FLUX_KD_DEFAULT  (s16)20

// Toruqe/Flux PID  parameter dividers
#define TF_KPDIV ((u16)(1024))
#define TF_KIDIV ((u16)(16384))
#define TF_KDDIV ((u16)(8192))

/* Define here below the period of the square waved reference torque generated
 when FLUX_TORQUE_PIDs_TUNING is uncommented in STM32F10x_MCconf.h          */
#define SQUARE_WAVE_PERIOD   (u16)2000//in msec 
