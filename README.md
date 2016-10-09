# NaviPanel

开发用底盘的驱动源码及原理图等相关资料

## Copyright &copy; 2016 Inmotion Corporation
该仓库所有文件为**深圳乐行天下科技有限公司**所有，未经许可不得用于商业用途。

# 目录树简介
    NaviPanel
    │  LICENSE
    │  README.md
    │
    ├─Document
    ├─Hardware
    └─Software
        ├─HAL  CubeMX 生成 STM32 库文件
        │  │  navi-panel.ioc
        │  │
        │  └─Src
        │          main.c  主文件
        │
        └─Source
           ├─App
           │      comm.c             通讯协议传输层解包封包等相关函数
           │      comm.h             通讯协议解包封包等相关声明
           │      global.c           一些全局变量
           │      global_defines.h   一些全局定义
           │      motion_control.c   运动控制模块，包含了一些加减速和初始化之类的算法
           │      motion_control.h   运动控制模块声明
           │      sensor_update.c    IMU 传感器解算
           │      sensor_update.h    IMU 传感器数据声明
           │      system_supervise.c 系统监控
           │      system_supervise.h 系统监控声明
           │
           ├─Arithmetic
           │      AVG_filter.c       均值滤波器   
           │      AVG_filter.h       均值滤波器声明   
           │      FIR_Filter.c       FIR滤波器   
           │      FIR_Filter.h       FIR滤波器声明   
           │      math_lib.c         数学函数库   
           │      math_lib.h         数学函数库声明   
           │      motor_params_typ.h 电机控制声明
           │      PID_regulators.c   PID 计算   
           │      PID_regulators.h   PID 计算声明   
           │      queue.c            循环队列   
           │      queue.h            循环队列声明   
           │      speed_loop.c       速度闭环相关处理   
           │      speed_loop.h       速度闭环相关声明   
           │
           ├─Driver
           │  │  adc_user.c          ADC 驱动
           │  │  adc_user.h          ADC 声明
           │  │  comm_usart.c        通讯用串口驱动   
           │  │  comm_usart.h        通讯用串口声明   
           │  │  contact_detection.c 碰撞传感器驱动
           │  │  contact_detection.h 碰撞传感器声明   
           │  │  encoder.c           增量式编码器程序   
           │  │  encoder.h           增量式编码器声明  
           │  │  gpio_user.c         GPIO 相关驱动   
           │  │  gpio_user.h         GPIO 相关声明   
           │  │  i2c_user.c          I2C 驱动   
           │  │  i2c_user.h          I2C 声明   
           │  │  infrared_drop.c     地测跌落传感器驱动   
           │  │  infrared_drop.h     地测跌落传感器声明   
           │  │  inv_mpu_driver.c    MPU6500 驱动
           │  │  inv_mpu_driver.h    MPU6500 声明  
           │  │  motor.c             有刷直流电机 PWM 驱动相关实现   
           │  │  motor.h             有刷直流电机 PWM 驱动相关声明   
           │  │  stm32_lib.h         变量类型声明
           │  │  tim_user.c          定时器相关驱动   
           │  │  tim_user.h          定时器相关驱动声明  
           │  │  TMPx75.c            温度检测芯片驱动   
           │  │  TMPx75.h            温度检测芯片驱动声明   
           │  │  ultrasonic.c        超声波模块驱动   
           │  │  ultrasonic.h        超声波模块驱动声明   
           │  │  usart_user.c        串口驱动   
           │  │  usart_user.h        串口驱动声明   
           │  │  v_detection.c       电压检测 （电源12V电压检测，当低于11.9V时会使指示灯闪烁）
           │  │  v_detection.h       电压检测声明   
           │  │
           │  └─eMPL  MPU 提供 SDK 源码
           │
           └─navipack_sdk  详见 navipack/mcu-sdk

# 相关链接
[navipack/mcu-sdk](https://github.com/navipack/mcu-sdk)