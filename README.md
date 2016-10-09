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
           │      comm.c
           │      comm.h
           │      global.c
           │      global_defines.h
           │      motion_control.c
           │      motion_control.h
           │      sensor_update.c
           │      sensor_update.h
           │      system_supervise.c
           │      system_supervise.h
           │
           ├─Arithmetic
           │      AVG_filter.c
           │      AVG_filter.h
           │      FIR_Filter.c
           │      FIR_Filter.h
           │      math_lib.c
           │      math_lib.h
           │      motor_params_typ.h
           │      PID_regulators.c
           │      PID_regulators.h
           │      queue.c
           │      queue.h
           │      speed_loop.c
           │      speed_loop.h
           │
           ├─Driver
           │  │  adc_user.c
           │  │  adc_user.h
           │  │  comm_usart.c
           │  │  comm_usart.h
           │  │  contact_detection.c
           │  │  contact_detection.h
           │  │  encoder.c
           │  │  encoder.h
           │  │  gpio_user.c
           │  │  gpio_user.h
           │  │  i2c_user.c
           │  │  i2c_user.h
           │  │  infrared_drop.c
           │  │  infrared_drop.h
           │  │  inv_mpu_driver.c
           │  │  inv_mpu_driver.h
           │  │  motor.c
           │  │  motor.h
           │  │  stm32_lib.h
           │  │  tim_user.c
           │  │  tim_user.h
           │  │  TMPx75.c
           │  │  TMPx75.h
           │  │  ultrasonic.c
           │  │  ultrasonic.h
           │  │  usart_user.c
           │  │  usart_user.h
           │  │  v_detection.c
           │  │  v_detection.h
           │  │
           │  └─eMPL  MPU 提供 SDK 源码
           │
           └─navipack_sdk  详见 navipack/mcu-sdk

# 相关链接
[navipack/mcu-sdk](https://github.com/navipack/mcu-sdk)