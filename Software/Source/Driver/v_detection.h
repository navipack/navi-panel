#ifndef __V_DETECTION_H__
#define __V_DETECTION_H__

#include "adc_user.h"
#include "stm32_lib.h"
#include "adc.h"

#define V_Alarm_Pin GPIO_PIN_5
#define V_Alarm_alarm_GPIO_Port GPIOD

void supply_voltage_detection(void);

#endif
