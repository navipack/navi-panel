#ifndef __V_DETECTION_H__
#define __V_DETECTION_H__

#include "adc_user.h"
#include "stm32_lib.h"
#include "adc.h"

#define V_Alarm_Pin GPIO_PIN_5          /** 电压警报Pin */
#define V_Alarm_alarm_GPIO_Port GPIOD   /** 电压警报Port */
#define V_Alarm 11.9                    /** 报警电压 */
void supply_voltage_detection(void);

#endif
