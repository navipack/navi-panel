
#include "v_detection.h"

double supply_v = 0;

void supply_voltage_detection(void)
{
    supply_v=HAL_ADC_GetValue(&hadc2) * 4.985 / ADC_GetRef() ;   
    if(supply_v<11.9)
    {
        HAL_GPIO_TogglePin(V_Alarm_alarm_GPIO_Port, V_Alarm_Pin);
    }
    else
    {
        HAL_GPIO_WritePin(V_Alarm_alarm_GPIO_Port, V_Alarm_Pin, GPIO_PIN_RESET);
    }
}
