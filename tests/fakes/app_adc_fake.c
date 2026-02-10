#include "app_adc.h"

#include <stddef.h>

bool g_adc_ok = false;
float g_adc_voltage = 0.0f;

void app_adc_init(void)
{
}

bool app_adc_read_voltage(float *voltage_out)
{
    if (!g_adc_ok || voltage_out == NULL) {
        return false;
    }
    *voltage_out = g_adc_voltage;
    return true;
}
