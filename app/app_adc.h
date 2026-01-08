#ifndef APP_ADC_H
#define APP_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Initialize ADC module
 *
 * Performs ADC calibration and prepares for conversions.
 */
void app_adc_init(void);

/**
 * @brief Read voltage from ADC1 CH4 (PC4)
 *
 * Performs a single ADC conversion, applies the voltage divider
 * multiplier from robot parameters, and returns the scaled voltage.
 *
 * @param voltage_out Pointer to store the voltage reading (V)
 * @return true if reading successful, false otherwise
 */
bool app_adc_read_voltage(float *voltage_out);

#ifdef __cplusplus
}
#endif

#endif /* APP_ADC_H */
