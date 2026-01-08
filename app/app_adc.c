#include "app_adc.h"

#include "app_main.h"
#include "main.h"
#include "stm32h7xx_hal.h"

/* ADC reference voltage (V) for STM32H7 */
#define ADC_VREF 3.3f

/* ADC oversampling ratio (configured in CubeMX) */
#define ADC_OVERSAMPLING_RATIO 16U

/* ADC resolution (16-bit) with oversampling factor */
/* Max value is 65535 * 16 = 1048560 when oversampling without right shift */
#define ADC_MAX_VALUE (65535.0f * ADC_OVERSAMPLING_RATIO)

/* ADC conversion timeout (ms) */
#define ADC_TIMEOUT_MS 100U

void app_adc_init(void) {
  /* Perform ADC calibration */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY,
                                   ADC_SINGLE_ENDED) != HAL_OK) {
    Error_Handler();
  }
}

bool app_adc_read_voltage(float *voltage_out) {
  if (voltage_out == NULL) {
    return false;
  }

  /* Start ADC conversion */
  if (HAL_ADC_Start(&hadc1) != HAL_OK) {
    return false;
  }

  /* Wait for conversion to complete */
  if (HAL_ADC_PollForConversion(&hadc1, ADC_TIMEOUT_MS) != HAL_OK) {
    HAL_ADC_Stop(&hadc1);
    return false;
  }

  /* Read ADC value */
  uint32_t adc_value = HAL_ADC_GetValue(&hadc1);

  /* Stop ADC */
  HAL_ADC_Stop(&hadc1);

  /* Convert to voltage (0-3.3V range) */
  float raw_voltage = ((float)adc_value / ADC_MAX_VALUE) * ADC_VREF;

  /* Apply voltage divider multiplier from parameters */
  float multiplier = g_robot_params.adc_voltage_multiplier;
  if (multiplier <= 0.0f) {
    multiplier = 1.0f; /* Default to 1.0 if not configured */
  }

  *voltage_out = raw_voltage * multiplier;
  return true;
}
