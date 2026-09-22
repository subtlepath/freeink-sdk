#pragma once
// FreeInk simulator — legacy ADC calibration shim. The model already works in
// calibrated millivolts, so characterization is a no-op and raw_to_voltage is
// the identity scaling the daemon uses for that pin.
#include <cstdint>

#include "esp_err.h"

typedef enum { ADC_UNIT_1 = 0, ADC_UNIT_2 = 1 } adc_unit_t;
typedef enum { ADC_ATTEN_DB_0 = 0, ADC_ATTEN_DB_2_5, ADC_ATTEN_DB_6, ADC_ATTEN_DB_11, ADC_ATTEN_DB_12 = 3 } adc_atten_t;
typedef enum { ADC_WIDTH_BIT_12 = 3, ADC_WIDTH_BIT_DEFAULT = 3 } adc_bits_width_t;
typedef enum { ESP_ADC_CAL_VAL_EFUSE_VREF = 0, ESP_ADC_CAL_VAL_EFUSE_TP, ESP_ADC_CAL_VAL_DEFAULT_VREF } esp_adc_cal_value_t;

typedef struct {
  adc_unit_t adc_num;
  adc_atten_t atten;
  adc_bits_width_t bit_width;
  uint32_t coeff_a;
  uint32_t coeff_b;
  uint32_t vref;
} esp_adc_cal_characteristics_t;

#ifdef __cplusplus
extern "C" {
#endif
esp_adc_cal_value_t esp_adc_cal_characterize(adc_unit_t unit, adc_atten_t atten, adc_bits_width_t width,
                                             uint32_t default_vref, esp_adc_cal_characteristics_t* chars);
uint32_t esp_adc_cal_raw_to_voltage(uint32_t raw, const esp_adc_cal_characteristics_t* chars);
#ifdef __cplusplus
}
#endif
