#pragma once

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

adc_oneshot_unit_handle_t get_adc1_handle(void);

adc_cali_handle_t get_cali_handle_adc1(void);

adc_oneshot_unit_handle_t get_adc2_handle(void);

adc_cali_handle_t get_cali_handle_adc2(void);

int get_battery_voltage_mV(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_handle);

void update_line_sensor_table(int *line_sensor_table, adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_handle, adc_oneshot_unit_handle_t adc2_handle, adc_cali_handle_t adc2_cali_handle);

int get_soc_from_voltage(int voltage_mV);