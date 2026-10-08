#pragma once

void init_both_adc_units(void);

int get_battery_voltage_mV(void);

int get_soc_from_voltage(int voltage_mV);

int get_soc(void);

void update_line_sensor_table(int *line_sensor_table, int *line_mask_ptr, int *line_mask_popcount_ptr);