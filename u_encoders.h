#pragma once

#include "driver/pulse_cnt.h"

void init_single_pcnt_unit(int gpio_pin, pcnt_unit_handle_t *unit_handle);
int get_rotations(pcnt_unit_handle_t encoder_pcnt_handle);