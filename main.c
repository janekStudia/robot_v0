#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "driver/gpio.h"

// #include "esp_err.h"

// wlasne
#include "u_motors.h"
#include "u_encoders.h"
#include "u_analog_sensors.h"
// #include "u_indication_led.h"
#include "u_control.h"

void read_battery_level_uart_debug(void)
{
  int voltage_mV = get_battery_voltage_mV();
  printf("[BATTERY_LEVEL]: voltage[mV] = %d ", voltage_mV);
  printf("soc[prc]: = %d\n", get_soc_from_voltage(voltage_mV));
}

extern void app_main(void)
{
  // --- INIT ---
  init_motor_control();
  init_both_adc_units();
  init_encoders_pcnt_units();
  init_my_timer();
  // init_indication_led();
  // --- END INIT ---

  vTaskDelay(pdMS_TO_TICKS(3000));
  set_motors_duty(200, 200);
  while (true)
  {
    update_rs_values();
    junction_detector();
    p_controler();
    update_pwm();
  }

  // read_battery_level_uart_debug(adc1_handle, adc1_cali_handle);
}