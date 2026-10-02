#include <stdio.h>

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"

// #include "driver/gpio.h"

// #include "esp_err.h"

// wlasne
#include "u_motors.h"
#include "u_encoders.h"
#include "u_analog_sensors.h"

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

  // --- END INIT ---

  // TODO dodac tablice z wartosciami napiec na sensorze linii i pobrac do niej wartosci

  // read_battery_level_uart_debug(adc1_handle, adc1_cali_handle);
  while (true)
  {
    vTaskDelay(pdMS_TO_TICKS(1000));
    // printf("[DEBUG]: obroty B = %d\n", get_encB_count(g_encoderB_pcnt_handle));
    // printf("[DEBUG]: predkosc chwilowa = %d\n", g_encoder_B_ticks_per_s);
  }
}