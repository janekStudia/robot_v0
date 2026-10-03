#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "driver/gpio.h"

// #include "esp_err.h"

// wlasne
#include "u_motors.h"
#include "u_encoders.h"
#include "u_analog_sensors.h"

#define RS_DATA_TABLE_SIZE 8

void read_battery_level_uart_debug(void)
{
  int voltage_mV = get_battery_voltage_mV();
  printf("[BATTERY_LEVEL]: voltage[mV] = %d ", voltage_mV);
  printf("soc[prc]: = %d\n", get_soc_from_voltage(voltage_mV));
}

void pattern_straight_line(void)
{
  static int pwmA = 200;
  static int pwmB = 200;
  set_motors_duty(pwmA, pwmB);
}

void loop_pattern_straigh_line(void)
{
  int i = 20;
  while (true)
  {
    pattern_straight_line();
    printf("[DEBUG]: motorA encoder ticks = %d\n", get_encA_count());
    printf("[DEBUG]: motorB encoder ticks = %d\n\n", get_encB_count());
    vTaskDelay(pdMS_TO_TICKS(1000));
    if (i > 0)
    {
      i--;
    }
    else
    {
      sleep_motors();
    }

    // printf("[DEBUG]: obroty B = %d\n", get_encB_count(g_encoderB_pcnt_handle));
    // printf("[DEBUG]: predkosc chwilowa = %d\n", g_encoder_B_ticks_per_s);
  }
}

void loop_print_rs_output(void)
{ // RS = Reflective Sensor
  int rs_value_table[RS_DATA_TABLE_SIZE - 1];
  rs_enable_leds();
  while (true)
  {
    update_line_sensor_table(rs_value_table);
    printf("[DEBUG] rs_value_table contents(1-8) in mV = [ ");

    for (int i = 0; i < RS_DATA_TABLE_SIZE; i++)
    {
      printf("%d:%d ", i + 1, rs_value_table[i]);
    }
    printf("]\n");

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

extern void app_main(void)
{
  // --- INIT ---
  init_motor_control();
  init_both_adc_units();
  init_encoders_pcnt_units();
  // --- END INIT ---

  // TODO dodac tablice z wartosciami napiec na sensorze linii i pobrac do niej wartosci
  loop_print_rs_output();
  // read_battery_level_uart_debug(adc1_handle, adc1_cali_handle);
}