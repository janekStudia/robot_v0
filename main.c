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
#define MAX_PWM_OUTPUT_DIFF_P_CONTROLER 105
#define MAX_RS_DIFFERENCE_P_CONTROLER 2200

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

void u_print_rs_value_table(const int *rs_value_table)
{
  printf("[DEBUG] rs_value_table contents(1-8) in mV = [ ");

  for (int i = 0; i < RS_DATA_TABLE_SIZE; i++)
  {
    printf("%d:%d ", i + 1, rs_value_table[i]);
  }
  printf("]\n");
}

void loop_simple_line_pid(void)
{ // RS = Reflective Sensor
  int rs_value_table[RS_DATA_TABLE_SIZE - 1];
  rs_enable_leds();
  int pwm_forward = 200;
  set_motors_duty(pwm_forward, pwm_forward);
  while (true)
  {
    update_line_sensor_table(rs_value_table);
    u_print_rs_value_table(rs_value_table);

    int p_error = rs_value_table[5] - rs_value_table[6];
    int p_result = ((MAX_PWM_OUTPUT_DIFF_P_CONTROLER * p_error) / 2) / MAX_RS_DIFFERENCE_P_CONTROLER;
    printf("[DEBUG] p_error=%d, p_result=%d\n", p_error, p_result);
    set_motors_duty(pwm_forward - p_result, pwm_forward + p_result);

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

  // loop_print_rs_output();
  loop_simple_line_pid();
  // TODO add a P controler function to follow a simple continous line
  // patrzymy czy srodkowe sa wewnatrz lini (powyzej konkretnego poziomu pochlaniania swiatla) ponizej tego poziomu roznica poziomu realnego od minimalnego to wejscie kontrolera P
  // to rozwiazanie nie rozwiazuje problemu zwiazengo z wyjsciem bardzo poza linie (bo wtedy ani jeden ani drugi nie są w zasiegu i nie wiadomo w ktorą stronę nadrobić)
  // najlepiej miec tablice z elementami 3 stanowymi - na linii, poza linią, na skraju.

  // read_battery_level_uart_debug(adc1_handle, adc1_cali_handle);
}