#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "driver/gpio.h"

// #include "esp_err.h"

// wlasne
#include "u_motors.h"
#include "u_encoders.h"
#include "u_analog_sensors.h"
#include "u_indication_led.h"

#define RS_DATA_TABLE_SIZE 8
#define MAX_PWM_OUTPUT_DIFF_P_CONTROLER 200
#define MAX_RS_ERROR_VAL_P_CONTROLER (3 * 2500)

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

void u_print_rs_value_table(const int *rs_value_table)
{
  printf("[DEBUG] rs_value_table contents(1-8) in mV = [ ");

  for (int i = 0; i < RS_DATA_TABLE_SIZE; i++)
  {
    printf("%d ", rs_value_table[i]);
  }
  printf("]\n");
}

void turn_left(void)
{
  set_motors_duty(150, -150);
  vTaskDelay(pdMS_TO_TICKS(900));
  set_motors_duty(200, 200);
}

void turn_right(void)
{
  set_motors_duty(-150, 150);
  vTaskDelay(pdMS_TO_TICKS(900));
  set_motors_duty(200, 200);
}

void skip_junction(void)
{
  vTaskDelay(pdMS_TO_TICKS(600));
}

void loop_simple_line_p(void)
{ // RS = Reflective Sensor
  int rs_value_table[RS_DATA_TABLE_SIZE];
  int pwm_forward = 200;

  static const int multipliers[RS_DATA_TABLE_SIZE] = {-3, -3, -2, -1, 1, 2, 3, 3};

  set_motors_duty(pwm_forward, pwm_forward);
  while (true)
  {
    update_line_sensor_table(rs_value_table);
    // u_print_rs_value_table(rs_value_table);

    int p_error = 0;
    int line_contacts_counter = 0;

    static int on_junction_counter = 0;
    static int junction_number = 0;

    for (int i = 0; i < RS_DATA_TABLE_SIZE; i++)
    {
      p_error += rs_value_table[i] * multipliers[i];
      if (rs_value_table[i] > 2000)
      {
        line_contacts_counter++;
      };
    }
    printf("[DEBUG] line_contacts_counter=%d", line_contacts_counter);
    if (line_contacts_counter > 3) // on junction
    {
      on_junction_counter++;
    }
    else // off junction
    {
      int p_result = ((MAX_PWM_OUTPUT_DIFF_P_CONTROLER * p_error) / 2) / MAX_RS_ERROR_VAL_P_CONTROLER;
      set_motors_duty(pwm_forward + p_result, pwm_forward - p_result);

      if (on_junction_counter > 0)
      {
        on_junction_counter--; // decreases until 0 when out of junction to take care of residual counter values for future junctions
      }
    }

    if (on_junction_counter > 20)
    {
      junction_number++;
      on_junction_counter = 0;
      if (junction_number == 1)
      {
        turn_right();
      }
      else if (junction_number % 7 == 4)
      {
        turn_right();
      }
      else if (junction_number % 7 == 1)
      {
        skip_junction();
      }
      else
      {
        turn_left();
      }
    }
  }
}

extern void app_main(void)
{
  // --- INIT ---
  init_motor_control();
  init_both_adc_units();
  init_encoders_pcnt_units();
  init_indication_led();
  // --- END INIT ---

  vTaskDelay(pdMS_TO_TICKS(3000));
  loop_simple_line_p();

  // read_battery_level_uart_debug(adc1_handle, adc1_cali_handle);
}