#include <stdio.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_timer.h"
#include "esp_attr.h"

#include "driver/gpio.h"
// #include "driver/ledc.h"
// #include "driver/uart.h"
#include "driver/pulse_cnt.h"

#include "esp_err.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include <stdatomic.h>

// wlasne
#include "u_motors.h"
#include "u_encoders.h"
#include "u_analog_sensors.h"

#define ENCODER_A_PIN GPIO_NUM_16
#define ENCODER_B_PIN GPIO_NUM_4

// magic numbers

#define TIMER_INTERVAL_US (1000 * 1000) // 100ms in us
#define TIMER_INTERVALS_PERS 1

static atomic_int g_encoder_B_ticks_per_s = 0;
pcnt_unit_handle_t g_encoderA_pcnt_handle = NULL;
pcnt_unit_handle_t g_encoderB_pcnt_handle = NULL;

static void IRAM_ATTR
timer_callback(void *arg)
{
  static int count;
  static int prev_count = 0;
  pcnt_unit_get_count(g_encoderB_pcnt_handle, &count);

  atomic_store(&g_encoder_B_ticks_per_s, (count - prev_count) * TIMER_INTERVALS_PERS);
  prev_count = count;

  printf("[DEBUG]: predkosc chwilowa = %d\n", g_encoder_B_ticks_per_s);
}

void init_timer()
{
  const esp_timer_create_args_t timer_args = {
      .callback = &timer_callback,
      .name = "speed_timer"};
  esp_timer_handle_t speed_timer;
  ESP_ERROR_CHECK(esp_timer_create(&timer_args, &speed_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(speed_timer, TIMER_INTERVAL_US));
}

void read_battery_level_uart_debug(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_handle)
{
  int voltage_mV = get_battery_voltage_mV(adc1_handle, adc1_cali_handle);
  printf("[BATTERY_LEVEL]: voltage[mV] = %d ", voltage_mV);
  printf("soc[prc]: = %d\n", get_soc_from_voltage(voltage_mV)); // TODO czy \% poprawne?
}

void turn_90_right(int czas_obrtu_ms)
{
  set_motors_duty(-300, 300);
  vTaskDelay(pdMS_TO_TICKS(czas_obrtu_ms));
  set_motors_duty(0, 0);
}
void turn_90_left(int czas_obrtu_ms)
{
  set_motors_duty(300, -300);
  vTaskDelay(pdMS_TO_TICKS(czas_obrtu_ms));
  set_motors_duty(0, 0);
}

void pattern1(void)
{
  for (int i = 0; i < 4; i++)
  {
    set_motors_duty(400, 390);
    vTaskDelay(pdMS_TO_TICKS(1800));
    set_motors_duty(0, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    turn_90_right(450);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

extern void app_main(void)
{
  // --- INIT ---
  // adc_oneshot_unit_handle_t adc1_handle = get_adc1_handle();
  // adc_cali_handle_t adc1_cali_handle = get_cali_handle_adc1();

  // adc_oneshot_unit_handle_t adc2_handle = get_adc2_handle();
  // adc_cali_handle_t adc2_cali_handle = get_cali_handle_adc2();

  init_single_pcnt_unit(ENCODER_A_PIN, &g_encoderA_pcnt_handle);
  init_single_pcnt_unit(ENCODER_B_PIN, &g_encoderB_pcnt_handle);

  init_motor_control();

  init_timer();

  fcntl(fileno(stdin), F_SETFL, O_NONBLOCK); // wylacza czekanie na znak przy getchar
  // TODO czy fcntl ustawia flage globalnie?
  // --- END INIT ---

  // TODO dodac tablice z wartosciami napiec na sensorze linii i pobrac do niej wartosci

  // read_battery_level_uart_debug(adc1_handle, adc1_cali_handle);
  int i = 1;
  while (true)
  {
    if (i > 4)
    {
      i = 1;
    }

    set_motors_duty(500, 500);
    vTaskDelay(pdMS_TO_TICKS(1000));
    // printf("[DEBUG]: obroty B = %d\n", get_B_rotations(g_encoderB_pcnt_handle));
    // printf("[DEBUG]: predkosc chwilowa = %d\n", g_encoder_B_ticks_per_s);

    i++;
  }
}