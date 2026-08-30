#include <stdio.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"
#include "esp_err.h"

// pins configuration
#define PWM_MA_GPIO GPIO_NUM_23
#define AIN2_MC_PIN GPIO_NUM_22
#define AIN1_MC_PIN GPIO_NUM_21
#define STBY_MC_PIN GPIO_NUM_19
#define BIN1_MC_PIN GPIO_NUM_18
#define BIN2_MC_PIN GPIO_NUM_5
#define PWM_MB_GPIO GPIO_NUM_4

// motor control mask
#define MC_OUTPUT_PINS_MASK ((1ULL << AIN1_MC_PIN) | \
                             (1ULL << AIN2_MC_PIN) | \
                             (1ULL << BIN1_MC_PIN) | \
                             (1ULL << BIN2_MC_PIN) | \
                             (1ULL << STBY_MC_PIN))

// pwm for motors configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE

#define PWM_MA_CHANNEL LEDC_CHANNEL_0
#define PWM_MB_CHANNEL LEDC_CHANNEL_1

void sleep_motors()
{
  gpio_set_level(STBY_MC_PIN, 0);
}

void wake_up_motors()
{
  gpio_set_level(STBY_MC_PIN, 1);
}

void init_dual_pwm(void)
{
  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = LEDC_TIMER,
      .freq_hz = 20000,
      .clk_cfg = LEDC_AUTO_CLK,
      .deconfigure = 0};
  ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

  ledc_channel_config_t ch_mA_conf = {
      .gpio_num = PWM_MA_GPIO,
      .speed_mode = LEDC_MODE,
      .channel = PWM_MA_CHANNEL,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .flags = {.output_invert = 0},
      .deconfigure = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ch_mA_conf));

  ledc_channel_config_t ch_mB_conf = {
      .gpio_num = PWM_MB_GPIO,
      .speed_mode = LEDC_MODE,
      .channel = PWM_MB_CHANNEL,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LEDC_TIMER,
      .duty = 0,
      .hpoint = 0,
      .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
      .flags = {.output_invert = 0},
      .deconfigure = 0};
  ESP_ERROR_CHECK(ledc_channel_config(&ch_mB_conf));
}

void init_motor_control(void)
{
  gpio_config_t mc_output_pins_conf = {
      .pin_bit_mask = MC_OUTPUT_PINS_MASK,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE, // zacznie od stanu niskiego
      .intr_type = GPIO_INTR_DISABLE};      // bez przerwan
  ESP_ERROR_CHECK(gpio_config(&mc_output_pins_conf));

  gpio_set_level(STBY_MC_PIN, 0);

  init_dual_pwm();

  gpio_set_level(AIN1_MC_PIN, 0);
  gpio_set_level(AIN2_MC_PIN, 1);

  gpio_set_level(BIN1_MC_PIN, 1);
  gpio_set_level(BIN2_MC_PIN, 0);

  gpio_set_level(STBY_MC_PIN, 1);
}

void set_motors_duty(uint32_t duty_A, uint32_t duty_B)
{
  // make sure that duty_A and duty_B are between 0 and (2^^pwm_clock_resolution - 1)
  ledc_set_duty(LEDC_MODE, PWM_MA_CHANNEL, duty_A);
  ledc_update_duty(LEDC_MODE, PWM_MA_CHANNEL);

  ledc_set_duty(LEDC_MODE, PWM_MB_CHANNEL, duty_B);
  ledc_update_duty(LEDC_MODE, PWM_MB_CHANNEL);
}

extern void app_main(void)
{
  init_motor_control();
  fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);

  uint32_t duty_AB = 1;
  sleep_motors();

  // sleep_motors();
  while (true)
  {
    int32_t c = getchar();
    switch (c)
    {
    case (int)'1':
      duty_AB = 100;
      break;
    case (int)'2':
      duty_AB = 200;
      break;
    case (int)'3':
      duty_AB = 300;
      break;
    case (int)'4':
      duty_AB = 400;
      break;
    case (int)'5':
      duty_AB = 500;
      break;
    case (int)'6':
      duty_AB = 600;
      break;
    case (int)'7':
      duty_AB = 700;
      break;
    case (int)'8':
      duty_AB = 800;
      break;
    case (int)'9':
      duty_AB = 1023;
      break;

    case (int)'s':
      sleep_motors();
      break;
    case (int)'w':
      wake_up_motors();
      break;

    default:
      break;
    }

    set_motors_duty(duty_AB, duty_AB);
    printf("[DEBUG]: duty value = %ld\n", duty_AB);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}