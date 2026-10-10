#include "u_motors.h"

#include "config.h"

#include "driver/gpio.h"
#include "driver/ledc.h"

#include "esp_err.h"

// pwm for motors configuration
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_MODE LEDC_LOW_SPEED_MODE

#define PWM_MA_CHANNEL LEDC_CHANNEL_0
#define PWM_MB_CHANNEL LEDC_CHANNEL_1

#define FREQUENCY_FOR_PWM_TIMER 20000
#define MOTOR_PWM_FULL 1023 // 2^^pwm_clock_resolution - 1

#define STBY_DEACTIVATE_ENGINE 0
#define STBY_ACTIVATE_ENGINE 1

// motor control mask
#define MC_OUTPUT_PINS_MASK ((1ULL << AIN1_MC_PIN) | \
                             (1ULL << AIN2_MC_PIN) | \
                             (1ULL << BIN1_MC_PIN) | \
                             (1ULL << BIN2_MC_PIN) | \
                             (1ULL << STBY_MC_PIN))

#define AIN1_MOTOR_FORWARD 0
#define AIN2_MOTOR_FORWARD 1
#define AIN1_MOTOR_BACKWARD 1
#define AIN2_MOTOR_BACKWARD 0

#define BIN1_MOTOR_FORWARD 0
#define BIN2_MOTOR_FORWARD 1
#define BIN1_MOTOR_BACKWARD 1
#define BIN2_MOTOR_BACKWARD 0

void sleep_motors(void)
{
  gpio_set_level(STBY_MC_PIN, STBY_DEACTIVATE_ENGINE);
}

void wake_up_motors(void)
{
  gpio_set_level(STBY_MC_PIN, STBY_ACTIVATE_ENGINE);
}

static inline void init_dual_pwm(void)
{
  ledc_timer_config_t timer_conf = {
      .speed_mode = LEDC_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT, // last value 2**10 - 1
      .timer_num = LEDC_TIMER,
      .freq_hz = FREQUENCY_FOR_PWM_TIMER,
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

inline void init_motor_control(void)
{
  gpio_config_t mc_output_pins_conf = {
      .pin_bit_mask = MC_OUTPUT_PINS_MASK,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE, // zacznie od stanu niskiego
      .intr_type = GPIO_INTR_DISABLE};      // bez przerwan
  ESP_ERROR_CHECK(gpio_config(&mc_output_pins_conf));

  init_dual_pwm();

  gpio_set_level(AIN1_MC_PIN, AIN1_MOTOR_FORWARD);
  gpio_set_level(AIN2_MC_PIN, AIN2_MOTOR_FORWARD);

  gpio_set_level(BIN1_MC_PIN, BIN1_MOTOR_FORWARD);
  gpio_set_level(BIN2_MC_PIN, BIN2_MOTOR_FORWARD);

  gpio_set_level(STBY_MC_PIN, STBY_ACTIVATE_ENGINE);
}

void set_motor_A_forward(void)
{
  gpio_set_level(AIN1_MC_PIN, AIN1_MOTOR_FORWARD);
  gpio_set_level(AIN2_MC_PIN, AIN2_MOTOR_FORWARD);
}

void set_motor_B_forward(void)
{
  gpio_set_level(BIN1_MC_PIN, BIN1_MOTOR_FORWARD);
  gpio_set_level(BIN2_MC_PIN, BIN2_MOTOR_FORWARD);
}

void set_motor_A_backward(void)
{
  gpio_set_level(AIN1_MC_PIN, AIN1_MOTOR_BACKWARD);
  gpio_set_level(AIN2_MC_PIN, AIN2_MOTOR_BACKWARD);
}

void set_motor_B_backward(void)
{
  gpio_set_level(BIN1_MC_PIN, BIN1_MOTOR_BACKWARD);
  gpio_set_level(BIN2_MC_PIN, BIN2_MOTOR_BACKWARD);
}

void set_motors_duty(int duty_A, int duty_B)
{
  if (duty_A < 0)
  {
    duty_A = -duty_A;
    set_motor_A_backward();
  }
  else
  {
    set_motor_A_forward();
  }

  if (duty_A > MOTOR_PWM_FULL)
  {
    duty_A = MOTOR_PWM_FULL;
  }

  if (duty_B < 0)
  {
    duty_B = -duty_B;
    set_motor_B_backward();
  }
  else
  {
    set_motor_B_forward();
  }

  if (duty_B > MOTOR_PWM_FULL)
  {
    duty_B = MOTOR_PWM_FULL;
  }

  // make sure that duty_A and duty_B are between 0 and (2^^pwm_clock_resolution - 1)
  ledc_set_duty(LEDC_MODE, PWM_MA_CHANNEL, duty_A);
  ledc_update_duty(LEDC_MODE, PWM_MA_CHANNEL);

  ledc_set_duty(LEDC_MODE, PWM_MB_CHANNEL, duty_B);
  ledc_update_duty(LEDC_MODE, PWM_MB_CHANNEL);
}
