#include <stdio.h>
#include <fcntl.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/uart.h"

#include "esp_err.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

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

// magic numbers
#define DZIELNIK_NAPIECIA_Z_BMS 3
#define FREQUENCY_FOR_PWM_TIMER 20000
#define MOTOR_PWM_FULL 1023 // 2^^pwm_clock_resolution - 1

#define STBY_DEACTIVATE_ENGINE 0
#define STBY_ACTIVATE_ENGINE 1

#define AIN1_MOTOR_FORWARD 0
#define AIN2_MOTOR_FORWARD 1
#define AIN1_MOTOR_BACKWARD 1
#define AIN2_MOTOR_BACKWARD 0

#define BIN1_MOTOR_FORWARD 0
#define BIN2_MOTOR_FORWARD 1
#define BIN1_MOTOR_BACKWARD 1
#define BIN2_MOTOR_BACKWARD 0

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

void init_motor_control(void)
{
  gpio_config_t mc_output_pins_conf = {
      .pin_bit_mask = MC_OUTPUT_PINS_MASK,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_ENABLE, // zacznie od stanu niskiego
      .intr_type = GPIO_INTR_DISABLE};      // bez przerwan
  ESP_ERROR_CHECK(gpio_config(&mc_output_pins_conf));

  gpio_set_level(STBY_MC_PIN, STBY_DEACTIVATE_ENGINE);

  init_dual_pwm();

  gpio_set_level(AIN1_MC_PIN, AIN1_MOTOR_FORWARD);
  gpio_set_level(AIN2_MC_PIN, AIN2_MOTOR_FORWARD);

  gpio_set_level(BIN1_MC_PIN, BIN1_MOTOR_FORWARD);
  gpio_set_level(BIN2_MC_PIN, BIN2_MOTOR_FORWARD);

  gpio_set_level(STBY_MC_PIN, STBY_ACTIVATE_ENGINE);
}

// TODO czy tak definiowac flagi globalne
int is_A_forward = true;
int is_B_forward = true;

void set_motors_duty(int duty_A, int duty_B)
{
  // TODO przemyslec jak przelozyc wyjscie PID na duty_A i duty_B
  if (duty_A < 0)
  {
    duty_A = -duty_A;
    if (is_A_forward)
    {
      set_motor_A_backward();
    }
  }
  else if (!is_A_forward)
  {
    set_motor_A_forward();
  }

  if (duty_B < 0)
  {
    duty_B = -duty_B;
    if (is_B_forward)
    {
      set_motor_B_backward();
    }
  }
  else
  {
    if (!is_B_forward)
    {
      set_motor_B_forward();
    }
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

void set_motor_A_forward()
{
  gpio_set_level(AIN1_MC_PIN, AIN1_MOTOR_FORWARD);
  gpio_set_level(AIN2_MC_PIN, AIN2_MOTOR_FORWARD);
  is_A_forward = true;
}

void set_motor_B_forward()
{
  gpio_set_level(BIN1_MC_PIN, BIN1_MOTOR_FORWARD);
  gpio_set_level(BIN2_MC_PIN, BIN2_MOTOR_FORWARD);
  is_B_forward = true;
}

void set_motor_A_backward()
{
  gpio_set_level(AIN1_MC_PIN, AIN1_MOTOR_BACKWARD);
  gpio_set_level(AIN2_MC_PIN, AIN2_MOTOR_BACKWARD);
  is_A_forward = false;
}

void set_motor_B_backward()
{
  gpio_set_level(BIN1_MC_PIN, BIN1_MOTOR_BACKWARD);
  gpio_set_level(BIN2_MC_PIN, BIN2_MOTOR_BACKWARD);
  is_B_forward = false;
}

// -- start adc1 (battery sensor) configuration and calibration --
adc_oneshot_unit_handle_t get_adc1_soc_handle()
{
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_cfg1 = {
      .unit_id = ADC_UNIT_1,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg1, &adc1_handle));

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_12, // 0 - 3.3V
  };
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_6, &config));
  return adc1_handle;
}

adc_cali_handle_t get_cali_handle_adc1_battery()
{
  adc_cali_handle_t adc1_cali_handle = NULL;

  adc_cali_line_fitting_config_t cali_cfg = {
      .unit_id = ADC_UNIT_1,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_cfg, &adc1_cali_handle));
  return adc1_cali_handle;
}

int get_battery_voltage_mV(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_handle)
{
  int raw_adc;
  int voltage_mV;
  adc_oneshot_read(adc1_handle, ADC_CHANNEL_6, &raw_adc);
  adc_cali_raw_to_voltage(adc1_cali_handle, raw_adc, &voltage_mV);
  return voltage_mV;
}
// -- end adc1 (battery sensor) configuration and calibration --

// -- start adc2 (line sensor) configuration and calibration --
adc_oneshot_unit_handle_t get_adc2_line_sensor_handle()
{
  adc_oneshot_unit_handle_t adc2_handle;
  adc_oneshot_unit_init_cfg_t init_cfg2 = {
      .unit_id = ADC_UNIT_2,
  };
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg2, &adc2_handle));

  adc_oneshot_chan_cfg_t config = {
      .bitwidth = ADC_BITWIDTH_DEFAULT,
      .atten = ADC_ATTEN_DB_12, // 0 - 3.3V
  };
  for (adc_channel_t i = 0; i < 8; i++)
  {
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, i, &config));
  }

  return adc2_handle;
}

adc_cali_handle_t get_cali_handle_adc2_line_sensor()
{
  adc_cali_handle_t adc2_cali_handle = NULL;

  adc_cali_line_fitting_config_t cali_cfg = {
      .unit_id = ADC_UNIT_2,
      .atten = ADC_ATTEN_DB_12,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };
  ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&cali_cfg, &adc2_cali_handle));
  return adc2_cali_handle;
}

void update_line_sensor_table(int *line_sensor_table, adc_oneshot_unit_handle_t adc2_handle, adc_cali_handle_t adc2_cali_handle) // TODO how to properly take this argument
{
  int raw_adc_curr;
  int voltage_mV_curr;
  for (int i = 0; i < sizeof(line_sensor_table); i++)
  {
    adc_oneshot_read(adc2_handle, ADC_CHANNEL_6, &raw_adc_curr);
    adc_cali_raw_to_voltage(adc2_cali_handle, raw_adc_curr, &voltage_mV_curr);
    line_sensor_table[i] = voltage_mV_curr;
    // TODO dorobic do wszystkich funckji zwracajacych esp_ok akcje w przypadku kiedy nie wystapi esp_ok, co przekazac dalej? najlawtiej moze byc nie zwracac niczego z funkcji tylko pracowac na kopiach tak zeby nie trzeba bylo zwracac jakiejsc arbitralnej funkcji tylko zwrocic sukces porazka i w funkcji wywolujacej rozwiazywac sytuacje pojawienia sie bledu. Ewnetualnie jakies lepsze operaowanie na typach bledow. Generalnie trzeba bedzie uzbroic kod w przyszlosci.
  }
}

// -- end adc2 (line sensor) configuration and calibration --

int get_soc_from_voltage(int voltage_mV)
{
  // TODO lookup do pamieci programu?
  int lookup_soc[][2] = {
      {6000 / DZIELNIK_NAPIECIA_Z_BMS, 0},
      {6900 / DZIELNIK_NAPIECIA_Z_BMS, 10},
      {7200 / DZIELNIK_NAPIECIA_Z_BMS, 20},
      {7360 / DZIELNIK_NAPIECIA_Z_BMS, 30},
      {7460 / DZIELNIK_NAPIECIA_Z_BMS, 40},
      {7560 / DZIELNIK_NAPIECIA_Z_BMS, 50},
      {7680 / DZIELNIK_NAPIECIA_Z_BMS, 60},
      {7800 / DZIELNIK_NAPIECIA_Z_BMS, 70},
      {7960 / DZIELNIK_NAPIECIA_Z_BMS, 80},
      {8160 / DZIELNIK_NAPIECIA_Z_BMS, 90},
      {8400 / DZIELNIK_NAPIECIA_Z_BMS, 100}};

  if (voltage_mV <= lookup_soc[0][0])
  {
    return 0;
  }
  if (voltage_mV >= lookup_soc[-1][0])
  {
    return 100;
  }

  for (int i = 0; i < sizeof(lookup_soc); i++)
  {
    if (voltage_mV > lookup_soc[i][0] && voltage_mV <= lookup_soc[i + 1][0])
    {
      int x0 = lookup_soc[i][0];
      int x1 = lookup_soc[i + 1][0];

      int y0 = lookup_soc[i][1];
      int y1 = lookup_soc[i + 1][1];

      return y0 + ((y1 - y0) / (x1 - x0)) * (voltage_mV - x0);
    }
  }
  return -1;
}

void keyboard_engine_speed_control_debug()
{

  while (true)
  {
    uint32_t duty_AB = 1;

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

    case (int)'e':
      return;

    default:
      break;
    }

    set_motors_duty(duty_AB, duty_AB);
    // printf("[DEBUG]: duty value = %ld\n", duty_AB);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void read_battery_level_uart_debug(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_handle)
{
  int voltage_mV = get_battery_voltage_mV(adc1_handle, adc1_cali_handle);
  printf("[BATTERY_LEVEL]: voltage[mV] = %d ", voltage_mV);
  printf("soc[prc]: = %d\n", get_soc_from_voltage(voltage_mV)); // TODO czy \% poprawne?
}

extern void app_main(void)
{
  // --- INIT ---
  adc_oneshot_unit_handle_t adc1_handle = get_adc1_soc_handle();
  adc_cali_handle_t adc1_cali_handle = get_cali_handle_adc1_battery();

  adc_oneshot_unit_handle_t adc2_handle = get_adc2_line_sensor_handle();
  adc_cali_handle_t adc2_cali_handle = get_cali_handle_adc2_battery();

  init_motor_control();

  fcntl(fileno(stdin), F_SETFL, O_NONBLOCK); // wylacza czekanie na znak przy getchar
  // TODO czy fcntl ustawia flage globalnie?
  // --- END INIT ---

  // TODO dodac tablice z wartosciami napiec na sensorze linii i pobrac do niej wartosci

  while (1)
  {
    read_battery_level_uart_debug(adc1_handle, adc1_cali_handle);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}