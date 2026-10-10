#include "u_analog_sensors.h"

#include "config.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

#include "esp_rom_sys.h"

#include "driver/gpio.h"

#define LEDS_ENABLED 1
#define LEDS_DISABLED 0

#define SOC_LOOKUP_SIZE 11

static adc_oneshot_unit_handle_t adc1_handle = NULL;
static adc_cali_handle_t adc1_cali_handle = NULL;

static adc_oneshot_unit_handle_t adc2_handle = NULL;
static adc_cali_handle_t adc2_cali_handle = NULL;

static adc_oneshot_unit_handle_t get_adc1_handle(void)
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

  const adc_channel_t adc1_channels_table[NUM_OF_CHANNELS_ADC1] = {
      SOC_ADC_CHANNEL,
      REFLECTIVE_SENSOR_D1_CHANNEL,
      REFLECTIVE_SENSOR_D2_CHANNEL,
      REFLECTIVE_SENSOR_D3_CHANNEL,
      REFLECTIVE_SENSOR_D4_CHANNEL,
      REFLECTIVE_SENSOR_D5_CHANNEL,
  };

  for (int i = 0; i < NUM_OF_CHANNELS_ADC1; i++)
  {
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, adc1_channels_table[i], &config));
  }

  return adc1_handle;
}

static adc_cali_handle_t get_cali_handle_adc1(void)
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

static adc_oneshot_unit_handle_t get_adc2_handle(void)
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
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, REFLECTIVE_SENSOR_D6_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, REFLECTIVE_SENSOR_D7_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc2_handle, REFLECTIVE_SENSOR_D8_CHANNEL, &config));

  return adc2_handle;
}

static adc_cali_handle_t get_cali_handle_adc2(void)
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

static void rs_enable_leds(void)
{
  gpio_set_level(LINE_LED_ON_PIN, LEDS_ENABLED);
}

static void rs_disable_leds(void)
{
  gpio_set_level(LINE_LED_ON_PIN, LEDS_DISABLED);
}

static void init_led_on_pin()
{
  gpio_reset_pin(LINE_LED_ON_PIN);
  gpio_set_direction(LINE_LED_ON_PIN, GPIO_MODE_OUTPUT);
}

void init_both_adc_units(void)
{
  adc1_handle = get_adc1_handle();
  adc1_cali_handle = get_cali_handle_adc1();

  adc2_handle = get_adc2_handle();
  adc2_cali_handle = get_cali_handle_adc2();

  init_led_on_pin();
  rs_disable_leds();
}

int get_battery_voltage_mV(void)
{
  int raw_adc;
  int voltage_mV;
  adc_oneshot_read(adc1_handle, SOC_ADC_CHANNEL, &raw_adc);
  adc_cali_raw_to_voltage(adc1_cali_handle, raw_adc, &voltage_mV);
  return voltage_mV;
}

int get_soc_from_voltage(int voltage_mV)
{
  static const int lookup_soc[SOC_LOOKUP_SIZE][2] = {
      {6000 / VOLTAGE_DIVIDER_BMS, 0},
      {6900 / VOLTAGE_DIVIDER_BMS, 10},
      {7200 / VOLTAGE_DIVIDER_BMS, 20},
      {7360 / VOLTAGE_DIVIDER_BMS, 30},
      {7460 / VOLTAGE_DIVIDER_BMS, 40},
      {7560 / VOLTAGE_DIVIDER_BMS, 50},
      {7680 / VOLTAGE_DIVIDER_BMS, 60},
      {7800 / VOLTAGE_DIVIDER_BMS, 70},
      {7960 / VOLTAGE_DIVIDER_BMS, 80},
      {8160 / VOLTAGE_DIVIDER_BMS, 90},
      {8400 / VOLTAGE_DIVIDER_BMS, 100}};

  if (voltage_mV <= lookup_soc[0][0])
  {
    return 0;
  }
  if (voltage_mV >= lookup_soc[SOC_LOOKUP_SIZE - 1][0])
  {
    return 100;
  }

  for (int i = 0; i < SOC_LOOKUP_SIZE; i++)
  {
    if (voltage_mV > lookup_soc[i][0] && voltage_mV <= lookup_soc[i + 1][0])
    {
      int x0 = lookup_soc[i][0];
      int x1 = lookup_soc[i + 1][0];

      int y0 = lookup_soc[i][1];
      int y1 = lookup_soc[i + 1][1];

      return y0 + (((y1 - y0) * (voltage_mV - x0)) / (x1 - x0));
    }
  }
  return -1;
}

int get_soc(void)
{
  return get_soc_from_voltage(get_battery_voltage_mV());
}

void update_line_sensor_table(int *line_sensor_table, int *line_mask_ptr, int *line_mask_popcount_ptr)
{
  int local_mask = 0;
  int local_mask_popcount = 0;

  int raw_adc_curr;
  int voltage_mV_curr;

  const adc_oneshot_unit_handle_t rs_unit_handles_table[NUM_OF_SENSOR_CHANNELS] = {
      // 5 from adc1
      adc1_handle,
      adc1_handle,
      adc1_handle,
      adc1_handle,
      adc1_handle,
      // 3 from adc2
      adc2_handle,
      adc2_handle,
      adc2_handle,
  };
  const adc_cali_handle_t rs_cali_handles_table[NUM_OF_SENSOR_CHANNELS] = {
      // 5 from adc1
      adc1_cali_handle,
      adc1_cali_handle,
      adc1_cali_handle,
      adc1_cali_handle,
      adc1_cali_handle,
      // 3 from adc2
      adc2_cali_handle,
      adc2_cali_handle,
      adc2_cali_handle,
  };
  static const adc_channel_t rs_channels_table[NUM_OF_SENSOR_CHANNELS] = {
      REFLECTIVE_SENSOR_D1_CHANNEL,
      REFLECTIVE_SENSOR_D2_CHANNEL,
      REFLECTIVE_SENSOR_D3_CHANNEL,
      REFLECTIVE_SENSOR_D4_CHANNEL,
      REFLECTIVE_SENSOR_D5_CHANNEL,
      REFLECTIVE_SENSOR_D6_CHANNEL,
      REFLECTIVE_SENSOR_D7_CHANNEL,
      REFLECTIVE_SENSOR_D8_CHANNEL,
  };

  rs_enable_leds();
  esp_rom_delay_us(20);

  for (int i = 0; i < NUM_OF_SENSOR_CHANNELS; i++)
  {
    adc_oneshot_read(rs_unit_handles_table[i], rs_channels_table[i], &raw_adc_curr);
    adc_cali_raw_to_voltage(rs_cali_handles_table[i], raw_adc_curr, &voltage_mV_curr);
    local_mask <<= 1;
    if (voltage_mV_curr > MIN_RS_VALUE_TO_BE_ON_LINE)
    {
      local_mask |= 1;
      local_mask_popcount++;
    }
    line_sensor_table[i] = voltage_mV_curr;
  }

  *line_mask_ptr = local_mask;
  *line_mask_popcount_ptr = local_mask_popcount;
  rs_disable_leds();
}

// [ ] dorobic do wszystkich funckji zwracajacych esp_ok akcje w przypadku kiedy nie wystapi esp_ok, co przekazac dalej? najlawtiej moze byc nie zwracac niczego z funkcji tylko pracowac na kopiach tak zeby nie trzeba bylo zwracac jakiejsc arbitralnej funkcji tylko zwrocic sukces porazka i w funkcji wywolujacej rozwiazywac sytuacje pojawienia sie bledu. Ewnetualnie jakies lepsze operaowanie na typach bledow. Generalnie trzeba bedzie uzbroic kod w przyszlosci.