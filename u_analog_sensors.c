#include "u_analog_sensors.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ADC1:
#define SOC_ADC_CHANNEL ADC_CHANNEL_0
#define REFLECTIVE_SENSOR_D1_CHANNEL ADC_CHANNEL_3
#define REFLECTIVE_SENSOR_D2_CHANNEL ADC_CHANNEL_6
#define REFLECTIVE_SENSOR_D3_CHANNEL ADC_CHANNEL_7
#define REFLECTIVE_SENSOR_D4_CHANNEL ADC_CHANNEL_4
#define REFLECTIVE_SENSOR_D5_CHANNEL ADC_CHANNEL_5

// ADC2:
#define REFLECTIVE_SENSOR_D6_CHANNEL ADC_CHANNEL_8
#define REFLECTIVE_SENSOR_D7_CHANNEL ADC_CHANNEL_9
#define REFLECTIVE_SENSOR_D8_CHANNEL ADC_CHANNEL_7

#define VOLTAGE_DIVIDER_BMS 3

adc_oneshot_unit_handle_t get_adc1_handle(void)
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
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, SOC_ADC_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, REFLECTIVE_SENSOR_D1_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, REFLECTIVE_SENSOR_D2_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, REFLECTIVE_SENSOR_D3_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, REFLECTIVE_SENSOR_D4_CHANNEL, &config));
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, REFLECTIVE_SENSOR_D5_CHANNEL, &config));

  return adc1_handle;
}

adc_cali_handle_t get_cali_handle_adc1(void)
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

adc_oneshot_unit_handle_t get_adc2_handle(void)
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

adc_cali_handle_t get_cali_handle_adc2(void)
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

int get_battery_voltage_mV(adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_handle)
{
  int raw_adc;
  int voltage_mV;
  adc_oneshot_read(adc1_handle, SOC_ADC_CHANNEL, &raw_adc);
  adc_cali_raw_to_voltage(adc1_cali_handle, raw_adc, &voltage_mV);
  return voltage_mV;
}

void update_line_sensor_table(int *line_sensor_table, adc_oneshot_unit_handle_t adc1_handle, adc_cali_handle_t adc1_cali_handle, adc_oneshot_unit_handle_t adc2_handle, adc_cali_handle_t adc2_cali_handle) // TODO how to properly take this argument
{
  int raw_adc_curr;
  int voltage_mV_curr;

  static const adc_channel_t table_of_adc1_channels[] = {REFLECTIVE_SENSOR_D1_CHANNEL,
                                                         REFLECTIVE_SENSOR_D2_CHANNEL, REFLECTIVE_SENSOR_D3_CHANNEL, REFLECTIVE_SENSOR_D4_CHANNEL,
                                                         REFLECTIVE_SENSOR_D5_CHANNEL};

  static const adc_channel_t table_of_adc2_channels[] = {REFLECTIVE_SENSOR_D6_CHANNEL,
                                                         REFLECTIVE_SENSOR_D7_CHANNEL,
                                                         REFLECTIVE_SENSOR_D8_CHANNEL};

  // adc1
  for (int i = 0; i < sizeof(table_of_adc1_channels); i++)
  {
    adc_oneshot_read(adc1_handle, table_of_adc1_channels[i], &raw_adc_curr);
    adc_cali_raw_to_voltage(adc1_cali_handle, raw_adc_curr, &voltage_mV_curr);
    line_sensor_table[i] = voltage_mV_curr;
  }

  // adc2
  for (int i = sizeof(table_of_adc1_channels); i < sizeof(table_of_adc2_channels) + sizeof(table_of_adc1_channels); i++)
  {
    adc_oneshot_read(adc2_handle, table_of_adc2_channels[i], &raw_adc_curr);
    adc_cali_raw_to_voltage(adc2_cali_handle, raw_adc_curr, &voltage_mV_curr);
    line_sensor_table[i] = voltage_mV_curr;
  }
  // TODO moze do zmiany powyzsze, troche zagmatwane

  // TODO dorobic do wszystkich funckji zwracajacych esp_ok akcje w przypadku kiedy nie wystapi esp_ok, co przekazac dalej? najlawtiej moze byc nie zwracac niczego z funkcji tylko pracowac na kopiach tak zeby nie trzeba bylo zwracac jakiejsc arbitralnej funkcji tylko zwrocic sukces porazka i w funkcji wywolujacej rozwiazywac sytuacje pojawienia sie bledu. Ewnetualnie jakies lepsze operaowanie na typach bledow. Generalnie trzeba bedzie uzbroic kod w przyszlosci.
}

int get_soc_from_voltage(int voltage_mV)
{
  // @returns estimated State Of Charge of the battery pack, values 0-100
  // TODO lookup do pamieci programu?
  int lookup_soc[][2] = {
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

// TODO zmienic wszystkie biblioteki zeby mialy wartosci stale statyczne ktorych wartosc mozna czerpac za pomoca getterow lub w cale nie sa potrzebne poza biblioteka
// chodzi o to zeby uniknac przekazywania handles albo koniecznosci korzystania z dziwnych typow wewentrz main