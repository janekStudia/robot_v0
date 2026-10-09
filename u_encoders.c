#include "u_encoders.h"

#include "driver/gpio.h"
#include "driver/pulse_cnt.h"

#define ENCODER_A_PIN GPIO_NUM_16
#define ENCODER_B_PIN GPIO_NUM_4

#define PCNT_LOW_LIMIT -32768 // -2^15
#define PCNT_HIGH_LIMIT 32767 // 2^15 - 1

#define MAX_GLITCH_NS 1000

#define UNUSED -1

static pcnt_unit_handle_t encA_pcnt_handle = NULL;
static pcnt_unit_handle_t encB_pcnt_handle = NULL;

static inline void init_and_start_single_pcnt_unit(int gpio_pin, pcnt_unit_handle_t *unit_handle)
{
  pcnt_unit_config_t unit_config = {
      .low_limit = PCNT_LOW_LIMIT,
      .high_limit = PCNT_HIGH_LIMIT,
      .flags = {.accum_count = 1}, // pcnt_unit_get_count() will return accumulated value
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, unit_handle));

  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = MAX_GLITCH_NS,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*unit_handle, &filter_config));

  pcnt_chan_config_t chan_config = {
      .edge_gpio_num = gpio_pin,
      .level_gpio_num = UNUSED,
  };
  pcnt_channel_handle_t chan_handle = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(*unit_handle, &chan_config, &chan_handle));

  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));

  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

  ESP_ERROR_CHECK(pcnt_unit_enable(*unit_handle));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(*unit_handle));
  ESP_ERROR_CHECK(pcnt_unit_start(*unit_handle));
}

inline void init_and_start_encoders_pcnt_units(void)
{
  init_and_start_single_pcnt_unit(ENCODER_A_PIN, &encA_pcnt_handle);
  init_and_start_single_pcnt_unit(ENCODER_B_PIN, &encB_pcnt_handle);
}

int get_encA_count(void)
{
  static int count;
  pcnt_unit_get_count(encA_pcnt_handle, &count);
  return count;
}

int get_encB_count(void)
{
  static int count;
  pcnt_unit_get_count(encB_pcnt_handle, &count);
  return count;
}

void clear_both_encoders_count(void)
{
  pcnt_unit_clear_count(encA_pcnt_handle);
  pcnt_unit_clear_count(encA_pcnt_handle);
}

void clear_encA_count(void)
{
  pcnt_unit_clear_count(encA_pcnt_handle);
}

void clear_encB_count(void)
{
  pcnt_unit_clear_count(encB_pcnt_handle);
}