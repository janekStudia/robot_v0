#include "u_encoders.h"

#include "driver/pulse_cnt.h"

void init_single_pcnt_unit(int gpio_pin, pcnt_unit_handle_t *unit_handle)
{
  pcnt_unit_config_t unit_config = {
      .low_limit = -32768, // TODO delete magic numbers
      .high_limit = 32767,
      .flags = {.accum_count = 1},
  };
  ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, unit_handle));

  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = 100,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(*unit_handle, &filter_config));

  pcnt_chan_config_t chan_config = {
      .edge_gpio_num = gpio_pin,
      .level_gpio_num = -1, // -1 means unused
  };
  pcnt_channel_handle_t chan_handle = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(*unit_handle, &chan_config, &chan_handle));

  ESP_ERROR_CHECK(pcnt_channel_set_edge_action(chan_handle, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));

  ESP_ERROR_CHECK(pcnt_channel_set_level_action(chan_handle, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_KEEP));

  ESP_ERROR_CHECK(pcnt_unit_enable(*unit_handle));
  ESP_ERROR_CHECK(pcnt_unit_clear_count(*unit_handle));
  ESP_ERROR_CHECK(pcnt_unit_start(*unit_handle));
}

int get_rotations(pcnt_unit_handle_t encoder_pcnt_handle)
{
  int count;
  if (!pcnt_unit_get_count(encoder_pcnt_handle, &count)) // ESP_OK = 0 , jest zwracane przy sukcesie
  {
    return count;
  }
  return -1;
}