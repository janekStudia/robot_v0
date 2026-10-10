#include "u_indication_led.h"

#include "config.h"

#include "driver/gpio.h"

#define LED_ON 1
#define LED_OFF 0

inline void init_indication_led(void)
{
  gpio_reset_pin(INDICATION_LED_GPIO_NUM);
  gpio_set_direction(INDICATION_LED_GPIO_NUM, GPIO_MODE_OUTPUT);
}

void indication_led_on(void)
{
  gpio_set_level(INDICATION_LED_GPIO_NUM, LED_ON);
}

void indication_led_off(void)
{
  gpio_set_level(INDICATION_LED_GPIO_NUM, LED_OFF);
}