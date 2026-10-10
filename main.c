#include <stdio.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// #include "driver/gpio.h"

// #include "esp_err.h"

// wlasne
#include "u_motors.h"
#include "u_encoders.h"
#include "u_analog_sensors.h"
// #include "u_indication_led.h"
#include "u_control.h"

void read_battery_level_uart_debug(void)
{
  int voltage_mV = get_battery_voltage_mV();
  printf("[BATTERY_LEVEL]: voltage[mV] = %d ", voltage_mV);
  printf("soc[prc]: = %d\n", get_soc_from_voltage(voltage_mV));
}

void control_task(void *arg)
{
  TickType_t last_wake = xTaskGetTickCount();

  for (;;)
  {
    control_loop();
    vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(10));
  }
}

extern void app_main(void)
{
  // --- INIT ---
  init_motor_control();
  init_both_adc_units();
  init_and_start_encoders_pcnt_units();
  // init_indication_led();
  // --- END INIT ---

  xTaskCreate(
      control_task,   // Wskaźnik na funkcję zadania
      "control_task", // Nazwa własna zadania (do debugowania)
      2048,           // Rozmiar stosu w słowach/bajtach
      NULL,           // Parametr przekazywany do zadania (arg)
      5,              // Priorytet zadania (wyższy numer = wyższy priorytet)
      NULL            // Uchwyt zadania (TaskHandle_t), opcjonalnie NULL
  );

  vTaskDelay(pdMS_TO_TICKS(2000));

  start_line_following(200);
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// [ ] poprawic pliki naglowkowe