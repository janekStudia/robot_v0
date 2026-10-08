#include "u_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "u_motors.h"
#include "u_analog_sensors.h"

#define RS_DATA_TABLE_SIZE 8
#define MAX_PWM_OUTPUT_DIFF_P_CONTROLER 200
#define MAX_RS_ERROR_VAL_P_CONTROLER (3 * 2500)

#define PWM_BASE_VALUE 200

volatile static int p_controler_enabled_flag = true;
volatile static int junction_detection_enabled_flag = true;

static int rs_values_table[RS_DATA_TABLE_SIZE] = {0};
static int rs_values_mask = 0;
static int rs_values_mask_popcount = 0;

static int pwm_real_A = PWM_BASE_VALUE;
static int pwm_real_B = PWM_BASE_VALUE;

static TimerHandle_t xTimer1 = NULL;

static void end_of_manouver_callback(TimerHandle_t xTimer)
{
  p_controler_enabled_flag = true;
  junction_detection_enabled_flag = true;
  pwm_real_A = PWM_BASE_VALUE;
  pwm_real_B = PWM_BASE_VALUE;
}

void init_my_timer(void)
{
  xTimer1 = xTimerCreate(
      "Timer1",
      pdMS_TO_TICKS(1000),
      pdFALSE,
      (void *)0,
      end_of_manouver_callback);
}

void update_pwm(void)
{
  set_motors_duty(pwm_real_A, pwm_real_B);
}

void uprint_rs_value_table()
{
  printf("[DEBUG] rs_values_table contents(1-8) in mV = [ ");

  for (int i = 0; i < RS_DATA_TABLE_SIZE; i++)
  {
    printf("%d ", rs_values_table[i]);
  }
  printf("]\n");
}

void turn_left(void)
{
  pwm_real_A = 150;
  pwm_real_B = -150;
  p_controler_enabled_flag = false;
  junction_detection_enabled_flag = false;
  if (xTimer1 != NULL)
  {
    xTimerStart(xTimer1, pdMS_TO_TICKS(10));
  }
}

void turn_right(void)
{
  pwm_real_A = -150;
  pwm_real_B = 150;
  p_controler_enabled_flag = false;
  junction_detection_enabled_flag = false;
  if (xTimer1 != NULL)
  {
    xTimerStart(xTimer1, pdMS_TO_TICKS(10));
  }
  // TODO skalibrowac obrot na ticki enkodera
}

void skip_junction(void)
{
  pwm_real_A = PWM_BASE_VALUE;
  pwm_real_B = PWM_BASE_VALUE;
  p_controler_enabled_flag = false;
  junction_detection_enabled_flag = false;
  if (xTimer1 != NULL)
  {
    xTimerStart(xTimer1, pdMS_TO_TICKS(10));
  }
  // TODO zmienic delay na wylaczenie flagi p_controlera, powrotu do petli glownej i odlozenie w czasie wlaczenie jej spowrotem
}

void junction_detector(void)
{
  if (!junction_detection_enabled_flag)
  {
    return;
  }

  static int on_junction_counter = 0;
  if (rs_values_mask_popcount > 3) // is on junction
  {
    on_junction_counter++;
  }
  else if (on_junction_counter > 0)
  {
    on_junction_counter--; // decreases until 0 when out of junction to take care of residual counter values for future junctions
  }

  if (on_junction_counter > 20)
  {
    // code for when on junction for sure
    turn_left();
    on_junction_counter = 0;
    return;
  }
}

void p_controler(void)
{ // RS = Reflective Sensor
  if (!p_controler_enabled_flag)
  {
    return;
  }
  static const int multipliers[RS_DATA_TABLE_SIZE] = {-3, -3, -2, -1, 1, 2, 3, 3};

  int p_error = 0;
  for (int i = 0; i < RS_DATA_TABLE_SIZE; i++)
  {
    p_error += rs_values_table[i] * multipliers[i];
  }

  int p_result = ((MAX_PWM_OUTPUT_DIFF_P_CONTROLER * p_error) / 2) / MAX_RS_ERROR_VAL_P_CONTROLER;
  pwm_real_A = PWM_BASE_VALUE + p_result;
  pwm_real_B = PWM_BASE_VALUE - p_result;
}

void update_rs_values(void)
{
  update_line_sensor_table(rs_values_table, &rs_values_mask, &rs_values_mask_popcount);
}