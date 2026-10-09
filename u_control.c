#include "u_control.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "u_motors.h"
#include "u_analog_sensors.h"
#include "u_encoders.h"

#define RS_DATA_TABLE_SIZE 8
#define MAX_PWM_OUTPUT_DIFF_P_LINE_CONTROLER 200
#define MAX_RS_ERROR_VAL_P_CONTROLER (3 * 2500)

#define MAX_PWM_OUTPUT_DIFF_P_MANOUVER_CONTROLER 50
#define MAX_STEP_DIFF_ERROR_VAL_P_CONTROLER 10

#define PWM_BASE_VALUE 200

volatile static int p_line_controler_enabled_flag = true;
volatile static int junction_detection_enabled_flag = true;

volatile static int p_manouver_controler_enabled_flag = false;
volatile static int step_check_enabled_flag = false;

static int rs_values_table[RS_DATA_TABLE_SIZE] = {0};
static int rs_values_mask = 0;
static int rs_values_mask_popcount = 0;

static int goal_steps_A = 0;
static int goal_steps_B = 0;

static int pwm_manouver_A = 0;
static int pwm_manouver_B = 0;

static int pwm_real_A = PWM_BASE_VALUE;
static int pwm_real_B = PWM_BASE_VALUE;

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

void turn(int degrees)
{
  p_line_controler_enabled_flag = false;
  junction_detection_enabled_flag = false;
}

void go_forward(int steps)
{
  goal_steps_A = steps;
  goal_steps_B = steps;

  p_manouver_controler_enabled_flag = true;
  step_check_enabled_flag = true;

  clear_both_encoders_count();

  pwm_manouver_A = 200;
  pwm_manouver_B = 200;

  p_line_controler_enabled_flag = false;
  junction_detection_enabled_flag = false;
}

void manouver_ended(void)
{
  p_manouver_controler_enabled_flag = false;
  step_check_enabled_flag = false;
  clear_both_encoders_count();
  pwm_real_B = 0;
  pwm_real_B = 0;
  // p_line_controler_enabled_flag = true;
  // junction_detection_enabled_flag = true;
}

void check_steps(void)
{
  if (!step_check_enabled_flag)
  {
    return;
  }
  // printf("a_count:%d, b_count:%d, pwm_real_a:%d pwm_real_b:%d\n", get_encA_count(), get_encB_count(), pwm_real_A, pwm_real_B);
  if (get_encB_count() >= goal_steps_B && get_encA_count() >= goal_steps_A)
  {
    manouver_ended();
    return;
  }

  if (get_encA_count() >= goal_steps_A)
  {
    p_manouver_controler_enabled_flag = false;
    pwm_real_A = 0;
  }
  if (get_encB_count() >= goal_steps_B)
  {
    p_manouver_controler_enabled_flag = false;
    pwm_real_B = 0;
  }
}

void p_manouver_controler(void)
{
  if (!p_manouver_controler_enabled_flag)
  {
    return;
  }

  int p_error = get_encA_count() - get_encB_count();
  int p_result = ((MAX_PWM_OUTPUT_DIFF_P_MANOUVER_CONTROLER * p_error) / 2) / MAX_STEP_DIFF_ERROR_VAL_P_CONTROLER;
  pwm_real_A = pwm_manouver_A - p_result;
  pwm_real_B = pwm_manouver_B + p_result;
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
    // turn_left();
    on_junction_counter = 0;
    return;
  }
}

void p_line_controler(void)
{ // RS = Reflective Sensor
  if (!p_line_controler_enabled_flag)
  {
    return;
  }
  static const int multipliers[RS_DATA_TABLE_SIZE] = {-3, -3, -2, -1, 1, 2, 3, 3};

  int p_error = 0;
  for (int i = 0; i < RS_DATA_TABLE_SIZE; i++)
  {
    p_error += rs_values_table[i] * multipliers[i];
  }

  int p_result = ((MAX_PWM_OUTPUT_DIFF_P_LINE_CONTROLER * p_error) / 2) / MAX_RS_ERROR_VAL_P_CONTROLER;
  pwm_real_A = PWM_BASE_VALUE + p_result;
  pwm_real_B = PWM_BASE_VALUE - p_result;
}

void update_rs_values(void)
{
  update_line_sensor_table(rs_values_table, &rs_values_mask, &rs_values_mask_popcount);
}