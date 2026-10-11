#include "u_control.h"

#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"

#include "u_motors.h"
#include "u_analog_sensors.h"
#include "u_encoders.h"

#define RS_DATA_TABLE_SIZE 8

#define JUNCTION_NODES 7

typedef enum
{
  FORWARD,
  RIGHT,
  BACKWARD,
  LEFT,
  GOAL_REACHED
} direction_t;

typedef enum
{
  CONTROL_STATE_STOPPED,
  CONTROL_STATE_LINE_FOLLOWING,
  CONTROL_STATE_MANOEUVRE
} control_state_t;

static control_state_t control_state = CONTROL_STATE_STOPPED;

// volatile static int p_line_controler_enabled_flag = true;
// volatile static int junction_detection_enabled_flag = true;

// volatile static int p_manoeuvre_controler_enabled_flag = false;
// volatile static int step_check_enabled_flag = false;

static const char *TAG = "u_control";

static int rs_values_table[RS_DATA_TABLE_SIZE] = {0};
static int rs_values_mask = 0;
static int rs_values_mask_popcount = 0;

static int goal_steps_A = 0;
static int goal_steps_B = 0;

static int base_speed_pwm_A = 0;
static int base_speed_pwm_B = 0;

static int pwm_A = 0;
static int pwm_B = 0;

// [ ] change pairs of values into structs

static int route[JUNCTION_NODES] = {RIGHT, LEFT, LEFT, LEFT, BACKWARD, GOAL_REACHED, GOAL_REACHED};

void uprint_rs_value_table()
{
  printf("[DEBUG] rs_values_table contents(1-8) in mV = [ ");

  for (int i = 0; i < RS_DATA_TABLE_SIZE; i++)
  {
    printf("%d ", rs_values_table[i]);
  }
  printf("]\n");
}

void start_manoeuvre(int steps_A, int steps_B, int speed_pwm)
{
  if (steps_A >= 0)
  {
    goal_steps_A = steps_A;
    base_speed_pwm_A = speed_pwm;
  }
  else
  {
    goal_steps_A = -steps_A;
    base_speed_pwm_A = -speed_pwm;
  }
  if (steps_B >= 0)
  {
    goal_steps_B = steps_B;
    base_speed_pwm_B = speed_pwm;
  }
  else
  {
    goal_steps_B = -steps_B;
    base_speed_pwm_B = -speed_pwm;
  }

  control_state = CONTROL_STATE_MANOEUVRE;

  clear_both_encoders_count();
}

void start_stopped_state(void)
{
  control_state = CONTROL_STATE_STOPPED;
  pwm_A = 0;
  pwm_B = 0;
}

void start_line_following(int speed_pwm)
{
  control_state = CONTROL_STATE_LINE_FOLLOWING;
  base_speed_pwm_A = speed_pwm;
  base_speed_pwm_B = speed_pwm;
}

static void check_steps(void)
{
  if (control_state != CONTROL_STATE_MANOEUVRE)
  {
    return;
  }
  // ESP_LOGI(TAG, "%d, %d", get_encB_count(), get_encA_count());

  if (get_encB_count() >= goal_steps_B && get_encA_count() >= goal_steps_A)
  {
    // start_stopped_state();
    start_line_following(200);

    return;
  }
}

static void p_manoeuvre_controler(void)
{
  if (control_state != CONTROL_STATE_MANOEUVRE)
  {
    return;
  }

  int p_error = get_encA_count() - get_encB_count();
  int p_result = ((MAX_PWM_OUTPUT_DIFF_P_MANOEUVRE_CONTROLER * p_error) / 2) / MAX_STEP_DIFF_ERROR_VAL_P_CONTROLER;
  pwm_A = base_speed_pwm_A > 0 ? base_speed_pwm_A - p_result : base_speed_pwm_A + p_result;
  pwm_B = base_speed_pwm_B > 0 ? base_speed_pwm_B + p_result : base_speed_pwm_B - p_result;
}

static void manoeuvre(direction_t direction)
{
  switch (direction)
  {
  case FORWARD:
    start_manoeuvre(5, 5, 200);
    break;
  case RIGHT:
    start_manoeuvre(-10, 10, 200);
    break;
  case BACKWARD:
    start_manoeuvre(-21, 21, 200);
    break;
  case LEFT:
    start_manoeuvre(10, -10, 200);
    break;
  case GOAL_REACHED:
    ESP_LOGE(TAG, "error, this should not be reached, E100\n");

  default:
    break;
  }
}

static void junction_detector(void)
{
  if (control_state != CONTROL_STATE_LINE_FOLLOWING)
  {
    return;
  }
  static int route_instruction_idx = 0;
  static int on_junction_counter = 0;
  if (rs_values_mask_popcount > 3) // is on junction
  {
    on_junction_counter++;
  }
  else if (on_junction_counter > 0)
  {
    on_junction_counter--; // decreases until 0 when out of junction to take care of residual counter values for future junctions
  }

  if (on_junction_counter > 5)
  {
    // code for when on junction for sure
    if (route_instruction_idx < JUNCTION_NODES && route[route_instruction_idx] != GOAL_REACHED)
    {
      manoeuvre(route[route_instruction_idx]);
      route_instruction_idx++;
    }
    else if (route[route_instruction_idx] == GOAL_REACHED)
    {
      start_stopped_state();
    }

    on_junction_counter = 0;
    return;
  }
}

static void p_line_controler(void)
{
  // [ ] make controlers variable based on speed
  if (control_state != CONTROL_STATE_LINE_FOLLOWING)
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
  pwm_A = base_speed_pwm_A + p_result;
  pwm_B = base_speed_pwm_B - p_result;
}

static void update_rs_values(void)
{
  if (control_state != CONTROL_STATE_LINE_FOLLOWING)
  {
    return;
  }

  update_line_sensor_table(rs_values_table, &rs_values_mask, &rs_values_mask_popcount);
}

static void update_pwm(void)
{
  set_motors_duty(pwm_A, pwm_B);
}

void control_loop(void)
{
  update_rs_values();
  p_line_controler();
  junction_detector();

  p_manoeuvre_controler();
  check_steps();

  // uprint_rs_value_table();

  update_pwm();
}