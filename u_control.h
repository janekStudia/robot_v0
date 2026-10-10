#pragma once

void control_loop(void);

void uprint_rs_value_table();

void start_manoeuvre(int steps_A, int steps_B, int speed_pwm);

void start_stopped_state(void);

void start_line_following(int speed_pwm);
