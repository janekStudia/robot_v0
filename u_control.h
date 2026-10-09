#pragma once

void init_my_timer(void);

void update_pwm(void);

void uprint_rs_value_table();

void turn_left(void);

void turn_right(void);

void skip_junction(void);

void go_forward(int steps);

void manouver_ended(void);

void check_steps(void);

void p_manouver_controler(void);

void junction_detector(void);

void p_line_controler(void);

void update_rs_values(void);
