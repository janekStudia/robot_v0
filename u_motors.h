#pragma once

void sleep_motors(void);

void wake_up_motors(void);

void init_dual_pwm(void);

void init_motor_control(void);

void set_motor_A_forward(void);

void set_motor_B_forward(void);

void set_motor_A_backward(void);

void set_motor_B_backward(void);

void set_motors_duty(int duty_A, int duty_B);