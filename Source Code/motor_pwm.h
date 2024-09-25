
#ifndef MOTOR_PWM_H_
#define MOTOR_PWM_H_

// MOTOR RIGHT CONNECTED TO PORT A

#define MOTOR_RIGHT_PINF  3
#define MOTOR_RIGHT_PINB  4


// MOTOR LEFT CONNECTED TO PORT A

#define MOTOR_LEFT_PINF  6
#define MOTOR_LEFT_PINB  5



#define LOAD_VALUE_PWM 1600
// duty cycle formula ((100-duty_percent)*1600)/100;
#define DUTY_CYCLE (LOAD_VALUE_PWM/2)

void pwm_init_motor_left();
void pwm_init_motor_right();



void motors_move_forward();
void motors_move_right();
void motors_move_stop();
void set_duty_cycle(int , int);

#endif /* MOTOR_PWM_H_ */
