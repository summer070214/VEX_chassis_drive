#ifndef MOTO_CONTROL_H
#define MOTO_CONTROL_H
void MotorEncoder_Init(void);

void car_forword(float target_speed);
void car_backword(float target_speed);
void car_left(float target_speed);
void car_right(float target_speed);
void car_stop(void);

void car_left_angle(void);
void car_right_angle(void);

#endif