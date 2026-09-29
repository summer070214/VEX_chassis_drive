#ifndef CHASSIS_CONTROL_H
#define CHASSIS_CONTROL_H
#define ANGLE_SPEED_MAX 70
#define ANGLE_SPEED_MIN 15
#define MAX_SPEED 60        // 最高速度
#define MIN_SPEED 18       // 最低速度
#define ACCEL_DIST 12      // 加减速距离（cm）
void arrive_target_angle(float targetAngle);
void arrive_target_position(float targetX,float targetY);
#endif