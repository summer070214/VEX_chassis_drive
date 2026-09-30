#include "moto_control.h"
#include "robot_config.h"
#include "PID.h"
#include"sensor_read.h"
#include"locate_5225A.h"

void MotorEncoder_Init()  //将电动机的编码器各个参数重置为零
{
    left_chassis_1.resetPosition();
    left_chassis_2.resetPosition();
    left_chassis_3.resetPosition();

    right_chassis_1.resetPosition();
    right_chassis_2.resetPosition();
    right_chassis_3.resetPosition();
}
//未加速度环，可加，觉没必要·
void car_forword(float target_speed){
    left_chassis_1.spin(forward,target_speed,rpm);//针对电压控制电机输出  forward前进方向  volt为电压单位 
    left_chassis_2.spin(forward,target_speed,rpm);
    left_chassis_3.spin(forward,target_speed,rpm);
    left_chassis_4.spin(forward,target_speed,rpm);
    right_chassis_1.spin(forward,target_speed,rpm);
    right_chassis_2.spin(forward,target_speed,rpm);
    right_chassis_3.spin(forward,target_speed,rpm);
    right_chassis_4.spin(forward,target_speed,rpm);
}
void car_backword(float target_speed){
    left_chassis_1.spin(reverse,target_speed,rpm);//针对电压控制电机输出  forward前进方向  volt为电压单位 
    left_chassis_2.spin(reverse,target_speed,rpm);
    left_chassis_3.spin(reverse,target_speed,rpm);
    left_chassis_4.spin(reverse,target_speed,rpm);
    right_chassis_1.spin(reverse,target_speed,rpm);
    right_chassis_2.spin(reverse,target_speed,rpm);
    right_chassis_3.spin(reverse,target_speed,rpm);
    right_chassis_4.spin(reverse,target_speed,rpm);
}
void car_left(float target_speed){
    left_chassis_1.spin(reverse,target_speed,rpm);//针对电压控制电机输出  forward前进方向  volt为电压单位 
    left_chassis_2.spin(reverse,target_speed,rpm);
    left_chassis_3.spin(reverse,target_speed,rpm);
    left_chassis_4.spin(reverse,target_speed,rpm);
    right_chassis_1.spin(forward,target_speed,rpm);
    right_chassis_2.spin(forward,target_speed,rpm);
    right_chassis_3.spin(forward,target_speed,rpm);
    right_chassis_4.spin(forward,target_speed,rpm);
}
void car_right(float target_speed){
    left_chassis_2.spin(forward,target_speed,rpm);
    left_chassis_3.spin(forward,target_speed,rpm);
    left_chassis_4.spin(forward,target_speed,rpm);
    right_chassis_1.spin(forward,target_speed,rpm);
    right_chassis_1.spin(reverse,target_speed,rpm);
    right_chassis_2.spin(reverse,target_speed,rpm);
    right_chassis_3.spin(reverse,target_speed,rpm);
    right_chassis_4.spin(reverse,target_speed,rpm);
}

void car_stop(void)
{
    // 停止左侧所有电机
    left_chassis_1.stop(coast);   
    left_chassis_2.stop(coast);
    left_chassis_3.stop(coast);
    left_chassis_4.stop(coast);

    // 停止右侧所有电机
    right_chassis_1.stop(coast);
    right_chassis_2.stop(coast);
    right_chassis_3.stop(coast);
    right_chassis_4.stop(coast);
}
void car_left_angle(void)
{
    // 1. 初始化 PID（请把 Kp、Ki、Kd 改成你实际调试的值）
    pidinit(&angle_pid_data, 0, 0, 0,10,100);   // 示例参数，需自己调

    // 2. 记录起始角度，并计算目标角度（左转 90°）
    float start_angle = Inertial_Heading_Get();          // 0 ~ 360
    float target_angle = start_angle + 90.0f;

    // 处理超过 360 的情况
    if (target_angle >= 360.0f) {
        target_angle -= 360.0f;
    }

    // 3. 循环转向，直到接近目标
    while (1)
    {
        float current = Inertial_Heading_Get();

        // 计算最短角度误差（-180 ~ +180），避免跨越 0° 出问题
        float error = target_angle - current;
        if (error > 180.0f)  error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        // 到达目标（允许 ±1.5° 误差）
        if (fabsf(error) < 1.5f) {
            break;
        }

        // PID 计算输出速度
        float target_speed = pid_compute(&angle_pid_data, target_angle, current);

        // 限制速度范围（根据你的车调整）
        if (target_speed > 80)  target_speed = 80;
        if (target_speed < 15)  target_speed = 15;   // 最小速度，防止停转

        car_left(target_speed);   // 你的左转函数

        wait(10, msec);           // 给 PID 和控制一点时间
    }

    // 4. 到位后停车
    car_stop();   // 请确保你有停车函数，没有的话自己补上
}

    void car_right_angle(void)
{
    // 1. 初始化 PID（参数和左转保持一致，方便调试）
    pidinit(&angle_pid_data, 1.2f, 0.0f, 0.15f,10,100);   // 示例参数，需自己调

    // 2. 记录起始角度，计算目标角度（右转 90°）
    float start_angle = Inertial_Heading_Get();      // 0 ~ 360
    float target_angle = start_angle - 90.0f;

    // 处理小于 0 的情况
    if (target_angle < 0.0f) {
        target_angle += 360.0f;
    }

    // 3. 循环转向，直到接近目标
    while (1)
    {
        float current = Inertial_Heading_Get();

        // 计算最短角度误差（-180 ~ +180）
        float error = target_angle - current;
        if (error > 180.0f)  error -= 360.0f;
        if (error < -180.0f) error += 360.0f;

        // 到达目标（允许 ±1.5° 误差）
        if (fabsf(error) < 1.5f) {
            break;
        }

        // PID 计算输出速度
        float target_speed = pid_compute(&angle_pid_data, target_angle, current);

        // 限制速度范围
        if (target_speed > 80)  target_speed = 80;
        if (target_speed < 15)  target_speed = 15;

        car_right(target_speed);   // 你的右转函数

        wait(10, msec);
    }

    // 4. 到位后停车
    car_stop();

}