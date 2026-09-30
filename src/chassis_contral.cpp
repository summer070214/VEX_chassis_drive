#include"chassis_contral.h"
#include"vex.h"
#include"vex_global.h"
#include"PID.h"
#include"locate_5225A.h"
#include "moto_control.h"
using namespace vex;
// 角度归一化到 -180 ~ +180
static float normalize_angle(float angle)
{
    while (angle > 180.0f)  angle -= 360.0f;
    while (angle < -180.0f) angle += 360.0f;
    return angle;
}

// 计算最短角度误差（-180 ~ +180）
static float angle_error(float target, float current)
{
    return normalize_angle(target - current);
}

// =========================================================
// 转到指定绝对角度（单位：度）
// targetAngle 建议传入 -180 ~ +180 范围
// =========================================================
void arrive_target_angle(float targetAngle)
{
    // 1. 把目标角度也归一化
    targetAngle = normalize_angle(targetAngle);

    // 2. 初始化角度 PID（参数请自己调试）
    pidinit(&angle_pid_data, 1.2f, 0.0f, 0.15f,10,1000);

    int timeout = 0;

    while (1)
    {
        // 更新定位（保证 cur_yaw_angle 是最新的）
        UpdateGlobalPosition();

        float current = position_data.cur_yaw_angle;
        float error   = angle_error(targetAngle, current);

        // 到位判断（允许 ±1.5° 误差）
        if (fabsf(error) < 1.5f) {
            break;
        }

        // PID 计算速度
        float speed = pid_compute(&angle_pid_data, targetAngle, current);

        // 速度限幅
        if (speed > ANGLE_SPEED_MAX)  speed = ANGLE_SPEED_MAX;
        if (speed < ANGLE_SPEED_MIN)  speed = ANGLE_SPEED_MIN;

        // 根据误差方向决定左转还是右转
        if (error > 0) {
            car_left(speed);     // 逆时针（角度增加）
        } else {
            car_right(speed);    // 顺时针（角度减小）
        }

        wait(10, msec);

        // 超时保护（约 4 秒）
        if (++timeout > 400) {
            break;
        }
    }

    // 到位后停车
    car_stop();
    wait(80, msec);   // 短暂稳定
}

// 沿当前车头方向移动指定距离（带简易梯形速度）
// distance > 0 前进，distance < 0 后退
static void move_distance(float distance)
{
    if (fabsf(distance) < 1.0f) return;

    UpdateGlobalPosition();
    float start_x = position_data.gxT;
    float start_y = position_data.gyT;

    float abs_dist   = fabsf(distance);
    

    // 距离环 PID（可选，参数自己调）
    pidinit(&distance_pid_data, 0.9f, 0.0f, 0.06f,10,1000);

    int timeout = 0;
    while (1)
    {
        UpdateGlobalPosition();

        float dx = position_data.gxT - start_x;
        float dy = position_data.gyT - start_y;
        float traveled = sqrtf(dx * dx + dy * dy);
        float remain   = abs_dist - traveled;

        if (remain < 1.2f) break;          // 到位

        // ---------- 梯形速度规划 ----------
        float speed;
        if (traveled < ACCEL_DIST) {
            // 加速段
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (traveled / ACCEL_DIST);
        }
        else if (remain < ACCEL_DIST) {
            // 减速段
            speed = MIN_SPEED + (MAX_SPEED - MIN_SPEED) * (remain / ACCEL_DIST);
        }
        else {
            speed = MAX_SPEED;             // 匀速段
        }

        // 用 PID 微调（可保留也可注释）
        float pid_speed = pid_compute(&distance_pid_data, abs_dist, traveled);
        if (pid_speed > speed) speed = pid_speed;

        // 限幅
        if (speed > MAX_SPEED) speed = MAX_SPEED;
        if (speed < MIN_SPEED) speed = MIN_SPEED;

        if (distance > 0)
            car_forword(speed);
        else
            car_backword(speed);

         wait(10, msec);

        if (++timeout > 900) break;        // 超时保护
    }

    car_stop();
    wait(80, msec);
}

// =========================================================
// 到达目标点 (targetX, targetY)
// 策略：先转到 X 方向走 X → 再转到 Y 方向走 Y
// =========================================================
void arrive_target_position(float targetX, float targetY)
{
    // 1. 获取当前坐标
    UpdateGlobalPosition();
    float currentX = position_data.gxT;
    float currentY = position_data.gyT;

    float deltaX = targetX - currentX;
    float deltaY = targetY - currentY;

    // 已经在目标点附近
    if (fabsf(deltaX) < 1.5f && fabsf(deltaY) < 1.5f) {
        car_stop();
        return;
    }

    // -------------------------------------------------
    // 第一步：走 X 轴
    // -------------------------------------------------
    if (fabsf(deltaX) > 1.5f)
    {
        // +X → 0°，-X → 180°
        float yaw_x = (deltaX >= 0) ? 0.0f : 180.0f;

        arrive_target_angle(yaw_x);        // 转到 X 方向
        move_distance(deltaX);             // 沿 X 移动
    }

    // -------------------------------------------------
    // 第二步：走 Y 轴
    // -------------------------------------------------
    UpdateGlobalPosition();                // 重新获取当前 Y
    deltaY = targetY - position_data.gyT;

    if (fabsf(deltaY) > 1.5f)
    {
        // +Y → 90°，-Y → -90°
        float yaw_y = (deltaY >= 0) ? 90.0f : -90.0f;

        arrive_target_angle(yaw_y);        // 转到 Y 方向
        move_distance(deltaY);             // 沿 Y 移动
    }

    // 最终停车
    car_stop();
}